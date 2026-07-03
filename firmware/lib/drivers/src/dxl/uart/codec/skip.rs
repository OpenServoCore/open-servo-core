//! Universal byte-skip FSM + the drift-sampling gate. Owns the two pieces
//! of RX-poll policy state that aren't parser or ring bookkeeping: whether
//! we're mid-skip past a foreign/own packet body, and whether the in-flight
//! packet is an Instruction (the only kind that feeds drift samples, per
//! [[drift_sampling_instruction_only]]). The codec drives the ring itself —
//! this type holds no bytes, only the counter, deadline, and packet-kind
//! flag the poll loop consults.

/// Slack added past the byte-count-derived skip end, in wire bytes. Absorbs
/// inter-byte gaps and HSI wobble within healthy streams so a deadline-
/// bounded skip doesn't false-trigger on a slow-but-fine predecessor. 2
/// bytes ≈ 7 µs at 3 Mbaud — negligible vs. the host's own ~1 ms timeout,
/// tight enough that a truncated chain aborts well before the host's retry.
const SKIP_DEADLINE_SLACK_BYTES: u16 = 2;

struct SkipState {
    bytes_remaining: u16,
    id: u8,
    /// WireClock u32 tick at which the skip gives up and clears itself, so
    /// a truncated upstream packet can't bleed its uncounted bytes into
    /// the next packet on the wire. Set at skip entry to
    /// `now + (bytes_remaining + SKIP_DEADLINE_SLACK_BYTES) * frame_ticks`.
    /// Compared with `(now.wrapping_sub(deadline_tick) as i32) >= 0` —
    /// u32 modular signed-comparison works for any elapsed budget under
    /// 2³¹ ticks (~44 s at HCLK), comfortably above the slowest baud's
    /// longest packet.
    deadline_tick: u32,
}

/// Byte-skip counter + drift-sampling gate. `Some` skip between a sink-
/// requested [`super::PollAction::Skip`] and the matching
/// [`super::PollEvent::SkipComplete`]; `packet_is_instruction` tracks the
/// in-flight packet's kind across both the parser and skip paths.
pub(super) struct SkipFsm {
    skip: Option<SkipState>,
    /// Whether the in-flight packet is an Instruction — set at the
    /// Instruction Header, cleared at Status Header, cleared at packet
    /// boundary (Crc / Resync / SkipComplete). The drift walker only runs
    /// for Instruction packets so Status frames never contribute drift
    /// samples per [[drift_sampling_instruction_only]].
    packet_is_instruction: bool,
}

impl SkipFsm {
    pub(super) const fn new() -> Self {
        Self {
            skip: None,
            packet_is_instruction: false,
        }
    }

    // -- events -----------------------------------------------------------------

    /// A parsed Header classifies the in-flight packet. Instruction packets
    /// (own or foreign) feed drift samples; Status frames don't.
    pub(super) fn on_header(&mut self, is_instruction: bool) {
        self.packet_is_instruction = is_instruction;
    }

    /// Packet boundary — the in-flight classification is spent. Called at
    /// Crc / Resync and after a completed or dropped skip.
    pub(super) fn on_packet_end(&mut self) {
        self.packet_is_instruction = false;
    }

    /// Arm the universal byte-skip: `bytes_remaining` body+CRC bytes past
    /// the sink-rejected Header. The give-up deadline is derived here —
    /// `now + (bytes_remaining + SKIP_DEADLINE_SLACK_BYTES) · frame_ticks`
    /// — so a truncated upstream packet can't bleed uncounted bytes into
    /// the next. `frame_ticks` is one wire byte's duration in WireClock
    /// ticks at the current baud.
    pub(super) fn arm(&mut self, bytes_remaining: u16, id: u8, now: u32, frame_ticks: u32) {
        let budget_bytes = bytes_remaining.saturating_add(SKIP_DEADLINE_SLACK_BYTES);
        let elapsed = (budget_bytes as u32).wrapping_mul(frame_ticks);
        self.skip = Some(SkipState {
            bytes_remaining,
            id,
            deadline_tick: now.wrapping_add(elapsed),
        });
    }

    /// Consume up to `avail` ring bytes of the in-flight skip; returns the
    /// count the caller must advance the ring + wire cursor by. Decrements
    /// the internal counter; `0` when not skipping or the ring is empty.
    pub(super) fn take(&mut self, avail: u16) -> u16 {
        match self.skip.as_mut() {
            Some(s) => {
                let take = s.bytes_remaining.min(avail);
                s.bytes_remaining -= take;
                take
            }
            None => 0,
        }
    }

    /// Finish an exhausted skip: clear it and return the predecessor `id`
    /// for [`super::PollEvent::SkipComplete`]. `None` if not skipping.
    /// Leaves `packet_is_instruction` intact so the caller can consult
    /// [`Self::should_sample_drift`] before [`Self::on_packet_end`].
    pub(super) fn finish(&mut self) -> Option<u8> {
        self.skip.take().map(|s| s.id)
    }

    /// Drop the in-flight skip without emitting `SkipComplete`. Chain-cancel
    /// at the chip's TX-complete (doc §5.3) and the stale-deadline path both
    /// route here.
    pub(super) fn clear(&mut self) {
        self.skip = None;
    }

    // -- accessors --------------------------------------------------------------

    pub(super) fn is_skipping(&self) -> bool {
        self.skip.is_some()
    }

    /// The armed skip has drained all its bytes and is ready to finish.
    pub(super) fn is_exhausted(&self) -> bool {
        self.skip.as_ref().is_some_and(|s| s.bytes_remaining == 0)
    }

    /// The armed skip's give-up deadline has passed at `now`. `false` when
    /// not skipping.
    pub(super) fn deadline_passed(&self, now: u32) -> bool {
        self.skip
            .as_ref()
            .is_some_and(|s| (now.wrapping_sub(s.deadline_tick) as i32) >= 0)
    }

    /// The drift-sampling gate: `true` while the in-flight (or just-
    /// completed) packet is an Instruction. Single call site for both the
    /// parser Crc walk and the skip-completion walk.
    pub(super) fn should_sample_drift(&self) -> bool {
        self.packet_is_instruction
    }
}

//! Discrete-event core: the time domain, the event heap, and the single
//! half-duplex wire model (`docs/osc-native-protocol.md` §2). Everything that
//! is *not* per-servo state lives here; the [`super::Sim`] facade owns this
//! behind an `Rc<RefCell<_>>` so every provider can schedule into one queue.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use osc_core::BaudRate;

use super::{Source, WireFrame};

/// Transport tick domain (§ architecture): 48 ticks/µs makes every bit-time
/// integral at all four rates (3M=16, 2M=24, 1M=48, 0.5M=96 ticks/bit).
pub const TICKS_PER_US: u64 = 48;

/// 10 bit-times per UART character (8N1) — the byte and break unit (§2).
const BITS_PER_BYTE: u64 = 10;

/// SBK break length; measured ~14 bit-times, zero variance [F5]. The FE ring
/// byte and `on_break` are delivered at the break's *end*, which is when the
/// line has risen for an ordinary break [F5] — so `LineSense::is_low` reads
/// high there and only a still-dominant rescue pulse (§9.1) tests low.
const BREAK_BITS: u64 = 14;

/// Runaway guards: a wedged scenario must fail loudly, not spin forever.
const MAX_EVENTS: u64 = 10_000_000;
const MAX_SIM_TICKS: u64 = 60 * 1_000_000 * TICKS_PER_US;

/// Ticks for one bit / byte / break at `baud` — integral by TICKS_PER_US choice.
#[inline]
pub fn bit_ticks(baud: BaudRate) -> u64 {
    TICKS_PER_US * 1_000_000 / baud.as_hz() as u64
}
#[inline]
pub fn byte_ticks(baud: BaudRate) -> u64 {
    bit_ticks(baud) * BITS_PER_BYTE
}
#[inline]
pub fn break_ticks(baud: BaudRate) -> u64 {
    bit_ticks(baud) * BREAK_BITS
}

/// Who is driving the wire (§2 drive discipline). `Host` schedules the bus;
/// `Servo(index)` is a replying node — index, not id, so the F8 assert names
/// an unambiguous node even mid-id-change.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Talker {
    Host,
    Servo(usize),
}

/// A scheduled wire/servo event. Ordered by `(time, seq)` for FIFO tie-breaks.
pub enum Event {
    /// Break FE point: ring one byte at every listener and fire `on_break`.
    /// `break_start` is the recorded frame's `at`; delivery is at break end.
    WireBreak { talker: Talker, break_start: u64 },
    /// One data byte at its stop-bit sample point. A listener whose baud
    /// differs from `baud` receives it as garble (ring + `on_break`) instead.
    WireData {
        talker: Talker,
        byte: u8,
        baud: BaudRate,
    },
    /// A lone framing-error byte, no frame around it (§ line noise, F4).
    WireGarble { byte: u8 },
    /// A rescue pulse's FE point: one FE at every servo, line still low (§9.1).
    RescuePulse,
    /// The host's frame drained — finalize the recorded frame.
    HostFrameEnd,
    /// A servo's tick-compare fired; `generation` guards against a cancelled/re-armed
    /// deadline (stale generations are dropped on delivery).
    Compare { servo: usize, generation: u64 },
    /// A servo TX DMA arm completed — drive `on_tx_complete`.
    TxArmDone { servo: usize },
    /// A servo's handler body ended (`super::cpu`): deliver one pended vector.
    CpuFree { servo: usize },
}

struct Scheduled {
    time: u64,
    seq: u64,
    event: Event,
}

impl PartialEq for Scheduled {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time && self.seq == other.seq
    }
}
impl Eq for Scheduled {}
impl Ord for Scheduled {
    // Reversed: `BinaryHeap` is a max-heap, we want the earliest `(time, seq)`.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .time
            .cmp(&self.time)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}
impl PartialOrd for Scheduled {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// A frame being assembled on the wire (half-duplex → at most one live).
struct PendingFrame {
    at: u64,
    end: u64,
    talker: Talker,
    bytes: Vec<u8>,
}

pub struct Core {
    heap: BinaryHeap<Scheduled>,
    seq: u64,
    now: u64,
    processed: u64,
    host_baud: BaudRate,
    // Drive discipline: the current owner and how far its claim reaches.
    busy: Option<(Talker, u64)>,
    // Dominant-low intervals for `LineSense` (breaks + rescue pulses).
    dominant: Vec<(u64, u64)>,
    pending: Option<PendingFrame>,
    recorded: Vec<WireFrame>,
}

impl Core {
    pub fn new(host_baud: BaudRate) -> Self {
        Self {
            heap: BinaryHeap::new(),
            seq: 0,
            now: 0,
            processed: 0,
            host_baud,
            busy: None,
            dominant: Vec::new(),
            pending: None,
            recorded: Vec::new(),
        }
    }

    pub fn now(&self) -> u64 {
        self.now
    }

    pub fn host_baud(&self) -> BaudRate {
        self.host_baud
    }

    pub fn set_host_baud(&mut self, baud: BaudRate) {
        self.host_baud = baud;
    }

    /// Queue `event` at absolute tick `time`; FIFO among equal times.
    pub fn schedule(&mut self, event: Event, time: u64) {
        let seq = self.seq;
        self.seq += 1;
        self.heap.push(Scheduled { time, seq, event });
    }

    /// Pop the earliest event, advancing the clock and tripping the runaway
    /// guards. Returns `None` when the queue drains.
    pub fn pop(&mut self) -> Option<Event> {
        let Scheduled { time, event, .. } = self.heap.pop()?;
        // DES invariant: the clock never rewinds. Every scheduler clamps to
        // `now` (the CPU occupancy model reads busy-windows against it).
        assert!(
            time >= self.now,
            "sim clock rewind: event at t={time} after now={}",
            self.now
        );
        self.now = time;
        self.processed += 1;
        assert!(
            self.processed <= MAX_EVENTS,
            "sim runaway: >{MAX_EVENTS} events"
        );
        assert!(self.now <= MAX_SIM_TICKS, "sim runaway: >60 s of sim time");
        Some(event)
    }

    // --- wire drive discipline (§2, F8) -----------------------------------

    /// Claim the wire for `talker` over `[start, end)`. A different talker
    /// overlapping an existing claim is the T_turn regression — panic loudly.
    pub fn claim(&mut self, talker: Talker, start: u64, end: u64) {
        if let Some((owner, owner_end)) = self.busy
            && owner != talker
            && start < owner_end
        {
            panic!(
                "drive discipline (F8): {talker:?} claims the wire at t={start} \
                 while {owner:?} holds it until t={owner_end}"
            );
        }
        let reach = match self.busy {
            Some((owner, owner_end)) if owner == talker => owner_end.max(end),
            _ => end,
        };
        self.busy = Some((talker, reach));
    }

    /// Mark `[start, end)` dominant-low for `LineSense` (break or rescue pulse).
    pub fn hold_low(&mut self, start: u64, end: u64) {
        self.dominant.push((start, end));
    }

    pub fn is_low(&self, at: u64) -> bool {
        self.dominant.iter().any(|&(s, e)| s <= at && at < e)
    }

    // --- wire recorder ----------------------------------------------------

    /// Begin the frame a break opens; its ring image starts with the 0x00
    /// break byte (§3.2 prefix). Overlap here means the wire model let two
    /// frames coexist — a harness invariant break.
    pub fn begin_frame(&mut self, at: u64, talker: Talker) {
        assert!(
            self.pending.is_none(),
            "wire recorder: frame began mid-frame"
        );
        self.pending = Some(PendingFrame {
            at,
            end: self.now,
            talker,
            bytes: vec![0x00],
        });
    }

    pub fn append_byte(&mut self, byte: u8, at: u64) {
        if let Some(p) = self.pending.as_mut() {
            p.bytes.push(byte);
            p.end = at;
        }
    }

    pub fn finalize_frame(&mut self) {
        let Some(p) = self.pending.take() else {
            return;
        };
        let from = match p.talker {
            Talker::Host => Source::Host,
            // The servo encodes its live id into byte 1 — id-at-send-time.
            Talker::Servo(_) => Source::Servo(p.bytes.get(1).copied().unwrap_or(0)),
        };
        self.recorded.push(WireFrame {
            at: p.at,
            end: p.end,
            from,
            bytes: p.bytes,
        });
    }

    /// Drain every frame recorded since the last drain, in wire order.
    pub fn take_recorded(&mut self) -> Vec<WireFrame> {
        let mut out = std::mem::take(&mut self.recorded);
        out.sort_by_key(|f| f.at);
        out
    }
}

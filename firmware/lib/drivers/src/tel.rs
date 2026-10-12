//! TEL burst seam between the kernel fast tick (PFIC HIGH) and the bus-side
//! frame stager (the transport ISRs; a banked batch wakes the stager).
//! The two sides never share `&mut`: the channel is a pair of ping-pong
//! payload buffers the kernel encodes into directly: each row lands at
//! its final wire offset via the core row encoder, so no intermediate
//! sample storage exists.
//! Cross-side traffic is single-writer-per-field volatile load/store plus
//! compiler fences at the buffer handoffs (single core, no atomic RMW on
//! rv32ec).
//!
//! Buffer ownership rides the `ready` tags: 0 = kernel's to fill, else the
//! arm epoch the batch was produced under -- the bus stages only its own
//! epoch and discards strays, which closes the mid-tick re-arm race (a
//! preempted `on_tick` finishing a stale batch publishes a dead epoch).

use core::cell::SyncUnsafeCell;
use core::sync::atomic::{Ordering, compiler_fence};

use osc_servo_core::tel::{
    RowPlan, STREAM_HDR, STREAM_PAYLOAD_MAX, STREAM_SAMPLES_MAX, STREAM_TRAILER, TelSample,
    TelStream, encode_stream_head, encode_stream_trailer,
};

use crate::traits::bus::StagerWake;

/// One finished frame's metadata, kernel-written at batch finalize (before
/// its `ready` tag), bus-read while the tag holds.
#[derive(Copy, Clone)]
pub(crate) struct BufMeta {
    pub(crate) len: u16,
    /// The first row's fault bit -> the frame's INST ALERT.
    pub(crate) alert: bool,
    pub(crate) last: bool,
}

const META_ZERO: BufMeta = BufMeta {
    len: 0,
    alert: false,
    last: false,
};

/// The shared storage. Field writers: `bufs`/`meta`/`ready`(set)/`drops`
/// belong to the kernel side ([`TelFeed`]); `active`, the arm mailbox, and
/// `ready`(clear) to the bus side ([`TelDrain`]). The dual-writer `ready`
/// tags are safe: each side stores only constants its protocol phase owns.
pub struct TelChannel {
    bufs: [SyncUnsafeCell<[u8; STREAM_PAYLOAD_MAX]>; 2],
    meta: [SyncUnsafeCell<BufMeta>; 2],
    ready: [SyncUnsafeCell<u8>; 2],
    /// Burst gate the producer polls each tick.
    active: SyncUnsafeCell<bool>,
    arm_mask: SyncUnsafeCell<u16>,
    /// Row plans the arm builds and the pickup copies: an arm writes the
    /// slot the last published arm does not name and posts its index, so
    /// the kernel, which the bus never preempts, never copies a slot
    /// mid-write, however many arms land between two ticks.
    plans: [SyncUnsafeCell<RowPlan>; 2],
    arm_slot: SyncUnsafeCell<u8>,
    arm_count: SyncUnsafeCell<u16>,
    /// Mailbox epoch, written last (fence-ordered), never 0; the kernel
    /// double-reads it around the payload to reject a torn pickup.
    arm_seq: SyncUnsafeCell<u8>,
    /// Samples dropped while both buffers were ready, monotonic wrapping.
    drops: SyncUnsafeCell<u16>,
}

impl TelChannel {
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            bufs: [
                SyncUnsafeCell::new([0; STREAM_PAYLOAD_MAX]),
                SyncUnsafeCell::new([0; STREAM_PAYLOAD_MAX]),
            ],
            meta: [
                SyncUnsafeCell::new(META_ZERO),
                SyncUnsafeCell::new(META_ZERO),
            ],
            ready: [SyncUnsafeCell::new(0), SyncUnsafeCell::new(0)],
            active: SyncUnsafeCell::new(false),
            arm_mask: SyncUnsafeCell::new(0),
            plans: [const { SyncUnsafeCell::new(RowPlan::EMPTY) }; 2],
            arm_slot: SyncUnsafeCell::new(0),
            arm_count: SyncUnsafeCell::new(0),
            arm_seq: SyncUnsafeCell::new(0),
            drops: SyncUnsafeCell::new(0),
        }
    }

    /// The bus-side burst gate. Any context: the bus is its only writer.
    pub fn active(&self) -> bool {
        // SAFETY: single-byte volatile read of a bus-written flag.
        unsafe { self.active.get().read_volatile() }
    }

    /// Rows dropped against full buffers, monotonic wrapping. Any context:
    /// the kernel side is the counter's only writer.
    pub fn drops(&self) -> u16 {
        // SAFETY: single-word volatile read of a kernel-written counter.
        unsafe { self.drops.get().read_volatile() }
    }

    /// Split into the two halves. Call once at bringup: a second feed or
    /// drain breaks the single-writer discipline.
    pub fn split<W: StagerWake>(&'static self, stager: W) -> (TelFeed<W>, TelDrain<W>) {
        (
            TelFeed {
                ch: self,
                stager: stager.clone(),
                mask: 0,
                plan: RowPlan::EMPTY,
                remaining: 0,
                seq: 0,
                epoch: 0,
                idx: 0,
                at: STREAM_HDR,
                n: 0,
                valid: 0,
                alert: false,
                lead_alert: false,
            },
            TelDrain {
                ch: self,
                slot: 0,
                stager,
            },
        )
    }
}

/// Kernel-side half: the fast tick's [`TelStream`] sink and the incremental
/// frame encoder. All fields but `ch` and `stager` are kernel-private
/// encoder state.
pub struct TelFeed<W: StagerWake> {
    ch: &'static TelChannel,
    stager: W,
    mask: u16,
    plan: RowPlan,
    remaining: u16,
    seq: u8,
    /// Arm epoch this burst produces under (`arm_seq` at pickup).
    epoch: u8,
    idx: usize,
    at: usize,
    n: u32,
    valid: u32,
    alert: bool,
    lead_alert: bool,
}

impl<W: StagerWake> TelFeed<W> {
    /// The bus-side burst gate, as the feed sees it.
    pub fn active(&self) -> bool {
        self.ch.active()
    }

    /// Consume a posted arm: seq moved and the payload read back consistent
    /// (a torn read -- the mailbox rewritten mid-pickup -- retries next
    /// tick). A pickup resets the whole encoder.
    fn poll_arm(&mut self) {
        // SAFETY: bus-written mailbox seq, volatile-read (type doc).
        let seq = unsafe { self.ch.arm_seq.get().read_volatile() };
        if seq != self.epoch {
            self.pickup(seq);
        }
    }

    #[cold]
    #[inline(never)]
    fn pickup(&mut self, seq: u8) {
        let ch = self.ch;
        // SAFETY: mailbox fields are bus-written, volatile-read here; the
        // seq double-read brackets the payload (type doc).
        let (mask, slot, count) = unsafe {
            let mask = ch.arm_mask.get().read_volatile();
            let slot = ch.arm_slot.get().read_volatile() as usize & 1;
            let count = ch.arm_count.get().read_volatile();
            if ch.arm_seq.get().read_volatile() != seq {
                return;
            }
            (mask, slot, count)
        };
        compiler_fence(Ordering::Acquire);
        // SAFETY: the published arm's slot, which no arm writes (type doc):
        // a plain word copy.
        let plan = unsafe { *ch.plans[slot].get() };
        self.epoch = seq;
        self.mask = mask;
        self.plan = plan;
        self.remaining = if plan.is_empty() { 0 } else { count };
        self.seq = 0;
        self.idx = 0;
        self.at = STREAM_HDR;
        self.n = 0;
        self.valid = 0;
        self.alert = false;
    }
}

impl<W: StagerWake> TelStream for TelFeed<W> {
    fn row_mask(&mut self) -> u16 {
        if !self.ch.active() {
            return 0;
        }
        self.poll_arm();
        if self.remaining == 0 { 0 } else { self.mask }
    }

    /// Out of line: the row's whole cost is its own, one call from the
    /// kernel, never the kernel's register pressure.
    #[inline(never)]
    fn on_tick(&mut self, sample: &TelSample) {
        if self.remaining == 0 {
            return;
        }
        let ch = self.ch;
        let idx = self.idx & 1;
        // SAFETY: single-writer discipline (type doc). The buffer `&mut` is
        // exclusive while its ready tag is 0 (the bus touches only tagged
        // buffers).
        unsafe {
            if ch.ready[idx].get().read_volatile() != 0 {
                // Consumer stalled: never block the fast tick.
                let d = ch.drops.get();
                d.write_volatile(d.read_volatile().wrapping_add(1));
                return;
            }
            let buf = &mut *ch.bufs[idx].get();
            if self.n == 0 {
                let last = self.remaining as usize <= STREAM_SAMPLES_MAX;
                encode_stream_head(self.seq, last, sample.tick, buf);
                self.lead_alert = sample.fault;
            }
            let at = self.at;
            self.at = at + self.plan.len();
            self.valid |= (sample.window_valid as u32) << self.n;
            self.alert |= sample.fault;
            self.n += 1;
            self.remaining -= 1;
            if let Some(row) = buf.get_mut(at..) {
                self.plan.encode(sample, row);
            }
        }
        if self.n as usize == STREAM_SAMPLES_MAX || self.remaining == 0 {
            self.bank(idx);
        }
    }
}

impl<W: StagerWake> TelFeed<W> {
    /// Close the batch in buffer `idx`: trailer, metadata, its ready tag,
    /// then the stager's wake.
    #[inline(never)]
    fn bank(&mut self, idx: usize) {
        let ch = self.ch;
        let last = self.remaining == 0;
        // SAFETY: single-writer discipline (type doc); the tag store is
        // fenced behind the payload and metadata writes.
        unsafe {
            if let Some(out) = (*ch.bufs[idx].get()).get_mut(self.at..) {
                encode_stream_trailer(self.valid, self.alert, out);
            }
            ch.meta[idx].get().write_volatile(BufMeta {
                len: (self.at + STREAM_TRAILER) as u16,
                alert: self.lead_alert,
                last,
            });
            // A re-arm that landed mid-batch abandons the batch: its
            // pickup next tick restarts the encoder, and the epoch tag
            // would be dead at the bus anyway.
            if ch.arm_seq.get().read_volatile() == self.epoch {
                compiler_fence(Ordering::Release);
                ch.ready[idx].get().write_volatile(self.epoch);
                self.stager.wake();
                crate::bench::tel_banked();
                self.seq = self.seq.wrapping_add(1);
                self.idx ^= 1;
            }
        }
        self.at = STREAM_HDR;
        self.n = 0;
        self.valid = 0;
        self.alert = false;
    }
}

/// Bus-side half: posts arms, gates the producer, consumes ready buffers.
pub struct TelDrain<W: StagerWake> {
    ch: &'static TelChannel,
    /// The plan slot the last posted arm named.
    slot: u8,
    stager: W,
}

impl<W: StagerWake> TelDrain<W> {
    /// Post an arm to the kernel side; returns its epoch (never 0). A mask
    /// with no row in the buffers posts an empty plan, which parks the
    /// burst instead of overrunning them.
    pub(crate) fn send_arm(&mut self, mask: u16, count: u16) -> u8 {
        let ch = self.ch;
        let plan = RowPlan::new(mask).unwrap_or(RowPlan::EMPTY);
        self.slot ^= 1;
        // SAFETY: sole mailbox writer (type doc); payload lands before the
        // fenced seq store the kernel keys on, in the slot the published
        // arm does not name.
        unsafe {
            ch.plans[self.slot as usize & 1].get().write_volatile(plan);
            ch.arm_mask.get().write_volatile(mask);
            ch.arm_slot.get().write_volatile(self.slot);
            ch.arm_count.get().write_volatile(count);
            let mut seq = ch.arm_seq.get().read_volatile().wrapping_add(1);
            if seq == 0 {
                seq = 1;
            }
            compiler_fence(Ordering::Release);
            ch.arm_seq.get().write_volatile(seq);
            seq
        }
    }

    pub(crate) fn set_active(&mut self, on: bool) {
        // SAFETY: sole writer of `active` (type doc).
        unsafe { self.ch.active.get().write_volatile(on) };
    }

    /// Free both buffers (arm/abort: stale batches never stage).
    pub(crate) fn clear_ready(&mut self) {
        // SAFETY: ready-clear is the bus's store (type doc).
        unsafe {
            self.ch.ready[0].get().write_volatile(0);
            self.ch.ready[1].get().write_volatile(0);
        }
    }

    /// Metadata of buffer `idx` if it is ready under `epoch`; a stray epoch
    /// (dead burst) is discarded on sight.
    pub(crate) fn ready(&mut self, idx: usize, epoch: u8) -> Option<BufMeta> {
        let ch = self.ch;
        // SAFETY: tag read gates the meta read behind an acquire fence;
        // ready-clear is the bus's store (type doc).
        unsafe {
            let tag = ch.ready[idx & 1].get().read_volatile();
            if tag == 0 {
                return None;
            }
            if tag != epoch {
                ch.ready[idx & 1].get().write_volatile(0);
                return None;
            }
            compiler_fence(Ordering::Acquire);
            Some(ch.meta[idx & 1].get().read_volatile())
        }
    }

    /// Borrow a ready buffer's payload for staging.
    pub(crate) fn payload(&self, idx: usize) -> &[u8; STREAM_PAYLOAD_MAX] {
        // SAFETY: the caller holds the buffer's ready tag (`ready` returned
        // Some); the kernel writes only untagged buffers.
        unsafe { &*self.ch.bufs[idx & 1].get() }
    }

    /// Return buffer `idx` to the kernel (after the frame's TX released).
    pub(crate) fn release(&mut self, idx: usize) {
        // SAFETY: ready-clear is the bus's store (type doc).
        unsafe { self.ch.ready[idx & 1].get().write_volatile(0) };
    }

    pub(crate) fn wake(&self) {
        self.stager.wake();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::bus::FakeStager;
    use osc_servo_core::tel::{FLAG_LAST, encode_stream};
    const MASK_SIX: u16 = 0x3F;

    type Feed = TelFeed<FakeStager>;
    type Drain = TelDrain<FakeStager>;

    /// The kernel's per-tick sequence: ask for the row, then hand it over.
    fn tick(feed: &mut Feed, s: &TelSample) {
        if feed.row_mask() != 0 {
            feed.on_tick(s);
        }
    }

    fn leaked() -> &'static TelChannel {
        std::boxed::Box::leak(std::boxed::Box::new(TelChannel::new()))
    }

    fn channel() -> (Feed, Drain) {
        leaked().split(FakeStager::default())
    }

    fn sample(i: u16) -> TelSample {
        TelSample {
            pos: 0x4000 + i,
            current: -(i as i16),
            tick: 0xFFF0u16.wrapping_add(i),
            window_valid: i.is_multiple_of(2),
            ..Default::default()
        }
    }

    fn arm(drain: &mut Drain, mask: u16, count: u16) -> u8 {
        let epoch = drain.send_arm(mask, count);
        drain.clear_ready();
        drain.set_active(true);
        epoch
    }

    #[test]
    fn active_flag_is_the_bus_gate() {
        let (feed, mut drain) = channel();
        assert!(!feed.active());
        drain.set_active(true);
        assert!(feed.active());
        drain.set_active(false);
        assert!(!feed.active());
    }

    #[test]
    fn batches_match_encode_stream_byte_for_byte() {
        let (mut feed, mut drain) = channel();
        let epoch = arm(&mut drain, MASK_SIX, 20);
        let samples: std::vec::Vec<TelSample> = (0..20).map(sample).collect();
        for s in &samples {
            tick(&mut feed, s);
        }

        let mut want = [0u8; STREAM_PAYLOAD_MAX];
        let n0 = encode_stream(MASK_SIX, 0, false, &samples[..16], &mut want);
        let m0 = drain.ready(0, epoch).expect("first batch ready");
        assert_eq!((m0.len as usize, m0.last), (n0, false));
        assert_eq!(drain.payload(0)[..n0], want[..n0]);

        let n1 = encode_stream(MASK_SIX, 1, true, &samples[16..], &mut want);
        let m1 = drain.ready(1, epoch).expect("last batch ready");
        assert_eq!((m1.len as usize, m1.last), (n1, true));
        assert_eq!(drain.payload(1)[..n1], want[..n1]);
        assert_eq!(drain.payload(1)[1], FLAG_LAST);
    }

    /// The head is row 0's: LAST is known once the burst's remaining rows
    /// fit the batch, the stamp and INST ALERT are row 0's own. The trailer
    /// lands at the bank with the batch's fault OR.
    #[test]
    fn head_lands_at_row_0_and_trailer_at_the_bank() {
        use osc_protocol::tel::StreamFrame;
        let (mut feed, mut drain) = channel();
        let epoch = arm(&mut drain, MASK_SIX, 32);
        let mut s: std::vec::Vec<TelSample> = (0..32).map(sample).collect();
        s[5].fault = true;
        s[16].fault = true;
        tick(&mut feed, &s[0]);
        assert_eq!(drain.payload(0)[..STREAM_HDR], [0, 0, 0xF0, 0xFF]);
        for x in &s[1..17] {
            tick(&mut feed, x);
        }
        assert_eq!(drain.payload(1)[..STREAM_HDR], [1, FLAG_LAST, 0x00, 0x00]);

        let m0 = drain.ready(0, epoch).expect("first batch ready");
        assert!(!m0.alert, "row 0 ran clean");
        let f0 = StreamFrame::parse(MASK_SIX, &drain.payload(0)[..m0.len as usize]);
        let f0 = f0.expect("a frame");
        assert_eq!(
            (f0.len(), f0.tick, f0.last, f0.alert),
            (16, 0xFFF0, false, true)
        );

        for x in &s[17..] {
            tick(&mut feed, x);
        }
        let m1 = drain.ready(1, epoch).expect("last batch ready");
        assert!(m1.alert && m1.last, "row 0 faulted");
        let f1 = StreamFrame::parse(MASK_SIX, &drain.payload(1)[..m1.len as usize]);
        let f1 = f1.expect("a frame");
        assert_eq!((f1.len(), f1.tick, f1.valid), (16, 0x0000, 0x5555));
    }

    #[test]
    fn stalled_consumer_drops_and_counts_and_the_stamp_jumps() {
        let ch = leaked();
        let (mut feed, mut drain) = ch.split(FakeStager::default());
        let epoch = arm(&mut drain, MASK_SIX, 200);
        for i in 0..37 {
            tick(&mut feed, &sample(i));
        }
        // both buffers ready, 5 samples dropped against them
        assert_eq!(ch.drops(), 5);
        // release one: production resumes into it
        assert!(drain.ready(0, epoch).is_some());
        drain.release(0);
        for i in 0..16 {
            tick(&mut feed, &sample(100 + i));
        }
        assert_eq!(ch.drops(), 5);
        let m = drain.ready(0, epoch).expect("refilled batch");
        assert!(!m.last);
        // the dropped rows show as a stamp jump, not as a seq hole
        let p = drain.payload(0);
        assert_eq!(p[0], 2);
        assert_eq!(p[2..4], sample(100).tick.to_le_bytes());
    }

    /// Arms that land between two ticks alternate plan slots: the pickup
    /// takes the last one's plan whole, whatever the arms before it wrote.
    #[test]
    fn back_to_back_arms_take_the_latest_plan() {
        use osc_servo_core::tel::{BIT_CURRENT, BIT_POS};
        let (mut feed, mut drain) = channel();
        drain.send_arm(MASK_SIX, 16);
        drain.send_arm(BIT_POS | BIT_CURRENT, 16);
        let epoch = arm(&mut drain, BIT_POS, 2);
        let samples = [sample(1), sample(2)];
        for s in &samples {
            tick(&mut feed, s);
        }
        let mut want = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(BIT_POS, 0, true, &samples, &mut want);
        let m = drain.ready(0, epoch).expect("the last arm's batch");
        assert_eq!(m.len as usize, n);
        assert_eq!(drain.payload(0)[..n], want[..n]);
    }

    #[test]
    fn rearm_discards_the_old_epoch() {
        let (mut feed, mut drain) = channel();
        let e1 = arm(&mut drain, MASK_SIX, 16);
        for i in 0..16 {
            tick(&mut feed, &sample(i));
        }
        assert!(drain.ready(0, e1).is_some());

        // new burst: the un-staged old batch is a stray now
        let e2 = arm(&mut drain, MASK_SIX, 5);
        assert!(drain.ready(0, e2).is_none(), "old epoch discarded");
        for i in 0..5 {
            tick(&mut feed, &sample(50 + i));
        }
        let m = drain.ready(0, e2).expect("fresh burst lands in buffer 0");
        assert!(m.last);
        // seq restarted at 0
        assert_eq!(drain.payload(0)[0], 0);
    }

    #[test]
    fn count_zero_arm_parks_the_encoder() {
        let ch = leaked();
        let (mut feed, mut drain) = ch.split(FakeStager::default());
        arm(&mut drain, MASK_SIX, 32);
        for i in 0..4 {
            tick(&mut feed, &sample(i));
        }
        let e = drain.send_arm(MASK_SIX, 0);
        drain.clear_ready();
        drain.set_active(false);
        for i in 0..40 {
            tick(&mut feed, &sample(i));
        }
        assert!(drain.ready(0, e).is_none());
        assert!(drain.ready(1, e).is_none());
        assert_eq!(ch.drops(), 0);
    }

    /// The stager's only kernel-side event: each published batch wakes it
    /// once, after its ready tag. Rows, drops and an abandoned batch never
    /// wake it.
    #[test]
    fn each_banked_batch_wakes_the_stager_once() {
        let ch = leaked();
        let stager = FakeStager::default();
        let (mut feed, mut drain) = ch.split(stager.clone());
        let epoch = arm(&mut drain, MASK_SIX, 40);
        for i in 0..15 {
            tick(&mut feed, &sample(i));
        }
        assert_eq!(stager.wakes(), 0, "rows alone never wake");
        tick(&mut feed, &sample(15));
        assert_eq!(stager.wakes(), 1);
        assert!(drain.ready(0, epoch).is_some(), "tag lands with the wake");
        for i in 16..48 {
            tick(&mut feed, &sample(i));
        }
        assert_eq!(stager.wakes(), 2, "the stalled consumer's drops: none");
        assert!(ch.drops() > 0);

        // An arm that moved before the bank abandons the batch unpublished.
        drain.release(0);
        drain.release(1);
        for i in 0..7 {
            tick(&mut feed, &sample(i));
        }
        drain.send_arm(MASK_SIX, 40);
        feed.on_tick(&sample(7));
        assert_eq!(stager.wakes(), 2, "abandoned batch");
    }
}

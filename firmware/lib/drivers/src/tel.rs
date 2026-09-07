//! TEL burst seam between the kernel fast tick (PFIC LOW) and the bus-side
//! frame stager (HIGH transport ISRs / ISR-masked main loop). The two sides
//! never share `&mut`: the channel is a pair of ping-pong payload buffers
//! the kernel encodes into DIRECTLY -- each sample lands at its final wire
//! offset via the core appenders, so no intermediate sample storage exists.
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
    STREAM_HDR, STREAM_PAYLOAD_MAX, STREAM_SAMPLES_MAX, TelSample, TelStream, encode_sample,
    encode_stream_hdr,
};

/// One finished frame's metadata, kernel-written at batch finalize (before
/// its `ready` tag), bus-read while the tag holds.
#[derive(Copy, Clone)]
pub(crate) struct BufMeta {
    pub(crate) len: u16,
    /// OR of the batch's fault bits -> the frame's INST ALERT.
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
            arm_count: SyncUnsafeCell::new(0),
            arm_seq: SyncUnsafeCell::new(0),
            drops: SyncUnsafeCell::new(0),
        }
    }

    /// Split into the two halves. Call once at bringup: a second feed or
    /// drain breaks the single-writer discipline.
    pub fn split(&'static self) -> (TelFeed, TelDrain) {
        (
            TelFeed {
                ch: self,
                mask: 0,
                remaining: 0,
                seq: 0,
                epoch: 0,
                idx: 0,
                at: STREAM_HDR,
                n: 0,
                valid: 0,
                alert: false,
            },
            TelDrain { ch: self },
        )
    }
}

/// Kernel-side half: the fast tick's [`TelStream`] sink and the incremental
/// frame encoder. All fields but `ch` are kernel-private encoder state.
pub struct TelFeed {
    ch: &'static TelChannel,
    mask: u16,
    remaining: u16,
    seq: u8,
    /// Arm epoch this burst produces under (`arm_seq` at pickup).
    epoch: u8,
    idx: usize,
    at: usize,
    n: u32,
    valid: u16,
    alert: bool,
}

impl TelFeed {
    /// Consume a posted arm: seq moved and the payload read back consistent
    /// (a torn read -- the mailbox rewritten mid-pickup -- retries next
    /// tick). A pickup resets the whole encoder.
    fn poll_arm(&mut self) {
        let ch = self.ch;
        // SAFETY: mailbox fields are bus-written, volatile-read here; the
        // seq double-read brackets the payload (type doc).
        unsafe {
            let seq = ch.arm_seq.get().read_volatile();
            if seq == self.epoch {
                return;
            }
            let mask = ch.arm_mask.get().read_volatile();
            let count = ch.arm_count.get().read_volatile();
            if ch.arm_seq.get().read_volatile() != seq {
                return;
            }
            self.epoch = seq;
            self.mask = mask;
            self.remaining = count;
            self.seq = 0;
            self.idx = 0;
            self.at = STREAM_HDR;
            self.n = 0;
            self.valid = 0;
            self.alert = false;
        }
    }

    /// Samples dropped against full buffers, monotonic wrapping.
    pub fn drops(&self) -> u16 {
        // SAFETY: own counter, volatile read.
        unsafe { self.ch.drops.get().read_volatile() }
    }
}

impl TelStream for TelFeed {
    fn active(&self) -> bool {
        // SAFETY: bus-written flag, volatile read (type doc).
        unsafe { self.ch.active.get().read_volatile() }
    }

    fn on_tick(&mut self, sample: &TelSample) {
        self.poll_arm();
        if self.remaining == 0 {
            return;
        }
        let ch = self.ch;
        let idx = self.idx & 1;
        // SAFETY: single-writer discipline (type doc). The buffer `&mut` is
        // exclusive while its ready tag is 0 (the bus touches only tagged
        // buffers); the tag store is fenced behind the payload writes.
        unsafe {
            if ch.ready[idx].get().read_volatile() != 0 {
                // Consumer stalled: never block the fast tick.
                let d = ch.drops.get();
                d.write_volatile(d.read_volatile().wrapping_add(1));
                return;
            }
            let buf = &mut *ch.bufs[idx].get();
            if sample.window_valid {
                self.valid |= 1 << self.n;
            }
            self.alert |= sample.fault;
            self.at = encode_sample(self.mask, sample, buf, self.at);
            self.n += 1;
            self.remaining -= 1;
            if self.n as usize == STREAM_SAMPLES_MAX || self.remaining == 0 {
                let last = self.remaining == 0;
                encode_stream_hdr(self.seq, last, self.valid, buf);
                ch.meta[idx].get().write_volatile(BufMeta {
                    len: self.at as u16,
                    alert: self.alert,
                    last,
                });
                // A re-arm that landed mid-batch abandons the batch: its
                // pickup next tick restarts the encoder, and the epoch tag
                // would be dead at the bus anyway.
                if ch.arm_seq.get().read_volatile() == self.epoch {
                    compiler_fence(Ordering::Release);
                    ch.ready[idx].get().write_volatile(self.epoch);
                    self.seq = self.seq.wrapping_add(1);
                    self.idx ^= 1;
                }
                self.at = STREAM_HDR;
                self.n = 0;
                self.valid = 0;
                self.alert = false;
            }
        }
    }
}

/// Bus-side half: posts arms, gates the producer, consumes ready buffers.
pub struct TelDrain {
    ch: &'static TelChannel,
}

impl TelDrain {
    /// Post an arm to the kernel side; returns its epoch (never 0).
    pub(crate) fn send_arm(&mut self, mask: u16, count: u16) -> u8 {
        let ch = self.ch;
        // SAFETY: sole mailbox writer (type doc); payload lands before the
        // fenced seq store the kernel keys on.
        unsafe {
            ch.arm_mask.get().write_volatile(mask);
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_servo_core::tel::{FLAG_LAST, MASK_ALL, encode_stream};

    fn channel() -> (TelFeed, TelDrain) {
        std::boxed::Box::leak(std::boxed::Box::new(TelChannel::new())).split()
    }

    fn sample(i: u16) -> TelSample {
        TelSample {
            pos: 0x4000 + i,
            current: -(i as i16),
            window_valid: i.is_multiple_of(2),
            ..Default::default()
        }
    }

    fn arm(drain: &mut TelDrain, mask: u16, count: u16) -> u8 {
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
        let epoch = arm(&mut drain, MASK_ALL, 20);
        let samples: std::vec::Vec<TelSample> = (0..20).map(sample).collect();
        for s in &samples {
            feed.on_tick(s);
        }

        let mut want = [0u8; STREAM_PAYLOAD_MAX];
        let n0 = encode_stream(MASK_ALL, 0, false, &samples[..16], &mut want);
        let m0 = drain.ready(0, epoch).expect("first batch ready");
        assert_eq!((m0.len as usize, m0.last), (n0, false));
        assert_eq!(drain.payload(0)[..n0], want[..n0]);

        let n1 = encode_stream(MASK_ALL, 1, true, &samples[16..], &mut want);
        let m1 = drain.ready(1, epoch).expect("last batch ready");
        assert_eq!((m1.len as usize, m1.last), (n1, true));
        assert_eq!(drain.payload(1)[..n1], want[..n1]);
        assert_eq!(drain.payload(1)[1], FLAG_LAST);
    }

    #[test]
    fn stalled_consumer_drops_and_counts() {
        let (mut feed, mut drain) = channel();
        let epoch = arm(&mut drain, MASK_ALL, 200);
        for i in 0..37 {
            feed.on_tick(&sample(i));
        }
        // both buffers ready, 5 samples dropped against them
        assert_eq!(feed.drops(), 5);
        // release one: production resumes into it
        assert!(drain.ready(0, epoch).is_some());
        drain.release(0);
        for i in 0..16 {
            feed.on_tick(&sample(100 + i));
        }
        assert_eq!(feed.drops(), 5);
        let m = drain.ready(0, epoch).expect("refilled batch");
        assert!(!m.last);
    }

    #[test]
    fn rearm_discards_the_old_epoch() {
        let (mut feed, mut drain) = channel();
        let e1 = arm(&mut drain, MASK_ALL, 16);
        for i in 0..16 {
            feed.on_tick(&sample(i));
        }
        assert!(drain.ready(0, e1).is_some());

        // new burst: the un-staged old batch is a stray now
        let e2 = arm(&mut drain, MASK_ALL, 5);
        assert!(drain.ready(0, e2).is_none(), "old epoch discarded");
        for i in 0..5 {
            feed.on_tick(&sample(50 + i));
        }
        let m = drain.ready(0, e2).expect("fresh burst lands in buffer 0");
        assert!(m.last);
        // seq restarted at 0
        assert_eq!(drain.payload(0)[0], 0);
    }

    #[test]
    fn count_zero_arm_parks_the_encoder() {
        let (mut feed, mut drain) = channel();
        arm(&mut drain, MASK_ALL, 32);
        for i in 0..4 {
            feed.on_tick(&sample(i));
        }
        let e = drain.send_arm(MASK_ALL, 0);
        drain.clear_ready();
        drain.set_active(false);
        for i in 0..40 {
            feed.on_tick(&sample(i));
        }
        assert!(drain.ready(0, e).is_none());
        assert!(drain.ready(1, e).is_none());
        assert_eq!(feed.drops(), 0);
    }
}

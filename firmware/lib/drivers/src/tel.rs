//! TEL burst seam between the kernel fast tick (PFIC HIGH) and the bus-side
//! frame sender (the bus-level vectors, incl. the SW vector each tick pends).
//! The two sides never share `&mut`: the channel is a ring of [`TEL_BUFS`]
//! frame buffers in wire layout. The kernel encodes each sample DIRECTLY at
//! its final wire offset via the core appenders; the bus writes the header
//! and CRC around the banked payload and sends the frame from the buffer in
//! place, so no intermediate copy exists.
//! Cross-side traffic is single-writer-per-field volatile load/store plus
//! compiler fences at the buffer handoffs (single core, no atomic RMW on
//! rv32ec).
//!
//! Buffer ownership rides the `ready` tags: 0 = kernel's to fill, else the
//! arm epoch the batch was produced under - the bus sends only its own
//! epoch and discards strays, so a batch banked across a re-arm dies.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::{Ordering, compiler_fence};

use osc_protocol::frame::Header;
use osc_servo_core::tel::{
    FRAME_SAMPLES, STREAM_HDR, STREAM_PAYLOAD_MAX, TelSample, TelStream, encode_sample,
    encode_stream_hdr,
};

/// Frame buffers in the ring: one on the wire, one filling, and one that
/// covers the bank-to-wire latency (the CRC runs over a banked frame before
/// it leaves) and kernel jitter. No count covers a sustained wire deficit;
/// the one-arm send does (DES pin `tel_six_fields_keep_up_while_stepping`).
pub const TEL_BUFS: usize = 3;

pub(crate) const CRC_LEN: usize = core::mem::size_of::<u16>();

/// Bytes of one frame buffer: align byte and header, payload, CRC.
pub const FRAME_LEN: usize = Header::SIZE + STREAM_PAYLOAD_MAX + CRC_LEN;

/// One burst frame in wire layout. The CRC sits right behind the payload,
/// inside `payload` on a short LAST frame. Even-based: the CRC engine feeds
/// halfwords from `head[0]`.
#[repr(C, align(2))]
struct Frame {
    head: [u8; Header::SIZE],
    payload: [u8; STREAM_PAYLOAD_MAX],
    tail: [u8; CRC_LEN],
}

const _: () = assert!(core::mem::size_of::<Frame>() == FRAME_LEN);

/// One finished frame's metadata, kernel-written at batch finalize (before
/// its `ready` tag), bus-read while the tag holds.
#[derive(Copy, Clone)]
pub(crate) struct BufMeta {
    /// Payload bytes.
    pub(crate) len: u16,
    /// OR of the batch's fault bits -> the frame's INST ALERT.
    pub(crate) alert: bool,
    pub(crate) last: bool,
}

struct Slot {
    frame: Frame,
    meta: BufMeta,
    ready: u8,
}

const SLOT_ZERO: Slot = Slot {
    frame: Frame {
        head: [0; Header::SIZE],
        payload: [0; STREAM_PAYLOAD_MAX],
        tail: [0; CRC_LEN],
    },
    meta: BufMeta {
        len: 0,
        alert: false,
        last: false,
    },
    ready: 0,
};

/// The buffer after `idx` in ring order.
pub(crate) const fn next_buf(idx: usize) -> usize {
    if idx + 1 >= TEL_BUFS { 0 } else { idx + 1 }
}

/// The shared storage. Field writers: the payloads, `meta`, `ready`(set) and
/// `drops` belong to the kernel side ([`TelFeed`]); `active`, the arm
/// mailbox, the frame headers and CRC tails, and `ready`(clear) to the bus
/// side ([`TelDrain`]), which writes a frame only while it holds the tag.
/// The dual-writer `ready` tags are safe: each side stores only constants
/// its protocol phase owns.
pub struct TelChannel {
    slots: [SyncUnsafeCell<Slot>; TEL_BUFS],
    /// Burst gate the producer polls each tick.
    active: SyncUnsafeCell<bool>,
    arm_mask: SyncUnsafeCell<u16>,
    arm_count: SyncUnsafeCell<u16>,
    /// Mailbox epoch, written last (fence-ordered), never 0; the kernel
    /// double-reads it around the payload to reject a torn pickup.
    arm_seq: SyncUnsafeCell<u8>,
    /// Samples dropped while no buffer was free, monotonic wrapping.
    drops: SyncUnsafeCell<u16>,
}

impl TelChannel {
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            slots: [const { SyncUnsafeCell::new(SLOT_ZERO) }; TEL_BUFS],
            active: SyncUnsafeCell::new(false),
            arm_mask: SyncUnsafeCell::new(0),
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

    fn slot(&self, idx: usize) -> Option<*mut Slot> {
        self.slots.get(idx).map(SyncUnsafeCell::get)
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
    /// (a torn read, the mailbox rewritten mid-pickup, retries next
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
            // Belt to the table rule's suspenders: an invalid mask parks
            // the burst instead of overrunning the fixed payload buffers.
            self.remaining = if osc_servo_core::tel::mask_valid(mask) {
                count
            } else {
                0
            };
            self.seq = 0;
            self.idx = 0;
            self.at = STREAM_HDR;
            self.n = 0;
            self.valid = 0;
            self.alert = false;
        }
    }
}

impl TelStream for TelFeed {
    fn active(&self) -> bool {
        self.ch.active()
    }

    fn on_tick(&mut self, sample: &TelSample) {
        self.poll_arm();
        if self.remaining == 0 {
            return;
        }
        let ch = self.ch;
        let Some(slot) = ch.slot(self.idx) else {
            return;
        };
        // SAFETY: single-writer discipline (type doc). The payload `&mut` is
        // exclusive while its ready tag is 0 (the bus touches only tagged
        // buffers); the tag store is fenced behind the payload writes.
        unsafe {
            if (&raw const (*slot).ready).read_volatile() != 0 {
                // Consumer stalled: never block the fast tick.
                let d = ch.drops.get();
                d.write_volatile(d.read_volatile().wrapping_add(1));
                return;
            }
            let buf = &mut (*slot).frame.payload;
            if sample.window_valid {
                self.valid |= 1 << self.n;
            }
            self.alert |= sample.fault;
            self.at = encode_sample(self.mask, sample, buf, self.at);
            self.n += 1;
            self.remaining -= 1;
            if self.n as usize == FRAME_SAMPLES || self.remaining == 0 {
                let last = self.remaining == 0;
                encode_stream_hdr(self.seq, last, self.valid, buf);
                (&raw mut (*slot).meta).write_volatile(BufMeta {
                    len: self.at as u16,
                    alert: self.alert,
                    last,
                });
                // A re-arm that landed mid-batch abandons the batch: its
                // pickup next tick restarts the encoder, and the epoch tag
                // would be dead at the bus anyway.
                if ch.arm_seq.get().read_volatile() == self.epoch {
                    compiler_fence(Ordering::Release);
                    (&raw mut (*slot).ready).write_volatile(self.epoch);
                    self.seq = self.seq.wrapping_add(1);
                    self.idx = next_buf(self.idx);
                }
                self.at = STREAM_HDR;
                self.n = 0;
                self.valid = 0;
                self.alert = false;
            }
        }
    }
}

/// Bus-side half: posts arms, gates the producer, frames and frees banked
/// buffers.
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

    /// Free every buffer (arm/abort: stale batches never send).
    pub(crate) fn clear_ready(&mut self) {
        for idx in 0..TEL_BUFS {
            self.release(idx);
        }
    }

    /// Metadata of buffer `idx` if it is ready under `epoch`; a stray epoch
    /// (dead burst) is discarded on sight.
    pub(crate) fn ready(&mut self, idx: usize, epoch: u8) -> Option<BufMeta> {
        let slot = self.ch.slot(idx)?;
        // SAFETY: tag read gates the meta read behind an acquire fence;
        // ready-clear is the bus's store (type doc).
        unsafe {
            let ready = &raw mut (*slot).ready;
            let tag = ready.read_volatile();
            if tag == 0 {
                return None;
            }
            if tag != epoch {
                ready.write_volatile(0);
                return None;
            }
            compiler_fence(Ordering::Acquire);
            Some((&raw const (*slot).meta).read_volatile())
        }
    }

    /// Buffer `idx` as wire bytes, for the header, the CRC and the send.
    /// The caller holds its ready tag (`ready` returned Some).
    pub(crate) fn frame(&mut self, idx: usize) -> Option<&mut [u8; FRAME_LEN]> {
        let slot = self.ch.slot(idx)?;
        // SAFETY: `Frame` is u8 arrays only, repr(C), FRAME_LEN bytes with no
        // padding (const-asserted), so any byte view is valid; the kernel
        // writes only untagged buffers, and the caller holds this one's tag.
        unsafe { Some(&mut *(&raw mut (*slot).frame).cast::<[u8; FRAME_LEN]>()) }
    }

    /// Return buffer `idx` to the kernel (after its frame left the wire).
    pub(crate) fn release(&mut self, idx: usize) {
        if let Some(slot) = self.ch.slot(idx) {
            // SAFETY: ready-clear is the bus's store (type doc).
            unsafe { (&raw mut (*slot).ready).write_volatile(0) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_servo_core::tel::{FLAG_LAST, encode_stream};
    const MASK_SIX: u16 = 0x3F;

    fn leaked() -> &'static TelChannel {
        std::boxed::Box::leak(std::boxed::Box::new(TelChannel::new()))
    }

    fn channel() -> (TelFeed, TelDrain) {
        leaked().split()
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

    fn payload(drain: &mut TelDrain, idx: usize) -> std::vec::Vec<u8> {
        drain.frame(idx).expect("buffer")[Header::SIZE..Header::SIZE + STREAM_PAYLOAD_MAX].to_vec()
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
    fn frame_buffers_are_even_based_for_the_crc_feed() {
        let ch = leaked();
        for idx in 0..TEL_BUFS {
            let p = ch.slot(idx).expect("slot");
            // SAFETY: address only.
            let at = unsafe { &raw const (*p).frame } as usize;
            assert_eq!(at % 2, 0, "buffer {idx}");
        }
        assert!(ch.slot(TEL_BUFS).is_none());
    }

    #[test]
    fn batches_match_encode_stream_byte_for_byte() {
        let (mut feed, mut drain) = channel();
        let epoch = arm(&mut drain, MASK_SIX, 20);
        let samples: std::vec::Vec<TelSample> = (0..20).map(sample).collect();
        for s in &samples {
            feed.on_tick(s);
        }

        let k = FRAME_SAMPLES;
        let mut want = [0u8; STREAM_PAYLOAD_MAX];
        let n0 = encode_stream(MASK_SIX, 0, false, &samples[..k], &mut want);
        let m0 = drain.ready(0, epoch).expect("first batch ready");
        assert_eq!((m0.len as usize, m0.last), (n0, false));
        assert_eq!(payload(&mut drain, 0)[..n0], want[..n0]);

        let n1 = encode_stream(MASK_SIX, 1, true, &samples[k..], &mut want);
        let m1 = drain.ready(1, epoch).expect("last batch ready");
        assert_eq!((m1.len as usize, m1.last), (n1, true));
        assert_eq!(payload(&mut drain, 1)[..n1], want[..n1]);
        assert_eq!(payload(&mut drain, 1)[1], FLAG_LAST);
    }

    #[test]
    fn stalled_consumer_drops_and_counts() {
        let ch = leaked();
        let (mut feed, mut drain) = ch.split();
        let k = FRAME_SAMPLES as u16;
        let epoch = arm(&mut drain, MASK_SIX, 200);
        for i in 0..3 * k + 5 {
            feed.on_tick(&sample(i));
        }
        // all three buffers ready, 5 samples dropped against them
        assert_eq!(ch.drops(), 5);
        // release the oldest: production resumes into it, in ring order
        assert!(drain.ready(0, epoch).is_some());
        drain.release(0);
        for i in 0..k {
            feed.on_tick(&sample(100 + i));
        }
        assert_eq!(ch.drops(), 5);
        let m = drain.ready(0, epoch).expect("refilled batch");
        assert!(!m.last);
        assert_eq!(payload(&mut drain, 0)[0], 3, "fourth batch, seq 3");
    }

    #[test]
    fn rearm_discards_the_old_epoch() {
        let (mut feed, mut drain) = channel();
        let k = FRAME_SAMPLES as u16;
        let e1 = arm(&mut drain, MASK_SIX, k);
        for i in 0..k {
            feed.on_tick(&sample(i));
        }
        assert!(drain.ready(0, e1).is_some());

        // new burst: the un-sent old batch is a stray now
        let e2 = arm(&mut drain, MASK_SIX, 5);
        assert!(drain.ready(0, e2).is_none(), "old epoch discarded");
        for i in 0..5 {
            feed.on_tick(&sample(50 + i));
        }
        let m = drain.ready(0, e2).expect("fresh burst lands in buffer 0");
        assert!(m.last);
        // seq restarted at 0
        assert_eq!(payload(&mut drain, 0)[0], 0);
    }

    #[test]
    fn count_zero_arm_parks_the_encoder() {
        let ch = leaked();
        let (mut feed, mut drain) = ch.split();
        arm(&mut drain, MASK_SIX, 32);
        for i in 0..4 {
            feed.on_tick(&sample(i));
        }
        let e = drain.send_arm(MASK_SIX, 0);
        drain.clear_ready();
        drain.set_active(false);
        for i in 0..40 {
            feed.on_tick(&sample(i));
        }
        for idx in 0..TEL_BUFS {
            assert!(drain.ready(idx, e).is_none());
        }
        assert_eq!(ch.drops(), 0);
    }
}

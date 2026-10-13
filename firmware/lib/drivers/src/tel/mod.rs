//! TEL burst lane (protocol sec 5.6): while a burst streams, the kernel
//! fast tick (PFIC HIGH) owns the bus TX lane and is the transmitter. Each
//! row is a copy of the table's chunks straight to its final wire offset in
//! one of two frame images, started by the kernel at fixed points of the
//! tick; a frame breaks once its lead row has landed and its DMA reads the
//! rows behind it as the wire reaches them, the CRC engine fed by the
//! producer and the value patched in before the DMA gets there. No TX
//! interrupt and no bus-level body runs for the burst: the bus lends the
//! lane at the arming ack's release (at the arm itself when no ack
//! follows) and takes it back in its next body once the LAST frame has
//! left the wire.
//!
//! Cross-side traffic is single-writer-per-field volatile load/store plus
//! compiler fences (single core, no atomic RMW on rv32ec). The bus writes
//! the arm mailbox only while the lane is its own, then `active`; the
//! kernel, which the bus never preempts, takes the arm whole. The row plan
//! is plain memory, ordered by the fences around `arm_seq`.

use core::cell::SyncUnsafeCell;

use osc_protocol::frame::Header;
use osc_protocol::wire;
use osc_servo_core::ControlTableCell;
use osc_servo_core::tel::{STREAM_HDR, STREAM_PAYLOAD_MAX};

use crate::bench;
use crate::traits::bus::Providers;

mod lane;
mod post;
mod rows;

use lane::Lane;
pub use lane::TelLane;
pub use post::TelPost;
use rows::RowPlan;

const CRC_LEN: usize = core::mem::size_of::<u16>();

/// A frame image at its wire offsets: ALIGN, ID, LEN, INST, the payload,
/// the CRC (sec 3.1). The DMA streams ID..CRC.
pub const IMAGE_LEN: usize = Header::SIZE + STREAM_PAYLOAD_MAX + CRC_LEN;

const ID_AT: usize = 1;
const LEN_AT: usize = 2;
const INST_AT: usize = 3;
const FLAGS_AT: usize = Header::SIZE + 1;
const ROWS_AT: usize = Header::SIZE + STREAM_HDR;

// Rows land as halfword copies, and every covered span (ROWS_AT and whole
// even rows) feeds the CRC engine whole: no odd byte to fold.
const _: () = assert!(ROWS_AT.is_multiple_of(2));

/// Even base: the CRC feed reads halfwords from it (F12).
#[repr(C, align(2))]
struct Image([u8; IMAGE_LEN]);

/// The image's covered length (ALIGN..trailer), from its LEN, never past
/// the CRC slot.
#[inline(always)]
fn covered(img: &[u8; IMAGE_LEN]) -> usize {
    wire::covered_len(img[LEN_AT]).min(IMAGE_LEN - CRC_LEN)
}

/// The shared storage. Field writers: the images, `drops`, `last_out` and
/// `resume` belong to the kernel side ([`TelLane`]); the mailbox and
/// `active` to the bus side ([`TelPost`]).
pub struct TelChannel {
    images: [SyncUnsafeCell<Image>; 2],
    /// The kernel owns the TX lane.
    active: SyncUnsafeCell<bool>,
    arm_row: SyncUnsafeCell<RowPlan>,
    arm_count: SyncUnsafeCell<u16>,
    arm_id: SyncUnsafeCell<u8>,
    /// Mailbox epoch, written last, never 0.
    arm_seq: SyncUnsafeCell<u8>,
    /// The epoch whose LAST frame is on the wire, and the RX ring cursor
    /// read while it streamed.
    last_out: SyncUnsafeCell<u8>,
    resume: SyncUnsafeCell<u16>,
    /// Rows dropped while both images held unfinished frames, monotonic
    /// wrapping.
    drops: SyncUnsafeCell<u16>,
}

impl TelChannel {
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            images: [const { SyncUnsafeCell::new(Image([0; IMAGE_LEN])) }; 2],
            active: SyncUnsafeCell::new(false),
            arm_row: SyncUnsafeCell::new(RowPlan::NONE),
            arm_count: SyncUnsafeCell::new(0),
            arm_id: SyncUnsafeCell::new(0),
            arm_seq: SyncUnsafeCell::new(0),
            last_out: SyncUnsafeCell::new(0),
            resume: SyncUnsafeCell::new(0),
            drops: SyncUnsafeCell::new(0),
        }
    }

    /// The kernel owns the TX lane. Any context: the bus is its only
    /// writer.
    pub fn active(&self) -> bool {
        // SAFETY: single-byte volatile read of a bus-written flag.
        unsafe { self.active.get().read_volatile() }
    }

    /// Rows dropped against full images, monotonic wrapping. Any context:
    /// the kernel side is the counter's only writer.
    pub fn drops(&self) -> u16 {
        // SAFETY: single-word volatile read of a kernel-written counter.
        unsafe { self.drops.get().read_volatile() }
    }

    /// Split into the two halves over the lane's wire, CRC engine, RX ring
    /// and clock handles and the table the rows copy from. Call once at
    /// bringup, before interrupts run: a second feed or drain breaks the
    /// single-writer discipline.
    pub fn split<P: Providers>(
        &'static self,
        wire: P::Tx,
        crc: P::Crc,
        ring: P::Ring,
        clock: P::Deadline,
        table: &'static ControlTableCell,
    ) -> (TelLane<P>, TelPost) {
        bench::tel_calibrate(&clock);
        (
            TelLane {
                ch: self,
                wire,
                crc,
                ring,
                clock,
                #[cfg(feature = "bench-telcost")]
                stamp: false,
                table,
                epoch: 0,
                id: 0,
                plan: RowPlan::NONE,
                remaining: 0,
                seq: 0,
                done: 0,
                wr: 0,
                n: 0,
                rows: 0,
                lead: 0,
                phase: Lane::Idle,
                fed: 0,
                crc_due: false,
                landed: false,
            },
            TelPost { ch: self, table },
        )
    }

    /// Frame `f`'s image.
    ///
    /// # Safety
    /// Kernel side only: the images are its own, and the DMA only reads.
    #[allow(clippy::mut_from_ref)]
    unsafe fn image(&self, f: u8) -> &mut [u8; IMAGE_LEN] {
        // SAFETY: see fn doc.
        unsafe { &mut (*self.images[(f & 1) as usize].get()).0 }
    }
}

#[cfg(test)]
mod tests;

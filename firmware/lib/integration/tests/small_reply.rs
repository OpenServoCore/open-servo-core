//! A READ of a two-byte field answers from the live table through the TX
//! engine's small-copy path (`bus::tx`, payloads up to `SMALL_COPY_MAX`).
//! On the chip the kernel preempts that copy, and the ADC DMA lands `pos`
//! inside the kernel's tick, so a copy that loads the field a byte at a
//! time can carry one tick's low byte and the next tick's high byte: a
//! reading off by 256 while the shaft crosses a multiple of 256.
//!
//! The sim cannot express this: handler effects land at entry
//! (`sim::cpu`) and no kernel body runs inside a bus body, so the pin
//! races a second thread against the copy instead. The thread stands in
//! for the scan DMA: halfword stores of `pos` into the tick block,
//! flickering across the 2047/2048 boundary the way a parked pot does.
//! One aligned halfword load is atomic against such a store on the chip
//! (an interrupt lands between instructions, a DMA beat is one halfword)
//! and on every host this runs on, so a whole-field copy never fails here;
//! a byte-pair copy fails once the race lands between its loads. The race
//! lands in the unoptimized test profile `cargo test` builds; an optimized
//! host build puts the two byte loads back to back, where an out-of-order
//! core rarely lets a store between them, and a single-core host may never
//! land it: a pass there proves nothing about a byte-pair copy.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use control_table::RegisterFile;
use osc_protocol::wire::ResultCode;
use osc_servo_core::Shared;
use osc_servo_core::regions::telemetry::addr::tick;
use osc_servo_drivers::bus::tx::TxEngine;
use osc_servo_drivers::traits::bus::{CrcEngine, TxWire};

/// The two samples of a pot parked on the 2047/2048 boundary.
const BELOW: u16 = 0x07FF;
const ABOVE: u16 = 0x0800;

/// Stops at the first torn reply; a whole-field copy runs it out.
const READS: u32 = 2_000_000;
const RUN_FOR: Duration = Duration::from_secs(2);

const ID: u8 = 1;

/// Records the body arm: ID, LEN, INST, then the payload.
#[derive(Clone, Default)]
struct Wire(Rc<RefCell<Vec<u8>>>);

impl TxWire for Wire {
    fn start_frame(&mut self) {}
    fn send(&mut self, span: &[u8]) {
        let mut body = self.0.borrow_mut();
        if body.is_empty() {
            body.extend_from_slice(span);
        }
    }
    fn release(&mut self) {}
    fn idle(&self) -> bool {
        true
    }
    fn break_start(&mut self) {}
    fn stream(&mut self, _image: &[u8]) {}
}

struct Crc;

impl CrcEngine for Crc {
    fn reset(&mut self) {}
    fn feed(&mut self, _span: &[u8]) {}
    fn snapshot(&mut self, _off: u16, src: &[u8]) -> *const u8 {
        src.as_ptr()
    }
    unsafe fn copy(&mut self, _dst: *mut u8, _src: *const u8, _units: u16) {}
    fn result(&mut self) -> Option<u16> {
        Some(0)
    }
}

/// A two-byte reply carries `pos` from one sample: both bytes from the
/// same store, never the low byte of one and the high byte of the next.
#[test]
fn a_two_byte_reply_loads_its_field_whole() {
    assert_eq!(tick::POS % 2, 0, "pos is a naturally aligned halfword");
    let shared: &'static Shared = Box::leak(Box::new(Shared::new()));
    // SAFETY: in-bounds field projection of the leaked table.
    let pos = unsafe { &raw mut (*shared.tick_block()).pos };
    unsafe { pos.write_volatile(BELOW) };

    let stop = Arc::new(AtomicBool::new(false));
    let started = Arc::new(AtomicBool::new(false));
    let dma = {
        let (stop, started) = (stop.clone(), started.clone());
        let at = pos as usize;
        thread::spawn(move || {
            let p = at as *mut u16;
            started.store(true, Ordering::Release);
            while !stop.load(Ordering::Relaxed) {
                // SAFETY: the leaked table outlives the thread (joined
                // below); halfword stores, as the scan DMA lands them.
                unsafe {
                    p.write_volatile(ABOVE);
                    p.write_volatile(BELOW);
                }
            }
        })
    };
    while !started.load(Ordering::Acquire) {
        std::hint::spin_loop();
    }

    let wire = Wire::default();
    let body = wire.0.clone();
    let mut tx = TxEngine::new(wire);
    let mut crc = Crc;
    let t0 = Instant::now();
    let mut torn = None;
    let mut reads = 0;
    while reads < READS && t0.elapsed() < RUN_FOR {
        // The read path of `Dispatcher::read`: a slice of the live table,
        // staged as the status payload. Opaque, as on the chip, so the
        // optimizer cannot fold a byte-pair copy into one load.
        let data = std::hint::black_box(
            RegisterFile::read(&shared.table, tick::POS, 2).expect("pos is in the table"),
        );
        tx.stage(&mut crc, ID, ResultCode::Ok, false, data)
            .expect("engine idle");
        tx.trigger(&mut crc, None);
        let _ = tx.abort();
        let sent = {
            let mut b = body.borrow_mut();
            let v = u16::from_le_bytes([b[b.len() - 2], b[b.len() - 1]]);
            b.clear();
            v
        };
        reads += 1;
        if sent != BELOW && sent != ABOVE {
            torn = Some(sent);
            break;
        }
    }
    stop.store(true, Ordering::Relaxed);
    dma.join().expect("writer thread");

    if let Some(v) = torn {
        panic!(
            "read {reads}: pos {v} ({v:#06x}) is neither sample {BELOW} ({BELOW:#06x}) nor \
             {ABOVE} ({ABOVE:#06x}): the reply took the field's bytes from two stores"
        );
    }
    println!("{reads} two-byte replies, every one a whole sample");
}

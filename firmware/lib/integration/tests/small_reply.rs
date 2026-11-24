//! A READ answers from the live table through the TX engine: a payload of
//! two bytes or less, or one 4-aligned word, by the CPU into the staging
//! buffer (`bus::tx`), anything longer through the snapshot copy
//! (`bus::snapshot`), whose copy channel takes the aligned middle in word
//! beats while the CPU copies the ends. On the chip the kernel preempts the
//! CPU copy and stores fields beside the channel's, and the ADC DMA lands
//! `pos` inside the kernel's tick, so a copy that moves a field in narrower
//! pieces than its width can carry one tick's low bytes and the next tick's
//! high bytes: a reading off by 256 while the shaft crosses a multiple of
//! 256, or a tick count off by 65536.
//!
//! The sim cannot express this: handler effects land at entry
//! (`sim::cpu`) and no kernel body runs inside a bus body, so the pins race
//! a second thread against the copy instead. The thread stands in for the
//! scan DMA and the kernel: one store per field, flickering across a
//! boundary every byte of which changes. One aligned load is atomic against
//! such a store on the chip (an interrupt lands between instructions, a
//! DMA beat is one access) and on every host this runs on, so a whole-field
//! copy never fails here; a copy in narrower pieces fails once the race
//! lands between them. The race lands in the unoptimized test profile
//! `cargo test` builds; an optimized host build puts the pieces back to
//! back, where an out-of-order core rarely lets a store between them, and a
//! single-core host may never land it: a pass there proves nothing about a
//! piecewise copy.

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
use osc_servo_drivers::bus::snapshot::{self, Beat, Buf};
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

/// A CRC engine whose copy channel moves each beat as one load and one
/// store, as the chip's does.
#[derive(Default)]
struct Crc {
    snap: Box<Buf>,
    snapshots: usize,
}

impl CrcEngine for Crc {
    fn reset(&mut self) {}
    fn feed(&mut self, _span: &[u8]) {}
    unsafe fn snapshot(&mut self, spans: &[&[u8]]) -> *mut u8 {
        self.snapshots += 1;
        // SAFETY: the caller's total; `gather` hands the channel in-bounds
        // ends, which the assert holds to the beat's alignment.
        unsafe {
            snapshot::gather(&mut *self.snap, spans, |src, dst, beat, beats| {
                let w = beat as usize;
                assert_eq!((src.addr() | dst.addr()) % w, 0, "unaligned {w}-byte beat");
                for k in 0..beats as usize {
                    let (s, d) = (src.add(k * w), dst.add(k * w));
                    match beat {
                        Beat::Word => d.cast::<u32>().write(s.cast::<u32>().read_volatile()),
                        Beat::Half => d.cast::<u16>().write(s.cast::<u16>().read_volatile()),
                        Beat::Byte => d.write(s.read_volatile()),
                    }
                }
            })
        }
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
    let mut crc = Crc::default();
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

/// The two counts of a tick counter crossing 65536, and of a field crossing
/// 256: every byte differs between the two.
const TICK: [u32; 2] = [0x0000_FFFF, 0x0001_0000];
const RAW: [u16; 2] = [0x00FF, 0x0100];

/// A thread flipping `sample_tick` (a word at 0 mod 4) and
/// `vmotor_a_trough` (a halfword at 2 mod 4), one store each, until dropped.
struct Flipper {
    stop: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl Flipper {
    fn start(shared: &'static Shared) -> Self {
        // SAFETY: in-bounds field projections of the leaked table.
        let (tick, raw) = unsafe {
            let b = shared.tick_block();
            (&raw mut (*b).sample_tick, &raw mut (*b).vmotor_a_trough)
        };
        unsafe {
            tick.write_volatile(TICK[0]);
            raw.write_volatile(RAW[0]);
        }
        let stop = Arc::new(AtomicBool::new(false));
        let started = Arc::new(AtomicBool::new(false));
        let thread = {
            let (stop, started) = (stop.clone(), started.clone());
            let (tick, raw) = (tick as usize, raw as usize);
            thread::spawn(move || {
                let (tick, raw) = (tick as *mut u32, raw as *mut u16);
                started.store(true, Ordering::Release);
                while !stop.load(Ordering::Relaxed) {
                    // SAFETY: the leaked table outlives the thread (joined
                    // at drop); one store per field.
                    unsafe {
                        tick.write_volatile(TICK[1]);
                        raw.write_volatile(RAW[1]);
                        tick.write_volatile(TICK[0]);
                        raw.write_volatile(RAW[0]);
                    }
                }
            })
        };
        while !started.load(Ordering::Acquire) {
            std::hint::spin_loop();
        }
        Self {
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for Flipper {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            t.join().expect("writer thread");
        }
    }
}

/// One read shape: the spans `(addr, count)` it gathers and where its
/// payload holds the two flipping fields, if it covers them.
struct Shape {
    spans: &'static [(u16, u16)],
    tick_at: Option<usize>,
    raw_at: Option<usize>,
}

/// Stage `shape` from the live table as `Dispatcher` does, and return the
/// streamed payload.
fn read(
    tx: &mut TxEngine<Wire>,
    crc: &mut Crc,
    body: &RefCell<Vec<u8>>,
    shared: &Shared,
    shape: &Shape,
) -> Vec<u8> {
    let spans: Vec<&[u8]> = shape
        .spans
        .iter()
        .map(|&(addr, count)| RegisterFile::read(&shared.table, addr, count).expect("in the table"))
        .collect();
    // Opaque, as on the chip, so the optimizer cannot fold the copy.
    let spans = std::hint::black_box(spans);
    tx.stage_gather(crc, ID, ResultCode::Ok, false, &spans)
        .expect("engine idle");
    tx.trigger(crc, None);
    let _ = tx.abort();
    let mut b = body.borrow_mut();
    let payload = b[3..].to_vec();
    b.clear();
    payload
}

/// Race `shapes` against the flipper; panic on the first torn field.
fn race(shapes: &[Shape]) -> (u32, usize) {
    let shared: &'static Shared = Box::leak(Box::new(Shared::new()));
    assert_eq!(
        RegisterFile::read(&shared.table, 0, 1)
            .expect("in the table")
            .as_ptr() as usize
            % 4,
        0,
        "a table address and its pointer agree mod 4"
    );
    let flipper = Flipper::start(shared);
    let wire = Wire::default();
    let body = wire.0.clone();
    let mut tx = TxEngine::new(wire);
    let mut crc = Crc::default();
    let t0 = Instant::now();
    let mut reads = 0;
    while reads < READS && t0.elapsed() < RUN_FOR {
        let shape = &shapes[reads as usize % shapes.len()];
        let p = read(&mut tx, &mut crc, &body, shared, shape);
        reads += 1;
        if let Some(at) = shape.tick_at {
            let v = u32::from_le_bytes([p[at], p[at + 1], p[at + 2], p[at + 3]]);
            if !TICK.contains(&v) {
                drop(flipper);
                panic!(
                    "read {reads} {:x?}: sample_tick {v:#010x} is neither {:#010x} nor {:#010x}: \
                     the reply took the field's bytes from two stores",
                    shape.spans, TICK[0], TICK[1]
                );
            }
        }
        if let Some(at) = shape.raw_at {
            let v = u16::from_le_bytes([p[at], p[at + 1]]);
            if !RAW.contains(&v) {
                drop(flipper);
                panic!(
                    "read {reads} {:x?}: vmotor_a_trough {v:#06x} is neither {:#06x} nor {:#06x}: \
                     the reply took the field's bytes from two stores",
                    shape.spans, RAW[0], RAW[1]
                );
            }
        }
    }
    (reads, crc.snapshots)
}

const TICK_ADDR: u16 = tick::SAMPLE_TICK;
const RAW_ADDR: u16 = tick::VMOTOR_A_TROUGH;

/// A longer read carries each field it covers from one store, at every
/// frame shift and in a profile read: the word in the channel's word beats
/// or one CPU word load, the halfword in a word beat or one CPU halfword load.
#[test]
fn a_long_reply_copies_each_field_whole() {
    assert_eq!((TICK_ADDR % 4, RAW_ADDR % 4), (0, 2));
    let t = TICK_ADDR as usize;
    let r = RAW_ADDR as usize;
    let at = |field: usize, start: u16| Some(field - start as usize);
    // Plain reads at all four shifts, the halfword in the channel's part or
    // the CPU head; a profile read whose word and halfword spans are
    // mismatched, so the CPU copies them.
    let shapes = [
        Shape {
            spans: &[(RAW_ADDR - 5, 40)],
            tick_at: at(t, RAW_ADDR - 5),
            raw_at: at(r, RAW_ADDR - 5),
        },
        Shape {
            spans: &[(RAW_ADDR, 11)],
            tick_at: at(t, RAW_ADDR),
            raw_at: at(r, RAW_ADDR),
        },
        Shape {
            spans: &[(RAW_ADDR - 3, 13)],
            tick_at: at(t, RAW_ADDR - 3),
            raw_at: at(r, RAW_ADDR - 3),
        },
        Shape {
            spans: &[(RAW_ADDR - 2, 12)],
            tick_at: at(t, RAW_ADDR - 2),
            raw_at: at(r, RAW_ADDR - 2),
        },
        Shape {
            spans: &[(RAW_ADDR - 5, 2), (TICK_ADDR, 4), (RAW_ADDR, 2)],
            tick_at: Some(2),
            raw_at: Some(6),
        },
    ];
    let (reads, snapshots) = race(&shapes);
    assert_eq!(snapshots, reads as usize, "every shape takes the snapshot");
    println!("{reads} long replies, every field whole");
}

/// A four-byte read of a 4-aligned word takes one CPU word load into the
/// staging buffer, and carries the word from one store.
#[test]
fn a_four_byte_reply_loads_its_field_whole() {
    let shapes = [Shape {
        spans: &[(TICK_ADDR, 4)],
        tick_at: Some(0),
        raw_at: None,
    }];
    let (reads, snapshots) = race(&shapes);
    assert_eq!(snapshots, 0, "the staging buffer, not the snapshot");
    println!("{reads} four-byte replies, every one a whole count");
}

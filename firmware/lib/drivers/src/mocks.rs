//! Recording mocks for driver unit tests. Each fake implements one of the
//! driver-owned interfaces and logs the calls it received; tests assert
//! against the log. Host-side only (`cfg(test)`), so `Vec` is available
//! via the explicit `extern crate std`.

extern crate std;
use core::cell::Cell;
use std::vec::Vec;

use osc_core::BaudRate;

use dxl_protocol::SoftwareCrcUmts;

use crate::traits::dxl::{
    ClockTrim, DmaFlags, EdgeDma, FastLastScheduler, Providers, RxDma, SendKind, TxScheduler,
    UsartBaud,
};
use crate::traits::{DigitalOut, Monotonic};
use crate::types::Level;

/// Driver-side `Providers` impl for tests — bundles every fake provider into
/// the single super-trait the [`DxlUart`] composite consumes. Tests
/// instantiate `DxlUart<TestProviders, …>` directly; the fakes' recording
/// state is reached through the composite's accessors / debug `pub` fields
/// (e.g. `bus.scheduler.log`, `bus.fast_last.scheduler().log`).
///
/// [`DxlUart`]: crate::dxl::uart::DxlUart
pub struct TestProviders;

impl Providers for TestProviders {
    type UsartBaud = FakeUsartBaud;
    type ClockTrim = FakeClockTrim;
    type EdgeDma = FakeEdgeDma;
    type RxDma = FakeRxDma;
    type TxScheduler = FakeTxScheduler;
    type FastLastScheduler = FakeFastLastScheduler;
    type Crc = SoftwareCrcUmts;
}

#[derive(Default)]
pub struct FakeDigitalOut {
    pub log: Vec<Level>,
}

impl DigitalOut for FakeDigitalOut {
    fn set(&mut self, level: Level) {
        self.log.push(level);
    }
}

/// Settable tick value; tests advance time by writing `now`. `TICKS_PER_US`
/// is 1 so test arithmetic stays in µs without scaling.
#[derive(Default)]
pub struct FakeMonotonic {
    pub now: u32,
}

impl Monotonic for FakeMonotonic {
    const TICKS_PER_US: u32 = 1;

    fn ticks(&self) -> u32 {
        self.now
    }
}

#[derive(Default)]
pub struct FakeUsartBaud {
    pub log: Vec<BaudRate>,
}

impl UsartBaud for FakeUsartBaud {
    // Same as the production V006 binding (PCLK = HCLK = 48 MHz) so driver
    // ticks_per_bit math matches the chip-side reference table.
    const CLOCK_HZ: u32 = 48_000_000;

    fn apply_baud(&mut self, baud: BaudRate) {
        self.log.push(baud);
    }
}

#[derive(Default)]
pub struct FakeClockTrim {
    pub log: Vec<i8>,
}

impl ClockTrim for FakeClockTrim {
    // Same ratio as the real HSI so existing drift/threshold tests stay
    // numerically aligned without baking in chip-specific imports.
    const DELTA_MIN: i8 = -16;
    const DELTA_MAX: i8 = 15;
    const HZ: u32 = 24_000_000;
    const STEP_HZ: u32 = 60_000;

    fn apply_delta(&mut self, delta: i8) {
        self.log.push(delta);
    }
}

/// Configurable HT/TC flags + remaining count. Tests stage the next ISR
/// view by writing the fields; the read+ack call returns the staged flags
/// and pushes them into `ack_log` so the test can assert the sequence.
/// `pause` / `resume` toggle `paused` and append to `op_log`; while
/// `paused`, `read_and_ack` returns `DmaFlags::default()` without
/// consuming the staged flags — mirroring production where masking
/// HT/TC IE keeps the IRQ from firing in the first place.
#[derive(Default)]
pub struct FakeEdgeDma {
    pub next_flags: DmaFlags,
    pub remaining: u16,
    pub ack_log: Vec<DmaFlags>,
    pub paused: bool,
    pub op_log: Vec<EdgeDmaOp>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EdgeDmaOp {
    Pause,
    Resume,
}

/// NDTR readback only — used by the Fast Last fold body's intra-loop
/// refresh. Tests write `remaining.set(n)` to model new bytes landing
/// without re-entering the codec's `on_rx_dma_advance`. `Cell` rather
/// than a plain field so the test can mutate from inside a closure
/// that borrows the bus immutably.
#[derive(Default)]
pub struct FakeRxDma {
    pub remaining: Cell<u16>,
}

impl RxDma for FakeRxDma {
    fn remaining(&self) -> u16 {
        self.remaining.get()
    }
}

impl EdgeDma for FakeEdgeDma {
    fn read_and_ack(&mut self) -> DmaFlags {
        if self.paused {
            let flags = DmaFlags::default();
            self.ack_log.push(flags);
            return flags;
        }
        let flags = self.next_flags;
        self.ack_log.push(flags);
        self.next_flags = DmaFlags::default();
        flags
    }

    fn remaining(&self) -> u16 {
        self.remaining
    }

    fn pause(&mut self) {
        self.paused = true;
        self.op_log.push(EdgeDmaOp::Pause);
    }

    fn resume(&mut self) {
        self.paused = false;
        self.op_log.push(EdgeDmaOp::Resume);
    }
}

/// One entry per `TxScheduler` call; tests assert the recorded sequence
/// against expected TX scheduling.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ScheduleOp {
    Schedule {
        deadline_tick: u16,
        byte_count: u16,
        kind: SendKind,
    },
    Cancel,
    HandleStart,
    HandleTxComplete,
}

#[derive(Default)]
pub struct FakeTxScheduler {
    pub log: Vec<ScheduleOp>,
}

impl TxScheduler for FakeTxScheduler {
    // Same value as the production V006 binding (HCLK = 48 MHz) so driver
    // tests' deadline_tick math lands on the same numbers the chip sees.
    const TICKS_PER_US: u16 = 48;

    fn schedule(&mut self, deadline_tick: u16, byte_count: u16, kind: SendKind) {
        self.log.push(ScheduleOp::Schedule {
            deadline_tick,
            byte_count,
            kind,
        });
    }

    fn cancel(&mut self) {
        self.log.push(ScheduleOp::Cancel);
    }

    fn handle_start(&mut self) {
        self.log.push(ScheduleOp::HandleStart);
    }

    fn handle_tx_complete(&mut self) {
        self.log.push(ScheduleOp::HandleTxComplete);
    }
}

/// One entry per `FastLastScheduler` call; tests assert the recorded
/// sequence against expected scheduler operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FastLastSchedulerOp {
    SetDeadline {
        packet_end_tick: u16,
        deadline_ticks: u32,
    },
    Schedule {
        offset_ticks: u32,
    },
    Cancel,
}

/// `deadline_passed` reads via interior mutability so the trait method
/// stays `&self`-compatible with production's register-read impl. Tests
/// write `deadline_passed_value.set(true)` to drive the busy-wait exit.
#[derive(Default)]
pub struct FakeFastLastScheduler {
    pub log: Vec<FastLastSchedulerOp>,
    pub deadline_passed_value: Cell<bool>,
}

impl FastLastScheduler for FakeFastLastScheduler {
    // Values match the V006 measurements.rs defaults so driver-side grid
    // math lines up with the chip-side reference.
    const FAST_LAST_ENTRY_TICKS: u16 = 240;
    const BYTES_PER_INTERVAL: u16 = 15;
    const GUARD_BYTES: u16 = 1;

    fn set_deadline(&mut self, packet_end_tick: u16, deadline_ticks: u32) {
        self.log.push(FastLastSchedulerOp::SetDeadline {
            packet_end_tick,
            deadline_ticks,
        });
    }

    fn schedule(&mut self, offset_ticks: u32) {
        self.log
            .push(FastLastSchedulerOp::Schedule { offset_ticks });
    }

    fn deadline_passed(&self) -> bool {
        self.deadline_passed_value.get()
    }

    fn cancel(&mut self) {
        self.log.push(FastLastSchedulerOp::Cancel);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pause_then_read_and_ack_returns_no_flags() {
        let mut ring = FakeEdgeDma {
            next_flags: DmaFlags {
                ht: true,
                tc: false,
            },
            ..Default::default()
        };
        ring.pause();
        let flags = ring.read_and_ack();
        assert_eq!(flags, DmaFlags::default());
        // Staged flags not consumed — the IRQ never fired in production.
        assert_eq!(
            ring.next_flags,
            DmaFlags {
                ht: true,
                tc: false
            }
        );
    }

    #[test]
    fn resume_unmasks_pending_flags() {
        let mut ring = FakeEdgeDma {
            next_flags: DmaFlags {
                ht: false,
                tc: true,
            },
            ..Default::default()
        };
        ring.pause();
        let _ = ring.read_and_ack();
        ring.resume();
        let flags = ring.read_and_ack();
        assert_eq!(
            flags,
            DmaFlags {
                ht: false,
                tc: true
            }
        );
    }

    #[test]
    fn pause_resume_log_records_calls() {
        let mut ring = FakeEdgeDma::default();
        ring.pause();
        ring.resume();
        ring.pause();
        assert_eq!(
            ring.op_log,
            [EdgeDmaOp::Pause, EdgeDmaOp::Resume, EdgeDmaOp::Pause]
        );
    }
}

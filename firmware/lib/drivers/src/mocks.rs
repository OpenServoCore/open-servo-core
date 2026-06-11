//! Recording mocks for driver unit tests. Each fake implements one of the
//! driver-owned interfaces and logs the calls it received; tests assert
//! against the log. Host-side only (`cfg(test)`), so `Vec` is available
//! via the explicit `extern crate std`.

extern crate std;
use std::vec::Vec;

use crate::traits::{
    ClockTrim, DigitalOut, DmaFlags, DmaRing, DxlTxScheduler, Monotonic, UsartBaud,
};
use crate::types::Level;

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
    pub log: Vec<u32>,
}

impl UsartBaud for FakeUsartBaud {
    // Same as the production V006 binding (PCLK = HCLK = 48 MHz) so test
    // BRR math matches the chip-side reference table.
    const CLOCK_HZ: u32 = 48_000_000;

    fn set_baud(&mut self, brr: u32) {
        self.log.push(brr);
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
#[derive(Default)]
pub struct FakeDmaRing {
    pub next_flags: DmaFlags,
    pub remaining: u16,
    pub ack_log: Vec<DmaFlags>,
}

impl DmaRing for FakeDmaRing {
    fn read_and_ack(&mut self) -> DmaFlags {
        let flags = self.next_flags;
        self.ack_log.push(flags);
        self.next_flags = DmaFlags::default();
        flags
    }

    fn remaining(&self) -> u16 {
        self.remaining
    }
}

/// One entry per `DxlTxScheduler` call; tests assert the recorded sequence
/// against expected fire arming.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ScheduleOp {
    Schedule {
        wire_end_tick: u16,
        delay_us: u32,
    },
    ScheduleLastSlot {
        wire_end_tick: u16,
        delay_q88_us: u32,
        anchor_bytes: u32,
    },
    Cancel,
}

#[derive(Default)]
pub struct FakeDxlTxScheduler {
    pub log: Vec<ScheduleOp>,
}

impl DxlTxScheduler for FakeDxlTxScheduler {
    fn schedule(&mut self, wire_end_tick: u16, delay_us: u32) {
        self.log.push(ScheduleOp::Schedule {
            wire_end_tick,
            delay_us,
        });
    }

    fn schedule_last_slot(&mut self, wire_end_tick: u16, delay_q88_us: u32, anchor_bytes: u32) {
        self.log.push(ScheduleOp::ScheduleLastSlot {
            wire_end_tick,
            delay_q88_us,
            anchor_bytes,
        });
    }

    fn cancel(&mut self) {
        self.log.push(ScheduleOp::Cancel);
    }
}

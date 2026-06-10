//! Recording adapters for unit tests. Each fake implements a driver
//! interface and logs the calls it received; the test asserts against the
//! log. Host-side only (`cfg(test)`), so `Vec` is available.

extern crate std;
use std::vec::Vec;

use crate::drivers::traits::{ClockTrim, DigitalOut, DmaFlags, DmaRing, Monotonic, UsartBaud};
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

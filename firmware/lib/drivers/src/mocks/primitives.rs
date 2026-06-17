use std::vec::Vec;

use crate::traits::{DigitalOut, Monotonic};
use crate::types::Level;

#[derive(Default)]
pub struct MockDigitalOut {
    pub log: Vec<Level>,
}

impl DigitalOut for MockDigitalOut {
    fn set(&mut self, level: Level) {
        self.log.push(level);
    }
}

/// Settable tick value; tests advance time by writing `now`. `TICKS_PER_US`
/// is 1 so test arithmetic stays in µs without scaling.
#[derive(Default)]
pub struct MockMonotonic {
    pub now: u32,
}

impl Monotonic for MockMonotonic {
    const TICKS_PER_US: u32 = 1;

    fn ticks(&self) -> u32 {
        self.now
    }
}

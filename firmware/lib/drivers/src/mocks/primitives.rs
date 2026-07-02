use mockall::mock;

use crate::traits::{DigitalOut, Level, Monotonic};

mock! {
    pub DigitalOut {}
    impl DigitalOut for DigitalOut {
        fn set(&mut self, level: Level);
    }
}

mock! {
    pub Monotonic {}
    impl Monotonic for Monotonic {
        // TICKS_PER_US is 1 so test arithmetic stays in µs without scaling.
        const TICKS_PER_US: u32 = 1;

        fn ticks(&self) -> u32;
    }
}

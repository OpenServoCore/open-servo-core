use mockall::mock;

use crate::traits::Monotonic;

mock! {
    pub Monotonic {}
    impl Monotonic for Monotonic {
        // TICKS_PER_US is 1 so test arithmetic stays in us without scaling.
        const TICKS_PER_US: u32 = 1;

        fn ticks(&self) -> u32;
    }
}

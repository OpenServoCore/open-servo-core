use mockall::mock;

use crate::traits::{DigitalOut, Level};

mock! {
    pub DigitalOut {}
    impl DigitalOut for DigitalOut {
        fn set(&mut self, level: Level);
    }
}

use mockall::mock;

use crate::traits::TelTx;

mock! {
    pub TelTx {}
    impl TelTx for TelTx {
        fn send(&mut self, bytes: &[u8]) -> bool;
    }
}

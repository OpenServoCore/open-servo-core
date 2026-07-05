use mockall::mock;

use crate::traits::dxl::Telemetry;

mock! {
    pub Telemetry {}
    impl Telemetry for Telemetry {
        fn record_crc_patch_deadline_miss(&mut self);
    }
}

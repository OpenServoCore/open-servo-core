use mockall::mock;

use crate::traits::dxl::Telemetry;

mock! {
    pub Telemetry {}
    impl Telemetry for Telemetry {
        fn record_edge_anchor_miss(&mut self);
        fn record_crc_patch_deadline_miss(&mut self);
    }
}

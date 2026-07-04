use mockall::mock;

use crate::traits::dxl::EdgeDma;

mock! {
    pub EdgeDma {}
    impl EdgeDma for EdgeDma {
        fn remaining(&self) -> u16;
        // `take_ring_restart` intentionally not mocked — the trait default
        // (`false`, "ring never restarted") is what every parser/codec test
        // wants; the restart path is covered by a hand-rolled double in
        // `edge_capture.rs` tests.
    }
}

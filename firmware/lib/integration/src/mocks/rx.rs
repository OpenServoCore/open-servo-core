use std::cell::Cell;
use std::rc::Rc;

use osc_drivers::mocks::MockRxDma;

/// Test-side stage for the Fast Last fold body's intra-loop NDTR refresh.
/// `stage_remaining(n)` models new bytes landing without re-entering the
/// codec's `on_rx_dma_advance`.
#[derive(Clone, Default)]
pub struct RxDmaState {
    remaining: Rc<Cell<u16>>,
}

impl RxDmaState {
    pub fn stage_remaining(&self, n: u16) {
        self.remaining.set(n);
    }
}

pub fn mock_rx_dma() -> (MockRxDma, RxDmaState) {
    let state = RxDmaState::default();
    let mut m = MockRxDma::new();
    {
        let r = state.remaining.clone();
        m.expect_remaining().returning_st(move || r.get());
    }
    (m, state)
}

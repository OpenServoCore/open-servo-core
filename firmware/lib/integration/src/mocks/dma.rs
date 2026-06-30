use std::cell::Cell;
use std::rc::Rc;

use osc_drivers::mocks::MockEdgeDma;

/// Tests stage the NDTR readback by writing `stage_remaining`; the
/// `remaining` closure returns it on each call.
#[derive(Clone, Default)]
pub struct EdgeDmaState {
    remaining: Rc<Cell<u16>>,
}

impl EdgeDmaState {
    pub fn stage_remaining(&self, n: u16) {
        self.remaining.set(n);
    }
}

pub fn mock_edge_dma() -> (MockEdgeDma, EdgeDmaState) {
    let state = EdgeDmaState::default();
    let mut m = MockEdgeDma::new();
    {
        let r = state.remaining.clone();
        m.expect_remaining().returning_st(move || r.get());
    }
    (m, state)
}

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use osc_drivers::mocks::{EdgeDmaOp, MockEdgeDma};
use osc_drivers::traits::dxl::DmaFlags;

/// Tests stage the next ISR view by writing the staging fields; the
/// `read_and_ack` closure returns the staged flags. `pause` / `resume`
/// toggle `paused` and append to `operations`; while `paused`,
/// `read_and_ack` returns `DmaFlags::default()` without consuming the
/// staged flags — mirroring production where masking HT/TC IE keeps the
/// IRQ from firing in the first place.
#[derive(Clone, Default)]
pub struct EdgeDmaState {
    next_flags: Rc<Cell<DmaFlags>>,
    remaining: Rc<Cell<u16>>,
    paused: Rc<Cell<bool>>,
    operations: Rc<RefCell<Vec<EdgeDmaOp>>>,
}

impl EdgeDmaState {
    pub fn stage_next_flags(&self, flags: DmaFlags) {
        self.next_flags.set(flags);
    }

    pub fn stage_remaining(&self, n: u16) {
        self.remaining.set(n);
    }

    pub fn paused(&self) -> bool {
        self.paused.get()
    }

    pub fn operations(&self) -> Vec<EdgeDmaOp> {
        self.operations.borrow().clone()
    }
}

pub fn mock_edge_dma() -> (MockEdgeDma, EdgeDmaState) {
    let state = EdgeDmaState::default();
    let mut m = MockEdgeDma::new();
    {
        let nf = state.next_flags.clone();
        let paused = state.paused.clone();
        m.expect_read_and_ack().returning_st(move || {
            if paused.get() {
                return DmaFlags::default();
            }
            let flags = nf.get();
            nf.set(DmaFlags::default());
            flags
        });
    }
    {
        let r = state.remaining.clone();
        m.expect_remaining().returning_st(move || r.get());
    }
    {
        let p = state.paused.clone();
        let ops = state.operations.clone();
        m.expect_pause().returning_st(move || {
            p.set(true);
            ops.borrow_mut().push(EdgeDmaOp::Pause);
        });
    }
    {
        let p = state.paused.clone();
        let ops = state.operations.clone();
        m.expect_resume().returning_st(move || {
            p.set(false);
            ops.borrow_mut().push(EdgeDmaOp::Resume);
        });
    }
    (m, state)
}

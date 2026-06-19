use std::cell::Cell;
use std::rc::Rc;

use osc_drivers::mocks::MockRxDma;
use osc_drivers::traits::dxl::DmaFlags;

/// Test-side stage for the byte-ring publish ISR and the Fast Last fold
/// body's intra-loop NDTR refresh. `stage_remaining(n)` models new bytes
/// landing; `stage_next_flags(f)` queues what the publish ISR's
/// `read_and_ack` reads. The flags are consumed on read — staged for one
/// publish, then back to default.
#[derive(Clone, Default)]
pub struct RxDmaState {
    next_flags: Rc<Cell<DmaFlags>>,
    remaining: Rc<Cell<u16>>,
}

impl RxDmaState {
    pub fn stage_next_flags(&self, flags: DmaFlags) {
        self.next_flags.set(flags);
    }

    pub fn stage_remaining(&self, n: u16) {
        self.remaining.set(n);
    }
}

pub fn mock_rx_dma() -> (MockRxDma, RxDmaState) {
    let state = RxDmaState::default();
    let mut m = MockRxDma::new();
    {
        let nf = state.next_flags.clone();
        m.expect_read_and_ack().returning_st(move || {
            let flags = nf.get();
            nf.set(DmaFlags::default());
            flags
        });
    }
    {
        let r = state.remaining.clone();
        m.expect_remaining().returning_st(move || r.get());
    }
    (m, state)
}

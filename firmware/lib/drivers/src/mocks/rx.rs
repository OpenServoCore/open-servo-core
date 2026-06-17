use core::cell::Cell;

use crate::traits::dxl::RxDma;

/// NDTR readback only — used by the Fast Last fold body's intra-loop
/// refresh. Tests write `remaining.set(n)` to model new bytes landing
/// without re-entering the codec's `on_rx_dma_advance`. `Cell` rather
/// than a plain field so the test can mutate from inside a closure
/// that borrows the bus immutably.
#[derive(Default)]
pub struct MockRxDma {
    pub remaining: Cell<u16>,
}

impl RxDma for MockRxDma {
    fn remaining(&self) -> u16 {
        self.remaining.get()
    }
}

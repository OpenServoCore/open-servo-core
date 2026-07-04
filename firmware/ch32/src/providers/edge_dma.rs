//! Edge-DMA provider — binds `EdgeDma` to DMA1_CH7 (TIM2_CH4 input-capture
//! → ET ring destination). The channel is time-shared with the TX kickoff
//! (doc §5/§6); the [`tx_kickoff`] module owns the window state, so both
//! reads route through it.
//!
//! [`tx_kickoff`]: super::tx_kickoff

use osc_drivers::traits::dxl;

use super::tx_kickoff;

/// Production binding to DMA1_CH7 (TIM2_CH4 input-capture → ET ring).
pub struct EdgeDma;

impl dxl::EdgeDma for EdgeDma {
    #[inline(always)]
    fn remaining(&self) -> u16 {
        tx_kickoff::edge_remaining()
    }

    #[inline(always)]
    fn take_ring_restart(&mut self) -> bool {
        tx_kickoff::take_ring_restart()
    }
}

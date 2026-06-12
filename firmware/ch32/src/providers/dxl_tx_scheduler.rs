//! TX fire scheduler — no-op stub. M3 (#5) wires the interior to TIM2_CH3
//! OC (fire ISR) + TIM2_CH2 OC (hardware TX_EN) per
//! `docs/dxl-hw-timed-transport.md` §5.
//!
//! Until M3 lands, production wire TX is non-functional: the driver still
//! decodes inbound requests, runs the dispatcher, encodes the reply into the
//! TX buffer, and calls into this provider — but `schedule` /
//! `schedule_last_slot` don't arm any hardware, so DMA1_CH4 never runs and
//! the wire stays idle. The rest of the stack (parse, dispatch, encode) is
//! fully load-bearing through this gap, per [[m2_legacy_drain_scope]].

use osc_drivers::traits::DxlTxScheduler as DxlTxSchedulerTrait;

pub struct DxlTxScheduler;

impl DxlTxSchedulerTrait for DxlTxScheduler {
    fn schedule(&mut self, _wire_end_tick: u16, _delay_us: u32) {
        // M3 (#5).
    }

    fn schedule_last_slot(&mut self, _wire_end_tick: u16, _delay_q88_us: u32, _anchor_bytes: u32) {
        // M3 (#5).
    }

    fn cancel(&mut self) {
        // M3 (#5).
    }
}

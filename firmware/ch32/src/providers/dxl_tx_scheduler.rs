//! TX fire scheduler — wraps the legacy SysTick-based scheduler until M3
//! (#5) replaces the interior with TIM2_CH3 OC (fire ISR) + TIM2_CH2 OC
//! (hardware TX_EN) per `docs/dxl-hw-timed-transport.md` §5.
//!
//! Until M2 (#33) reshapes `osc-core`'s `DxlBus` trait and rewires
//! `Ch32Bus` through `Drivers::dxl_uart()`, *nothing in production calls
//! this provider*. The legacy fire path in `legacy::dxl::scheduler` is
//! still invoked directly from `services::bus`. This impl exists so the
//! driver's `DxlUart<.., S, ..>` type parameter has a concrete production
//! provider on the chip side and the registry alias can be written, even
//! before any caller routes through it.
//!
//! Load-bearing dishonesty: the driver's `wire_end_tick: u16` comes from
//! TIM2 CNT (BT ring); the legacy scheduler reads SysTick (32-bit, HCLK).
//! `wire_end_tick as u32` truncates the upper bits — fine while no caller
//! exercises this path, but the wire-time arithmetic doesn't line up.
//! M3 swaps the interior to TIM2-native and the dishonesty goes away.

use osc_drivers::traits::DxlTxScheduler as DxlTxSchedulerTrait;

use crate::legacy::dxl::{scheduler as legacy_scheduler, state as legacy_state};

pub struct DxlTxScheduler;

impl DxlTxSchedulerTrait for DxlTxScheduler {
    fn schedule(&mut self, wire_end_tick: u16, delay_us: u32) {
        let request_end_tick = wire_end_tick as u32;
        legacy_scheduler::start_plain_after(request_end_tick, delay_us);
    }

    fn schedule_last_slot(&mut self, wire_end_tick: u16, delay_q88_us: u32, anchor_bytes: u32) {
        let request_end_tick = wire_end_tick as u32;
        legacy_scheduler::start_fast_after(request_end_tick, delay_q88_us, Some(anchor_bytes));
    }

    fn cancel(&mut self) {
        legacy_state::cancel();
    }
}

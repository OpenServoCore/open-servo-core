//! Field-write hook surface for the `ControlTable` derive.
//!
//! Lives inside `regions` because it's a control-table concern: the
//! macro-generated `dispatch_events` fires methods on this trait whenever a
//! committed write touches a tagged field. The single [`ControlTableHooks`]
//! translator forwards each new value to the bus's [`DxlReply::stage_*`]
//! deferred-apply queue.
//!
//! Kept `pub(crate)`: chip impls only see `DxlReply`; the field→reply wiring
//! stays an internal concern of the table layer.

use crate::regions::config::BaudRate;
use crate::traits::DxlReply;

pub(crate) trait ControlTableHookEvents {
    fn on_id_write(&mut self, value: u8);
    fn on_baud_rate_idx_write(&mut self, value: BaudRate);
    fn on_return_delay_2us_write(&mut self, value: u8);
}

pub(crate) struct ControlTableHooks<'a, R: DxlReply + ?Sized> {
    reply: &'a mut R,
}

impl<'a, R: DxlReply + ?Sized> ControlTableHooks<'a, R> {
    pub(crate) fn new(reply: &'a mut R) -> Self {
        Self { reply }
    }
}

/// DXL wire encodes RDT in 2-µs units; the trait surface carries plain µs.
const RDT_UNIT_US: u32 = 2;

fn rdt_us_from_units(value: u8) -> u32 {
    value as u32 * RDT_UNIT_US
}

impl<R: DxlReply + ?Sized> ControlTableHookEvents for ControlTableHooks<'_, R> {
    fn on_id_write(&mut self, value: u8) {
        self.reply.stage_id(value);
    }
    fn on_baud_rate_idx_write(&mut self, value: BaudRate) {
        self.reply.stage_baud(value);
    }
    fn on_return_delay_2us_write(&mut self, value: u8) {
        self.reply.stage_rdt(rdt_us_from_units(value));
    }
}

#[cfg(test)]
mod tests {
    use super::rdt_us_from_units;

    #[test]
    fn rdt_us_widens_and_scales_by_two() {
        assert_eq!(rdt_us_from_units(0), 0);
        assert_eq!(rdt_us_from_units(1), 2);
        assert_eq!(rdt_us_from_units(127), 254);
        // u8::MAX * 2 = 510, past u8's range — proves the widening cast
        // happens before the multiply.
        assert_eq!(rdt_us_from_units(u8::MAX), 510);
    }
}

//! Field-write hook surface for the `ControlTable` derive.
//!
//! Lives inside `regions` because it's a control-table concern: the
//! macro-generated `dispatch_events` fires methods on this trait whenever a
//! committed write touches a tagged field. The single [`ControlTableHooks`]
//! translator forwards each new value to the bus's [`Reply`] deferred-apply
//! queue.
//!
//! Kept `pub(crate)`: chip impls only see `Reply`; the field→reply wiring
//! stays an internal concern of the table layer.

use crate::regions::config::BaudRate;
use crate::traits::Reply;

pub(crate) trait ControlTableHookEvents {
    fn on_id_write(&mut self, value: u8);
    fn on_baud_rate_idx_write(&mut self, value: BaudRate);
    fn on_response_deadline_us_write(&mut self, value: u16);
}

pub(crate) struct ControlTableHooks<'a, R: Reply + ?Sized> {
    reply: &'a mut R,
}

impl<'a, R: Reply + ?Sized> ControlTableHooks<'a, R> {
    pub(crate) fn new(reply: &'a mut R) -> Self {
        Self { reply }
    }
}

impl<R: Reply + ?Sized> ControlTableHookEvents for ControlTableHooks<'_, R> {
    fn on_id_write(&mut self, value: u8) {
        self.reply.stage_id(value);
    }
    fn on_baud_rate_idx_write(&mut self, value: BaudRate) {
        self.reply.stage_baud(value);
    }
    fn on_response_deadline_us_write(&mut self, value: u16) {
        self.reply.set_response_deadline(value);
    }
}

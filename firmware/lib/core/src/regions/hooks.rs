//! Field-write hook surface for the `ControlTable` derive.
//!
//! Lives inside `regions` because it's a control-table concern: the
//! macro-generated `dispatch_events` fires methods on this trait whenever a
//! committed write touches a tagged field. The single [`ControlTableHooks`]
//! translator forwards each new value to the bus's [`Reply`] deferred-apply
//! queue.
//!
//! Kept `pub(crate)`: chip impls only see `Reply`; the field->reply wiring
//! stays an internal concern of the table layer.

use crate::regions::config::BaudRate;
use crate::tel::Chunks;
use crate::traits::Reply;

pub(crate) trait ControlTableHookEvents {
    fn on_id_write(&mut self, value: u8);
    fn on_baud_rate_idx_write(&mut self, value: u8);
    fn on_response_deadline_us_write(&mut self, value: u16);
    fn on_tel_count_write(&mut self, value: u16);
}

pub(crate) struct ControlTableHooks<'a, R: Reply + ?Sized> {
    reply: &'a mut R,
    /// The commit's last `tel_count`, armed at [`Self::finish`].
    tel_count: Option<u16>,
}

impl<'a, R: Reply + ?Sized> ControlTableHooks<'a, R> {
    pub(crate) fn new(reply: &'a mut R) -> Self {
        Self {
            reply,
            tel_count: None,
        }
    }

    /// After the commit's events, outside any table borrow: a `tel_count`
    /// write arms with the post-commit `chunks`, read only then; a refused
    /// arm zeroes `tel_count` in the bus (`Reply::tel_arm`).
    pub(crate) fn finish(self, chunks: impl FnOnce() -> Chunks) {
        if let Some(value) = self.tel_count {
            let chunks = chunks();
            // No row cannot arm; count 0 carries the disarm.
            let count = if chunks.valid() { value } else { 0 };
            self.reply.tel_arm(chunks, count);
        }
    }
}

impl<R: Reply + ?Sized> ControlTableHookEvents for ControlTableHooks<'_, R> {
    fn on_id_write(&mut self, value: u8) {
        self.reply.stage_id(value);
    }
    fn on_baud_rate_idx_write(&mut self, value: u8) {
        // The le=3 table rule makes from_idx infallible here; a violated
        // invariant drops the stage rather than panicking.
        if let Some(baud) = BaudRate::from_idx(value) {
            self.reply.stage_baud(baud);
        }
    }
    fn on_response_deadline_us_write(&mut self, value: u16) {
        self.reply.set_response_deadline(value);
    }
    fn on_tel_count_write(&mut self, value: u16) {
        self.tel_count = Some(value);
    }
}

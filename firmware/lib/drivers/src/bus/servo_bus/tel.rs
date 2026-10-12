//! TEL burst arm (protocol sec 5.6): a committed nonzero `tel_count` write
//! arms it, and the bus lends the kernel the TX lane (`crate::tel`) at the
//! arming ack's release, or at the arm when no ack follows. The servo owns
//! the line from there to the LAST frame: the bus serves nothing until it
//! takes the lane back, in its first body after LAST has left the wire.

use osc_servo_core::BaudRate;

use crate::tel::TelDrain;

pub(super) struct TelBurst {
    drain: Option<TelDrain>,
    /// The arm the next lend streams.
    mask: u16,
    count: u16,
    /// The lent burst's epoch.
    epoch: u8,
}

impl TelBurst {
    pub(super) const fn new() -> Self {
        Self {
            drain: None,
            mask: 0,
            count: 0,
            epoch: 0,
        }
    }

    pub(super) fn attach(&mut self, drain: TelDrain) {
        self.drain = Some(drain);
    }

    /// `Reply::tel_arm`: true when the arm wants the lane (count 0 disarms).
    pub(super) fn arm(&mut self, mask: u16, count: u16) -> bool {
        self.mask = mask;
        self.count = count;
        count != 0 && self.drain.is_some()
    }

    /// Lend the lane: the kernel streams the arm from its next tick.
    pub(super) fn start(&mut self, id: u8, rate: BaudRate) {
        if let Some(d) = self.drain.as_mut() {
            self.epoch = d.start(self.mask, self.count, id, rate);
        }
    }

    /// The RX ring cursor to resume at, once the LAST frame has left the
    /// wire. `idle` is read after the kernel's LAST notice: read the other
    /// way round, a gap before LAST would pass for its end.
    pub(super) fn returned(&self, idle: impl FnOnce() -> bool) -> Option<u16> {
        let resume = self.drain.as_ref()?.last_out(self.epoch)?;
        idle().then_some(resume)
    }

    pub(super) fn stop(&mut self) {
        if let Some(d) = self.drain.as_mut() {
            d.stop();
        }
    }
}

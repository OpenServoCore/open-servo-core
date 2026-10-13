//! TEL burst arm (protocol sec 5.6): a committed nonzero `tel_count` write
//! arms it, and the bus lends the kernel the TX lane (`crate::tel`) at the
//! arming ack's release, or at the arm when no ack follows. The servo owns
//! the line from there to the LAST frame: the bus serves nothing until it
//! takes the lane back, in its first body after LAST has left the wire.

use osc_servo_core::tel::Chunks;

use crate::tel::TelPost;

pub(super) struct TelBurst {
    post: Option<TelPost>,
    /// The arm the next lend streams; its chunks are posted.
    count: u16,
    /// The lent burst's epoch.
    epoch: u8,
}

impl TelBurst {
    pub(super) const fn new() -> Self {
        Self {
            post: None,
            count: 0,
            epoch: 0,
        }
    }

    pub(super) fn attach(&mut self, post: TelPost) {
        self.post = Some(post);
    }

    /// `Reply::tel_arm`, never while the lane is lent: true when the arm
    /// wants the lane, its chunks then posted; count 0 disarms and ends it.
    pub(super) fn arm(&mut self, chunks: Chunks, count: u16) -> bool {
        self.count = count;
        let Some(d) = self.post.as_mut() else {
            return false;
        };
        if count == 0 {
            d.end();
            return false;
        }
        d.post(chunks);
        true
    }

    /// The posted arm will not stream: its lend was dropped with the ack.
    pub(super) fn void(&mut self) {
        if let Some(d) = self.post.as_mut() {
            d.end();
        }
    }

    /// Lend the lane: the kernel streams the arm from its next tick.
    pub(super) fn start(&mut self, id: u8) {
        if let Some(d) = self.post.as_mut() {
            self.epoch = d.start(self.count, id);
        }
    }

    /// The RX ring cursor to resume at, once the LAST frame has left the
    /// wire. `idle` is read after the kernel's LAST notice: read the other
    /// way round, a gap before LAST would pass for its end.
    pub(super) fn returned(&self, idle: impl FnOnce() -> bool) -> Option<u16> {
        let resume = self.post.as_ref()?.last_out(self.epoch)?;
        idle().then_some(resume)
    }

    /// The lane is back: the burst is over.
    pub(super) fn stop(&mut self) {
        if let Some(d) = self.post.as_mut() {
            d.stop();
            d.end();
        }
    }
}

//! Staged-then-commit mediator between the DXL dispatcher (CONFIG-region
//! writes + Reboot instructions) and the live values that govern bus
//! visibility. Every value with a `pending_*` twin lives here; the servo's
//! own TX completion is the one safe commit window — mutating `id` or RDT
//! while a reply is still scheduled against the old values would tear the
//! in-flight exchange.

use osc_core::BootMode;

pub(super) struct ConfigMediator {
    /// Live servo ID — the byte instruction targeting matches against.
    id: u8,
    /// Live Return Delay Time in µs — the base of every reply deadline.
    rdt_us: u32,
    pending_id: Option<u8>,
    pending_rdt_us: Option<u32>,
    /// Staged reboot request. A runtime signal rather than config, but it
    /// shares the exact commit gate (stage on parse, honor after our ACK
    /// leaves the wire), so it co-lives here instead of earning a
    /// sub-driver of its own.
    pending_reboot: Option<BootMode>,
}

impl ConfigMediator {
    pub(super) const fn new(id: u8, rdt_us: u32) -> Self {
        Self {
            id,
            rdt_us,
            pending_id: None,
            pending_rdt_us: None,
            pending_reboot: None,
        }
    }
}

// -- events -------------------------------------------------------------

impl ConfigMediator {
    /// The reply carrying the config write's ACK has fully drained the
    /// wire — commit staged values and surface any staged reboot for the
    /// composite to route chip-side.
    pub(super) fn on_tx_complete(&mut self) -> Option<BootMode> {
        if let Some(id) = self.pending_id.take() {
            self.id = id;
        }
        if let Some(rdt) = self.pending_rdt_us.take() {
            self.rdt_us = rdt;
        }
        self.pending_reboot.take()
    }
}

// -- commands -----------------------------------------------------------

impl ConfigMediator {
    /// Stage a deferred ID change — no-op when unchanged, so a same-value
    /// write doesn't create pending state.
    pub(super) fn stage_id(&mut self, id: u8) {
        if id != self.id {
            self.pending_id = Some(id);
        }
    }

    /// Stage a deferred Return Delay Time change in µs — no-op when
    /// unchanged.
    pub(super) fn stage_rdt(&mut self, us: u32) {
        if us != self.rdt_us {
            self.pending_rdt_us = Some(us);
        }
    }

    /// Stage a deferred reboot, honored after any in-flight TX drains.
    pub(super) fn stage_reboot(&mut self, mode: BootMode) {
        self.pending_reboot = Some(mode);
    }
}

// -- accessors ----------------------------------------------------------

impl ConfigMediator {
    pub(super) fn id(&self) -> u8 {
        self.id
    }

    pub(super) fn rdt_us(&self) -> u32 {
        self.rdt_us
    }
}

#[cfg(test)]
impl ConfigMediator {
    pub(super) fn pending_id(&self) -> Option<u8> {
        self.pending_id
    }

    pub(super) fn pending_reboot(&self) -> Option<BootMode> {
        self.pending_reboot
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn staged_values_commit_only_at_tx_complete() {
        let mut c = ConfigMediator::new(1, 250);
        c.stage_id(0x42);
        c.stage_rdt(500);
        assert_eq!(c.id(), 1);
        assert_eq!(c.rdt_us(), 250);
        assert_eq!(c.on_tx_complete(), None);
        assert_eq!(c.id(), 0x42);
        assert_eq!(c.rdt_us(), 500);
    }

    #[test]
    fn same_value_stage_is_a_noop() {
        let mut c = ConfigMediator::new(1, 250);
        c.stage_id(1);
        c.stage_rdt(250);
        assert!(c.pending_id().is_none());
        c.on_tx_complete();
        assert_eq!(c.id(), 1);
        assert_eq!(c.rdt_us(), 250);
    }

    #[test]
    fn reboot_surfaces_once() {
        let mut c = ConfigMediator::new(1, 250);
        c.stage_reboot(BootMode::Bootloader);
        assert_eq!(c.on_tx_complete(), Some(BootMode::Bootloader));
        assert_eq!(c.on_tx_complete(), None);
    }
}

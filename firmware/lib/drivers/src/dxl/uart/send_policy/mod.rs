//! Send-side policy sub-composite — the state that decides how this servo
//! participates in a DXL exchange, split out of the `DxlUart` composite per
//! driver-pattern §4.3 so wire routing and send policy stop sharing one
//! struct. Owns [`ConfigMediator`] (bus identity + staged-config mailbox).

mod config_mediator;

use config_mediator::ConfigMediator;
use osc_core::BootMode;

pub(super) struct SendPolicy {
    config: ConfigMediator,
}

impl SendPolicy {
    pub(super) const fn new(id: u8, rdt_us: u32) -> Self {
        Self {
            config: ConfigMediator::new(id, rdt_us),
        }
    }
}

// -- events -------------------------------------------------------------

impl SendPolicy {
    /// Our reply fully drained the wire — the safe commit window for
    /// staged config. Returns any staged reboot for the composite to
    /// route chip-side.
    pub(super) fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.config.on_tx_complete()
    }
}

// -- commands -----------------------------------------------------------

impl SendPolicy {
    pub(super) fn stage_id(&mut self, id: u8) {
        self.config.stage_id(id)
    }

    pub(super) fn stage_rdt(&mut self, us: u32) {
        self.config.stage_rdt(us)
    }

    pub(super) fn stage_reboot(&mut self, mode: BootMode) {
        self.config.stage_reboot(mode)
    }
}

// -- accessors ----------------------------------------------------------

impl SendPolicy {
    pub(super) fn id(&self) -> u8 {
        self.config.id()
    }

    pub(super) fn rdt_us(&self) -> u32 {
        self.config.rdt_us()
    }
}

#[cfg(test)]
impl SendPolicy {
    pub(super) fn pending_id(&self) -> Option<u8> {
        self.config.pending_id()
    }

    pub(super) fn pending_reboot(&self) -> Option<BootMode> {
        self.config.pending_reboot()
    }
}

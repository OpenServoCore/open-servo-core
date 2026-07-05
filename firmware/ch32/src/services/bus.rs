//! `Ch32Bus` — thin forwarder onto `Drivers::dxl_uart()`. All decoding,
//! reply encoding, scheduling, and staged-config tracking live in the
//! driver (`osc-drivers::dxl::uart`); this struct exists only to satisfy
//! the chip-agnostic [`osc_core::DxlBus`] trait by feeding each parser
//! event through to the services-layer dispatcher.

use osc_core::DxlBus;
use osc_core::traits::DxlDispatch;

use crate::runtime::registry::Drivers;

/// Sole `&mut` writer: the main loop holding the `Services` struct.
pub struct Ch32Bus;

impl Ch32Bus {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for Ch32Bus {
    fn default() -> Self {
        Self::new()
    }
}

impl DxlBus for Ch32Bus {
    fn poll<D: DxlDispatch>(&mut self, dispatcher: &mut D) {
        // SAFETY: services-layer caller holds the single `&mut Ch32Bus`
        // through `Services<Ch32Bus>`; the registry's `dxl_uart()` accessor
        // is documented as main-loop-or-same-priority-ISR only. The
        // dispatcher closure runs synchronously inside this call and
        // surrenders the borrow before returning.
        unsafe { Drivers::dxl_uart() }.poll(|req, ctx, reply| {
            dispatcher.dispatch(req, ctx, reply);
        });
    }
}

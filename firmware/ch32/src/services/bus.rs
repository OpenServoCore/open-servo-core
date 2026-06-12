//! `Ch32Bus` — thin forwarder onto `Drivers::dxl_uart()`. All decoding,
//! reply encoding, scheduling, and staged-config tracking live in the
//! driver (`osc-drivers::dxl::uart`); this struct exists only to satisfy
//! the chip-agnostic [`osc_core::DxlBus`] trait by handing the driver's
//! closure-based `poll` through to the services layer.
//!
//! Wire TX is non-functional from M2 → M3 (#33 → #5): the driver still
//! encodes replies + invokes the scheduler stub, but the stub doesn't fire
//! DMA1_CH4. The trait shape and the dispatcher flow are fully load-bearing.

use osc_core::DxlBus;

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
    fn poll<F>(&mut self, f: F)
    where
        F: for<'a> FnOnce(dxl_protocol::InstructionPacket<'a>, &mut dyn osc_core::DxlReply),
    {
        // SAFETY: services-layer caller holds the single `&mut Ch32Bus`
        // through `Services<Ch32Bus>`; the registry's `dxl_uart()` accessor
        // is documented as main-loop-or-same-priority-ISR only. The
        // dispatcher closure runs synchronously inside this call and
        // surrenders the borrow before returning.
        unsafe { Drivers::dxl_uart() }.poll(|packet, reply| {
            // reply: &mut ReplyHandle<'_, ..>, which impls DxlReply —
            // implicit `&mut dyn DxlReply` coercion at the call site.
            f(packet, reply);
        });
    }
}

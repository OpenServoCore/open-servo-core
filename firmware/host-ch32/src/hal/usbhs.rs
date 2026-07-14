//! USBHS controller primitives: block views (the device/host register
//! files overlay one base) and the SIE bring-up sequence. Endpoint
//! choreography and the control FSM live in `runtime::usb` -- this file is
//! registers only.

use ch32_metapac::usbhs::vals::SpeedType;
use ch32_metapac::usbhs::{Usb, Usbd, Usbh};
use ch32_metapac::{RCC, USBHS};

#[inline(always)]
pub const fn regs() -> Usb {
    USBHS
}

#[inline(always)]
pub fn dregs() -> Usbd {
    // SAFETY: the device register file overlays the USBHS base; the pac
    // exposes only the common block, so the view is constructed here.
    unsafe { Usbd::from_ptr(USBHS.as_ptr()) }
}

#[inline(always)]
pub fn hregs() -> Usbh {
    // SAFETY: as `dregs` -- the host file overlays the same base (only its
    // PHY control bit is used in device mode).
    unsafe { Usbh::from_ptr(USBHS.as_ptr()) }
}

/// SIE reset spec: >= 10 us between reset assert and release.
const SIE_RESET_SPINS: u32 = crate::hal::rcc::SYSCLK_HZ / 100_000;

/// Clock-gate + SIE reset + device-mode base config. The PHY's PLL
/// (`rcc::init_usbhs_pll`) must already be alive. `int_en` stays zero: the
/// stack is polled off `int_fg`, no PFIC vector.
pub fn init_device() {
    RCC.ahbpcenr().modify(|w| w.set_usbhsen(true));
    RCC.ahbrstr().modify(|w| w.set_usbhsrst(true));
    RCC.ahbrstr().modify(|w| w.set_usbhsrst(false));

    let r = regs();
    r.ctrl().write(|w| {
        w.set_clr_all(true);
        w.set_reset_sie(true);
    });
    let mut spins = SIE_RESET_SPINS;
    while spins > 0 {
        spins -= 1;
        core::hint::spin_loop();
    }
    r.ctrl().modify(|w| w.set_reset_sie(false));
    hregs().ctrl().write(|w| w.set_phy_suspendm(true));

    r.ctrl().write(|w| {
        w.set_int_busy(true); // auto-NAK until the pending flag is serviced
        w.set_dma_en(true);
        w.set_speed_type(SpeedType::HIGHSPEED);
    });
    r.int_en().write_value(ch32_metapac::usbhs::regs::IntEn(0));
}

/// Present the pull-up: the host sees an attach and starts enumeration.
pub fn attach() {
    regs().ctrl().modify(|w| w.set_dev_pu_en(true));
}

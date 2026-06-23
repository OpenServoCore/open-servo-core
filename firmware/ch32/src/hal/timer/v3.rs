use ch32_metapac::timer::vals::{CcmrInputCcs, Ckd, Cms, FilterValue, Mms, Ocm};
use ch32_metapac::{TIM1, TIM2};

use crate::hal::clocks::TIM_CLK_HZ;

#[derive(Copy, Clone)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

/// Center-aligned period = `2 * arr * (psc + 1)` timer ticks; picks smallest psc that fits ARR in u16.
pub const fn pwm_dividers_from_hz(freq_hz: u32) -> (u16, u16) {
    let denom = 2 * freq_hz;
    let mut psc: u32 = 0;
    loop {
        let arr = TIM_CLK_HZ / (denom * (psc + 1));
        if arr > 0 && arr <= u16::MAX as u32 {
            return (psc as u16, arr as u16);
        }
        psc += 1;
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
}

/// Sets ARPE: CCR/ARR writes take effect at next UEV, not immediately.
pub fn init_center_aligned_pwm(prescaler: u16, period: u16) {
    TIM1.psc().write_value(prescaler);
    TIM1.atrlr().write_value(period);
    TIM1.ctlr1().modify(|w| {
        w.set_cms(Cms::CENTERALIGNED3);
        w.set_arpe(true);
    });
}

pub fn configure_pwm_channel(ch: Channel, polarity: Polarity) {
    let n = (ch as u8 - 1) as usize;
    TIM1.chctlr_output(n / 2).modify(|w| {
        w.set_ocm(n % 2, Ocm::PWMMODE1);
        w.set_ocpe(n % 2, true);
    });
    TIM1.ccer().modify(|w| {
        w.set_cce(n, true);
        w.set_ccp(n, matches!(polarity, Polarity::ActiveLow));
    });
}

#[inline]
pub fn set_duty(ch: Channel, value: u16) {
    TIM1.chcvr((ch as u8 - 1) as usize).write_value(value);
}

/// RCR=0 with center-aligned → UEV at both peak and trough.
pub fn set_repetition(rcr: u8) {
    TIM1.rptcr().write(|w| w.set_rptcr(rcr));
}

pub fn set_trgo_update() {
    TIM1.ctlr2().modify(|w| w.set_mms(Mms::UPDATE));
}

/// Fires TRGO as a side effect — arm ADC after, not before.
pub fn force_update_event() {
    TIM1.swevgr().write(|w| w.set_ug(true));
}

pub fn enable_main_output() {
    TIM1.bdtr().modify(|w| w.set_moe(true));
}

pub fn start() {
    TIM1.ctlr1().modify(|w| w.set_cen(true));
}

/// Free-run TIM2 @ HCLK (ARR=0xFFFF, PSC=0) with CH4 as a falling-edge
/// input capture, no input prescaler, baud-derived input filter. Each
/// capture latches the 16-bit TIM2 count into CCR4 and fires a DMA1_CH7
/// request (CCDE.CC4). Caller wires PC1 to TIM2 via the AFIO remap and
/// picks `boot_filter` for the baud at boot; this just programs the timer.
///
/// CKD=0 pins `fDTS = fck_int = HCLK = 48 MHz`. RM default is already 0,
/// but the IC4F picker math (`docs/dxl-hw-timed-transport.md` §8.1) is
/// only valid under that assumption — defensive coupling.
pub fn init_tim2_ch4_ic_capture(boot_filter: FilterValue) {
    TIM2.psc().write_value(0);
    TIM2.atrlr().write_value(0xFFFF);
    TIM2.ctlr1().modify(|w| w.set_ckd(Ckd::DIV_1));
    TIM2.chctlr_input(1).modify(|w| {
        w.set_ccs(1, CcmrInputCcs::TI4);
        w.set_icpsc(1, 0);
        w.set_icf(1, boot_filter);
    });
    TIM2.ccer().modify(|w| {
        w.set_ccp(3, true);
        w.set_cce(3, true);
    });
    TIM2.dmaintenr().modify(|w| w.set_ccde(3, true));
    // `EdgeParser` lifts u16 IC4 stamps to u32 wire-time against
    // `WireClock::now()` (SysTick u32) by assuming the low 16 bits match.
    // The two counters share HCLK, so co-zeroing them in adjacent writes
    // immediately before TIM2.CEN=true locks the invariant in. Residual
    // offset is the HCLK gap between this SysTick CNT store and the TIM2
    // CEN store taking effect — one APB1 RMW, ~10 HCLK cycles, sub-tick
    // at any DXL baud. TIM2.CNT is already 0 (peripheral reset, CEN was
    // never on). Post-boot: no code may reset CNT on either timer or the
    // lift contract is broken silently.
    crate::hal::systick::reset_cnt();
    TIM2.ctlr1().modify(|w| w.set_cen(true));
}

/// Update the CH4 input-capture filter (IC4F) — the runtime mutation path
/// invoked by the USART baud provider at `apply_baud`.
#[inline]
pub fn set_tim2_ch4_icf(value: FilterValue) {
    TIM2.chctlr_input(1).modify(|w| w.set_icf(1, value));
}

/// Peripheral address of TIM2.CCR4 for the DMA1_CH7 PAR. CCR4 is the 16-bit
/// capture register — `chcvr(3)` is metapac's zero-based accessor.
#[inline]
pub fn tim2_ch4_capture_addr() -> u32 {
    TIM2.chcvr(3).as_ptr() as u32
}

/// CH2 + CH3 as output-compare on top of the already-running TIM2 free-run.
/// CH2 lives on PC2 (TX_EN, hardware-driven on CC2 match) under the
/// board's Remap2; CH3 has no pin output — only its CC3IF interrupt fires
/// the wire-driver activate sequence. Caller hands the TX_EN polarity from
/// the board wiring so CCMR1/CCER reflect it.
///
/// Both channels start with idle settings (OC2M=Force-inactive, OC3M=Frozen,
/// CC3IE off). The provider arms each fire by writing CCRs and flipping
/// OC2M to Active-on-match + enabling CC3IE.
pub fn init_tim2_tx_oc_channels(tx_active_high: bool) {
    TIM2.chctlr_output(0).modify(|w| {
        w.set_ccs(1, ch32_metapac::timer::vals::CcmrOutputCcs::OUTPUT);
        w.set_ocm(1, Ocm::FORCEINACTIVE);
        w.set_ocpe(1, false);
    });
    TIM2.chctlr_output(1).modify(|w| {
        w.set_ccs(0, ch32_metapac::timer::vals::CcmrOutputCcs::OUTPUT);
        w.set_ocm(0, Ocm::FROZEN);
        w.set_ocpe(0, false);
    });
    TIM2.ccer().modify(|w| {
        w.set_cce(1, true);
        w.set_ccp(1, !tx_active_high);
        w.set_cce(2, false);
    });
    TIM2.dmaintenr().modify(|w| w.set_ccie(2, false));
    TIM2.intfr().write(|w| {
        w.0 = !(1u32 << 3);
    });
}

#[inline]
pub fn set_tim2_ccr2(value: u16) {
    TIM2.chcvr(1).write_value(value);
}

#[inline]
pub fn set_tim2_ccr3(value: u16) {
    TIM2.chcvr(2).write_value(value);
}

/// `OC2M = Active-on-match`: PC2 rises at the next CC2 compare match. Wire-edge
/// landing is hardware-timed — no ISR in the path.
#[inline]
pub fn tim2_ch2_active_on_match() {
    TIM2.chctlr_output(0)
        .modify(|w| w.set_ocm(1, Ocm::ACTIVEONMATCH));
}

/// `OC2M = Force-inactive`: PC2 drops immediately. Used to release TX_EN at
/// TC and on `cancel`.
#[inline]
pub fn tim2_ch2_force_inactive() {
    TIM2.chctlr_output(0)
        .modify(|w| w.set_ocm(1, Ocm::FORCEINACTIVE));
}

/// `OC2M = Force-active`: PC2 rises immediately. Used by the set-and-recheck
/// manual-fire path when CCR2 has already wrapped past CNT.
#[inline]
pub fn tim2_ch2_force_active() {
    TIM2.chctlr_output(0)
        .modify(|w| w.set_ocm(1, Ocm::FORCEACTIVE));
}

#[inline]
pub fn enable_tim2_cc3_irq(enable: bool) {
    TIM2.dmaintenr().modify(|w| w.set_ccie(2, enable));
}

/// CC3IF is rc_w0 — write 0 to clear, 1 to leave alone. CC2IF (bit 2) gets
/// preserved alongside other flags via the `!0u32 & !mask` pattern.
#[inline]
pub fn clear_tim2_cc3_flag() {
    TIM2.intfr().write(|w| {
        w.0 = !(1u32 << 3);
    });
}

#[inline]
pub fn tim2_cnt() -> u16 {
    TIM2.cnt().read()
}

#[inline]
pub fn tim2_ccr3() -> u16 {
    TIM2.chcvr(2).read()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn realised_hz(psc: u16, arr: u16) -> u32 {
        TIM_CLK_HZ / (2 * (psc as u32 + 1) * arr as u32)
    }

    #[test]
    fn pwm_dividers_pin_production_freq() {
        let (psc, arr) = pwm_dividers_from_hz(20_000);
        assert_eq!((psc, arr), (0, 1200));
        assert_eq!(realised_hz(psc, arr), 20_000);
    }

    #[test]
    fn pwm_dividers_fits_low_freq() {
        // 50 Hz needs a prescaler; product (psc+1)*arr fits u32.
        let (psc, arr) = pwm_dividers_from_hz(50);
        assert!(arr > 0);
        let realised = realised_hz(psc, arr);
        assert!((45..=55).contains(&realised), "got {realised}");
    }

    #[test]
    fn pwm_dividers_high_freq() {
        let (psc, arr) = pwm_dividers_from_hz(100_000);
        assert_eq!(psc, 0);
        assert_eq!(realised_hz(psc, arr), 100_000);
    }
}

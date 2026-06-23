//! Bench-tuning stamps that record latency maxima into the
//! `TelemetryDxlTune` register block. Read by the B5 / B6 tune binaries
//! to derive paste-ready values for `firmware/ch32/src/measurements.rs`.
//!
//! Bodies live behind `#[cfg(feature = "tuning")]`; the helpers always
//! compile but are no-ops in production. Call sites in the IRS / scheduler
//! providers are also `cfg`-gated so the timestamp reads themselves cost
//! nothing in production builds — keeps the measured-baseline path
//! byte-identical to today's `[[isr_ordering]]`-tuned vector bodies.

#[cfg(feature = "tuning")]
use crate::runtime::statics::SHARED;

/// Saturating-min update of
/// [`crate::regions::telemetry::TelemetryDxlTune::tx_start_entry_min`].
/// `cur == 0` doubles as the "no sample yet" sentinel so the first
/// non-zero delta always lands. The CCR3 back-date math requires the
/// const to be sized to the MIN of observed entry latencies — see the
/// field docstring on `TelemetryDxlTune`.
///
/// SAFETY: telemetry SRAM is install-time initialised before any IRQ
/// unmask; all DXL ISRs (USART1 / DMA1_CH5 / DMA1_CH7 / TIM2 / SysTick)
/// share PFIC HIGH so cannot preempt each other. Host-side clears travel
/// the regmap and serialise against the IRQ path the same way the
/// `TelemetryDxlLink` carve-out already does.
#[inline]
pub fn record_tx_start_entry(delta: u16) {
    #[cfg(feature = "tuning")]
    unsafe {
        let p = &raw mut (*SHARED.table.telemetry.get()).tune.tx_start_entry_min;
        let cur = p.read_volatile();
        if cur == 0 || delta < cur {
            p.write_volatile(delta);
        }
    }
    #[cfg(not(feature = "tuning"))]
    let _ = delta;
}

/// Saturating-min update of `TelemetryDxlTune::fast_last_entry_min`.
/// See [`record_tx_start_entry`] for the SAFETY argument and the
/// rationale behind capturing the MIN rather than the MAX.
#[inline]
pub fn record_fast_last_entry(delta: u16) {
    #[cfg(feature = "tuning")]
    unsafe {
        let p = &raw mut (*SHARED.table.telemetry.get()).tune.fast_last_entry_min;
        let cur = p.read_volatile();
        if cur == 0 || delta < cur {
            p.write_volatile(delta);
        }
    }
    #[cfg(not(feature = "tuning"))]
    let _ = delta;
}

/// Saturating-max update of `TelemetryDxlTune::schedule_remaining_max`.
/// MAX direction here — the wrap-guard threshold is sized as an upper
/// bound on legitimate remainings. See [`record_tx_start_entry`] for the
/// SAFETY argument.
#[inline]
pub fn record_schedule_remaining(delta: u16) {
    #[cfg(feature = "tuning")]
    unsafe {
        let p = &raw mut (*SHARED.table.telemetry.get()).tune.schedule_remaining_max;
        let cur = p.read_volatile();
        if delta > cur {
            p.write_volatile(delta);
        }
    }
    #[cfg(not(feature = "tuning"))]
    let _ = delta;
}

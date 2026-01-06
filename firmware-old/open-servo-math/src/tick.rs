//! Tick domain types for structured timing.

/// Identifies which timing domain a tick belongs to.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TickDomain {
    /// Hard real-time control loop (ControlFast rate from ADC DMA).
    ControlFast,
    /// Decimated control loop (1kHz, derived from ControlFast).
    ControlMedium,
    /// System housekeeping (100Hz from TIM2).
    System,
}

/// Context passed to tick handlers.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TickCtx {
    /// Which domain this tick belongs to.
    pub domain: TickDomain,
    /// Delta time in microseconds for this tick.
    pub dt_us: u32,
    /// Monotonic sequence counter for this domain.
    pub seq: u32,
}

//! Current sensor traits.

use open_servo_math::MilliAmp;

/// Bus current sensing (for BDC motors, single shunt).
///
/// Boards with current sensing implement this trait.
pub trait BusCurrentSensor {
    /// Read bus current in milliamps.
    fn read_bus_current(&self) -> MilliAmp;

    /// Read raw current ADC value.
    fn read_bus_current_raw(&self) -> u16;
}

/// Safety capability trait for current sensing.
///
/// ALL boards implement this trait. Boards with current sensing get
/// automatic implementation via blanket impl. Boards without current
/// sensing implement this directly, returning `None`.
pub trait SafetyCurrentSource {
    /// Read current for safety checks.
    ///
    /// Returns `None` if the board has no current sensor,
    /// which causes SafetyManager to skip over-current checks.
    fn read_safety_current(&self) -> Option<MilliAmp>;
}

// Blanket impl: BusCurrentSensor automatically provides SafetyCurrentSource
impl<T: BusCurrentSensor> SafetyCurrentSource for T {
    #[inline]
    fn read_safety_current(&self) -> Option<MilliAmp> {
        Some(self.read_bus_current())
    }
}

/// Phase current sensing (for BLDC FOC, 2-3 shunts).
///
/// Used for field-oriented control where per-phase current is needed.
pub trait PhaseCurrentSensor {
    /// Read phase currents (A, B, C) in milliamps.
    ///
    /// Returns (Ia, Ib, Ic). Third may be calculated from Ia + Ib + Ic = 0.
    fn read_phase_currents(&self) -> (MilliAmp, MilliAmp, Option<MilliAmp>);
}

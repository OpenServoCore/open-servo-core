use core::ops::{Add, Sub};

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SimTime(u64);

impl SimTime {
    pub const ZERO: Self = Self(0);

    pub const fn from_ns(ns: u64) -> Self {
        Self(ns)
    }

    pub const fn from_us(us: u64) -> Self {
        Self(us * 1_000)
    }

    pub const fn from_ms(ms: u64) -> Self {
        Self(ms * 1_000_000)
    }

    pub const fn as_ns(self) -> u64 {
        self.0
    }
}

impl Add<u64> for SimTime {
    type Output = Self;
    fn add(self, ns: u64) -> Self {
        Self(self.0 + ns)
    }
}

impl Add for SimTime {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0)
    }
}

impl Sub for SimTime {
    type Output = u64;
    fn sub(self, other: Self) -> u64 {
        self.0 - other.0
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Clock {
    freq_hz: u32,
}

impl Clock {
    pub const fn new(freq_hz: u32) -> Self {
        Self { freq_hz }
    }

    pub const fn freq_hz(self) -> u32 {
        self.freq_hz
    }

    pub fn to_local(self, t: SimTime) -> u64 {
        (t.as_ns() as u128 * self.freq_hz as u128 / 1_000_000_000) as u64
    }

    pub fn from_local(self, local_ticks: u64) -> SimTime {
        SimTime((local_ticks as u128 * 1_000_000_000 / self.freq_hz as u128) as u64)
    }
}

/// Chip-blind HSI model. Composes two independent rate offsets against a
/// `nominal` factory-spec clock:
///
/// - **Factory drift** — `(num, den)` ratio set once by the test via
///   `set_factory_drift`. Models "this chip's HSI happens to sit X% off
///   factory spec." Survives across simulated power cycles and trim
///   adjustments.
/// - **Applied trim** — `applied_ppm` set by `drain_ops`, mirroring the
///   absolute correction the driver-side integrator last emitted to
///   `MockClockTrim::apply_ppm`. Runtime-mutable; converges toward the
///   negation of the factory drift as the integrator's auto-trim loop runs.
///
/// `live()` composes the two into the effective HCLK the rest of the sim
/// stamps wire events against. No chip-side constants (HSI step Hz, register
/// width) appear in this type — the only chip-aware code is the V006
/// provider's ppm → register quantization.
#[derive(Clone, Debug)]
pub struct HsiClock {
    nominal: Clock,
    factory_drift_num: i32,
    factory_drift_den: u32,
    applied_ppm: i32,
    drained: usize,
}

impl HsiClock {
    pub fn new(nominal: Clock) -> Self {
        Self {
            nominal,
            factory_drift_num: 0,
            factory_drift_den: 1,
            applied_ppm: 0,
            drained: 0,
        }
    }

    pub fn nominal(&self) -> Clock {
        self.nominal
    }

    pub fn factory_drift(&self) -> (i32, u32) {
        (self.factory_drift_num, self.factory_drift_den)
    }

    pub fn applied_ppm(&self) -> i32 {
        self.applied_ppm
    }

    /// Set factory drift as a rational fraction: `live_factory = nominal ×
    /// (den + num) / den`. `(0, 1)` is exactly-nominal; `(1, 50)` is +2%;
    /// `(-1, 50)` is −2%. The denominator must be non-zero.
    pub fn set_factory_drift(&mut self, num: i32, den: u32) {
        assert!(
            den != 0,
            "HsiClock factory drift denominator must be non-zero"
        );
        self.factory_drift_num = num;
        self.factory_drift_den = den;
    }

    /// Effective HCLK after applying factory drift × applied trim.
    /// `live_hz = nominal × (den + num) × (1e6 + applied_ppm) / (den × 1e6)`.
    /// Sim has hardware division — clarity over micro-optimization.
    pub fn live(&self) -> Clock {
        let n = self.nominal.freq_hz() as i128;
        let num = self.factory_drift_num as i128;
        let den = self.factory_drift_den as i128;
        let ppm = self.applied_ppm as i128;
        let scaled = n * (den + num) * (1_000_000 + ppm) / (den * 1_000_000);
        Clock::new(scaled.clamp(0, u32::MAX as i128) as u32)
    }

    /// Pull the latest absolute correction from a `MockClockTrim`-style ops
    /// log. Idempotent on repeated calls with the same log; the cursor
    /// advances so subsequent calls only consume new entries.
    pub fn drain_ops(&mut self, ops: &[i32]) {
        if let Some(&latest) = ops[self.drained..].last() {
            self.applied_ppm = latest;
        }
        self.drained = ops.len();
    }

    /// Reset the ops cursor — used by `Servo::rebuild_uart`, which discards
    /// the prior `ClockTrimState` and starts fresh.
    pub fn reset_drain(&mut self) {
        self.drained = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_units_compose() {
        assert_eq!(SimTime::from_us(1).as_ns(), 1_000);
        assert_eq!(SimTime::from_ms(1).as_ns(), 1_000_000);
        assert_eq!(SimTime::from_ms(2) - SimTime::from_us(500), 1_500_000);
    }

    #[test]
    fn clock_at_48mhz_yields_48k_ticks_per_ms() {
        let c = Clock::new(48_000_000);
        assert_eq!(c.to_local(SimTime::from_ms(1)), 48_000);
        assert_eq!(c.from_local(48_000), SimTime::from_ms(1));
    }

    #[test]
    fn drifted_clock_produces_fewer_ticks_per_wall_ns() {
        let nominal = Clock::new(24_000_000);
        let slow = Clock::new(23_500_000);
        let t = SimTime::from_ms(1);
        assert_eq!(nominal.to_local(t), 24_000);
        assert_eq!(slow.to_local(t), 23_500);
    }

    #[test]
    fn hsi_clock_at_nominal_with_no_trim_matches_factory() {
        let h = HsiClock::new(Clock::new(48_000_000));
        assert_eq!(h.live().freq_hz(), 48_000_000);
    }

    #[test]
    fn hsi_clock_factory_drift_plus_two_percent() {
        let mut h = HsiClock::new(Clock::new(48_000_000));
        h.set_factory_drift(1, 50);
        assert_eq!(h.live().freq_hz(), 48_960_000);
    }

    #[test]
    fn hsi_clock_factory_drift_minus_two_percent() {
        let mut h = HsiClock::new(Clock::new(48_000_000));
        h.set_factory_drift(-1, 50);
        assert_eq!(h.live().freq_hz(), 47_040_000);
    }

    #[test]
    fn hsi_clock_applied_trim_cancels_factory_drift() {
        let mut h = HsiClock::new(Clock::new(48_000_000));
        h.set_factory_drift(1, 50);
        // +2% drift cancelled by -20_000 ppm correction → back to nominal.
        h.drain_ops(&[-20_000]);
        // Composed via multiplication, not addition, so the cancellation
        // isn't perfect: 48M × 1.02 × (1 - 0.02) = 48M × 0.9996 = 47.98M.
        // The drift is small enough that the convergence test reads "close
        // to nominal" rather than "exactly nominal" — see hsi_trim.rs.
        let live = h.live().freq_hz();
        assert!(
            (live as i64 - 48_000_000).abs() < 25_000,
            "expected ~48 MHz after cancellation, got {live}",
        );
    }

    #[test]
    fn hsi_clock_drain_ops_consumes_only_new_entries() {
        let mut h = HsiClock::new(Clock::new(48_000_000));
        h.drain_ops(&[-2500, -5000]);
        assert_eq!(h.applied_ppm(), -5000);
        h.drain_ops(&[-2500, -5000, -7500]);
        assert_eq!(h.applied_ppm(), -7500);
    }
}

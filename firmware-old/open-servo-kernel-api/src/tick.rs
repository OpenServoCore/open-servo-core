use crate::units::MicroSecond;

/// **Semantic tick domains**.
///
/// These are *conceptual* timing classes used by the kernel/nodes to organize
/// responsibilities and performance budgets.
///
/// **Important:** `TickDomain` does **not** imply a specific hardware timer.
/// The **board** decides how often each domain is driven.
///
/// Typical mental model (not a hard rule):
/// - `ControlFast`: effort/torque-ish + hard safety (e.g. 10kHz)
/// - `ControlMedium`: velocity-ish + persistence windows (e.g. 1kHz)
/// - `ControlSlow`: position-ish outer loop / profiles (e.g. 100–250Hz)
/// - `System`: housekeeping (thermal/logging/persistence) (e.g. 50–100Hz)
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TickDomain {
    /// Inner-loop / hard real-time control domain.
    ControlFast,
    /// Mid-rate control domain.
    ControlMedium,
    /// Outer-loop (cascaded) control domain.
    ControlSlow,
    /// Housekeeping / supervision domain.
    System,
}

/// A tick "frame" describing one occurrence of a domain.
///
/// - `domain` = which conceptual domain this tick belongs to
/// - `dt`     = delta time in microseconds since the last tick of this domain
/// - `seq`    = monotonic counter for this domain (wrapping OK, monotonic within domain)
///
/// The kernel uses this to keep behavior disciplined and portable across boards.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Tick {
    pub domain: TickDomain,
    pub dt: MicroSecond,
    pub seq: u32,
}

/// Sanity upper bound for dt: 1 second (1_000_000 µs).
///
/// If a tick has dt > 1s, something is seriously wrong (missed ticks, timer overflow, etc.).
const DT_UPPER_BOUND_US: u32 = 1_000_000;

impl Tick {
    /// Debug-only sanity checks.
    ///
    /// Asserts:
    /// - `dt > 0` (time must advance)
    /// - `dt < 1 second` (sanity bound; indicates missed ticks or misconfiguration)
    #[inline]
    pub fn debug_assert_valid(&self) {
        debug_assert!(self.dt > MicroSecond::ZERO, "Tick.dt must be > 0");
        debug_assert!(
            self.dt.0 < DT_UPPER_BOUND_US,
            "Tick.dt must be < 1 second (sanity bound); got {} µs",
            self.dt.0
        );
    }
}

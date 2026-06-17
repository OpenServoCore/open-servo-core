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
}

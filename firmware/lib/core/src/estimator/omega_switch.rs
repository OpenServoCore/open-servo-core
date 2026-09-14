//! Velocity-loop feedback source: the back-EMF boxcar while its windows
//! are clean, the pot observer's omega otherwise. A two-state switch with
//! run-length hysteresis - no blend, the loop sees one estimate or the
//! other.

/// Published in `TelemetryMode.omega_hat_src`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum OmegaSource {
    Pot = 0,
    Bemf = 1,
}

/// Consecutive valid boxcar results before the loop trusts the back-EMF:
/// 5 clean halves, 2.5 ms at 20 kHz, so a duty hovering at the sampling
/// floor cannot flip the source every half.
const TO_BEMF: u8 = 4;
/// Consecutive invalid results before falling back. One sub-floor FAST
/// tick voids both boxcars it sits in, so 2 is the shortest run a real
/// gap produces: fall back as soon as the data can show one - a stale
/// boxcar under the loop through a reversal is worse than pot noise.
const TO_POT: u8 = 2;

/// i16 c/s full scale, the observer's own clamp (fusion OMEGA_LIM).
const OMEGA_LIM_CPS: i32 = 32767;

pub struct OmegaSwitch {
    src: OmegaSource,
    run: u8,
    /// Last valid boxcar, csQ16; rides the loop through a short gap while
    /// the source is still Bemf.
    bemf_q16: i32,
}

impl Default for OmegaSwitch {
    fn default() -> Self {
        Self::new()
    }
}

impl OmegaSwitch {
    pub const fn new() -> Self {
        Self {
            src: OmegaSource::Pot,
            run: 0,
            bemf_q16: 0,
        }
    }

    /// One MEDIUM-tick update -> omega_hat csQ16. `bemf_cps` is the boxcar
    /// result (None = a sub-floor window inside it), `pot_q16` the
    /// observer's omega.
    pub fn step(&mut self, bemf_cps: Option<i32>, pot_q16: i32) -> i32 {
        match (self.src, bemf_cps) {
            (OmegaSource::Pot, Some(w)) => {
                self.bemf_q16 = w.clamp(-OMEGA_LIM_CPS, OMEGA_LIM_CPS) << 16;
                self.run += 1;
                if self.run >= TO_BEMF {
                    self.src = OmegaSource::Bemf;
                    self.run = 0;
                }
            }
            (OmegaSource::Bemf, Some(w)) => {
                self.bemf_q16 = w.clamp(-OMEGA_LIM_CPS, OMEGA_LIM_CPS) << 16;
                self.run = 0;
            }
            (OmegaSource::Bemf, None) => {
                self.run += 1;
                if self.run >= TO_POT {
                    self.src = OmegaSource::Pot;
                    self.run = 0;
                }
            }
            (OmegaSource::Pot, None) => self.run = 0,
        }
        match self.src {
            OmegaSource::Bemf => self.bemf_q16,
            OmegaSource::Pot => pot_q16,
        }
    }

    pub fn source(&self) -> OmegaSource {
        self.src
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const POT: i32 = 1234 << 16;

    #[test]
    fn boots_on_the_pot() {
        let mut sw = OmegaSwitch::new();
        assert_eq!(sw.step(None, POT), POT);
        assert_eq!(sw.source(), OmegaSource::Pot);
    }

    #[test]
    fn switches_after_to_bemf_consecutive_valid() {
        let mut sw = OmegaSwitch::new();
        for _ in 0..TO_BEMF - 1 {
            assert_eq!(sw.step(Some(500), POT), POT);
            assert_eq!(sw.source(), OmegaSource::Pot);
        }
        assert_eq!(sw.step(Some(500), POT), 500 << 16);
        assert_eq!(sw.source(), OmegaSource::Bemf);
        // tracks every valid result once switched
        assert_eq!(sw.step(Some(-700), POT), -700 << 16);
    }

    #[test]
    fn one_invalid_restarts_the_valid_run() {
        let mut sw = OmegaSwitch::new();
        for _ in 0..TO_BEMF - 1 {
            sw.step(Some(500), POT);
        }
        sw.step(None, POT);
        for _ in 0..TO_BEMF - 1 {
            assert_eq!(sw.step(Some(500), POT), POT);
        }
        assert_eq!(sw.step(Some(500), POT), 500 << 16);
    }

    #[test]
    fn falls_back_after_to_pot_consecutive_invalid() {
        let mut sw = OmegaSwitch::new();
        for _ in 0..TO_BEMF {
            sw.step(Some(500), POT);
        }
        assert_eq!(sw.source(), OmegaSource::Bemf);
        // rides the last boxcar for the first invalid result
        for _ in 0..TO_POT - 1 {
            assert_eq!(sw.step(None, POT), 500 << 16);
            assert_eq!(sw.source(), OmegaSource::Bemf);
        }
        assert_eq!(sw.step(None, POT), POT);
        assert_eq!(sw.source(), OmegaSource::Pot);
        // and needs the full clean run to come back
        for _ in 0..TO_BEMF - 1 {
            assert_eq!(sw.step(Some(600), POT), POT);
        }
        assert_eq!(sw.step(Some(600), POT), 600 << 16);
    }

    #[test]
    fn a_valid_result_resets_the_invalid_run() {
        let mut sw = OmegaSwitch::new();
        for _ in 0..TO_BEMF {
            sw.step(Some(500), POT);
        }
        for _ in 0..8 {
            sw.step(None, POT);
            assert_eq!(sw.step(Some(500), POT), 500 << 16);
            assert_eq!(sw.source(), OmegaSource::Bemf);
        }
    }

    #[test]
    fn bemf_clamps_to_i16_full_scale() {
        let mut sw = OmegaSwitch::new();
        for _ in 0..TO_BEMF {
            sw.step(Some(i32::MAX), POT);
        }
        assert_eq!(sw.step(Some(i32::MAX), POT), 32767 << 16);
        assert_eq!(sw.step(Some(i32::MIN), POT), -32767 << 16);
    }
}

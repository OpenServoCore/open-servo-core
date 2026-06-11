//! Vcal reference low-pass filter. EWMA, α = 1/128. `state_q6` keeps 6
//! sub-LSB bits so the filter resolves drift slower than 1 LSB per step
//! without integer-rounding biasing the steady state.

pub(super) struct VcalLpf {
    state_q6: i32,
    initialized: bool,
}

impl VcalLpf {
    pub(super) const fn new() -> Self {
        Self {
            state_q6: 0,
            initialized: false,
        }
    }

    pub(super) fn update(&mut self, raw: u16) -> u16 {
        let x = (raw as i32) << 6;
        if !self.initialized {
            self.state_q6 = x;
            self.initialized = true;
        } else {
            self.state_q6 += (x - self.state_q6) >> 7;
        }
        (self.state_q6 >> 6) as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initialises_on_first_sample() {
        let mut lpf = VcalLpf::new();
        assert_eq!(lpf.update(1234), 1234);
    }

    #[test]
    fn constant_input_steady_state() {
        let mut lpf = VcalLpf::new();
        lpf.update(500);
        for _ in 0..256 {
            assert_eq!(lpf.update(500), 500);
        }
    }

    #[test]
    fn step_converges() {
        let mut lpf = VcalLpf::new();
        lpf.update(0);
        let mut last = 0;
        for _ in 0..2000 {
            last = lpf.update(1000) as i32;
        }
        assert!((last - 1000).abs() <= 2, "last={last}");
    }
}

use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

#[derive(Copy, Clone, Debug)]
pub struct SampleFrame {
    pub tick: u32,
    pub pos: Microrads,
    pub current: Milliamps,
    pub temp: CentiCelsius,
    pub vbus: Millivolts,
    /// |V_A − V_B| during the ON window of the PWM period.
    pub vmotor: Millivolts,
    pub raw: RawSamples,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct RawSamples {
    pub pos: u16,
    /// Averaged shunt count.
    pub current: u16,
    pub temp: u16,
    pub vbus: u16,
    pub vmotor_a: u16,
    pub vmotor_b: u16,
}

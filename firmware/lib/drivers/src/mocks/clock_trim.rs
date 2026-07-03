use mockall::mock;

use crate::traits::dxl::ClockTrim;

mock! {
    pub ClockTrim {}
    impl ClockTrim for ClockTrim {
        // Same envelope + step as the production V006 binding so existing
        // drift/threshold tests stay numerically aligned without baking in
        // chip-specific imports. 60 kHz / 24 MHz HSI = 2500 ppm/step;
        // envelope = -16/+15 trim register units × 2500 ppm/step.
        const STEP_PPM: u32 = 2500;
        const ENVELOPE_PPM: (i32, i32) = (-40_000, 37_500);

        fn apply_ppm(&mut self, ppm: i32);
    }
}

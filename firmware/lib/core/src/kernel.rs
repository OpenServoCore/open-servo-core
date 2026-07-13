use crate::traits::ControlIo;
use crate::{Sample, Shared};

/// Runs in the PWM ISR; one `on_tick` per period.
pub struct Kernel<I: ControlIo> {
    pub io: I,
    pub state: KernelState,
    pub stream_decimation_counter: u8,
    pub stream_duration_remaining_ticks: u32,
}

#[derive(Default)]
pub struct KernelState {
    pub pid_integrator: i32,
    pub pid_prev_error: i32,
}

impl<I: ControlIo> Kernel<I> {
    pub fn new(io: I) -> Self {
        Self {
            io,
            state: KernelState::default(),
            stream_decimation_counter: 1,
            stream_duration_remaining_ticks: 0,
        }
    }

    /// Must complete well inside the kernel period (~50 us at 20 kHz).
    pub fn on_tick(&mut self, _sample: Sample, _shared: &Shared) {
        // TODO: PID + mode dispatch + motor.write once the control loop lands.
    }
}

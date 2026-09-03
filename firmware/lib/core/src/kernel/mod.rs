pub mod current;
pub mod limits;
pub mod position;
pub mod trajectory;
pub mod velocity;

pub use current::{CurrentGains, CurrentLoop};
pub use limits::{LimitCfg, LimitState};
pub use position::{PosOut, PositionCfg};
pub use trajectory::{TrajCfg, TrajGen};
pub use velocity::{VelocityGains, VelocityLoop};

use crate::estimator::VcalLpf;
use crate::traits::ControlIo;
use crate::{RegionStorageRaw, SensorFrame, Shared};

/// Runs in the PWM ISR; one `on_tick` per period.
pub struct Kernel<I: ControlIo> {
    pub io: I,
    pub state: KernelState,
    pub vcal_lpf: VcalLpf,
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
            vcal_lpf: VcalLpf::new(),
        }
    }

    /// Must complete well inside the kernel period (~50 us at 20 kHz).
    pub fn on_tick(&mut self, frame: SensorFrame, shared: &Shared) {
        let vcal_lpf = self.vcal_lpf.update(frame.vcal);

        // SAFETY: ISR context is the region's sole writer (the `sample_tick`
        // contract); volatile per field so the stores survive optimization.
        unsafe {
            let s = &raw mut (*shared.table.region_ptr()).telemetry.sensors;
            (&raw mut (*s).pos).write_volatile(frame.pos);
            (&raw mut (*s).current).write_volatile(frame.current);
            (&raw mut (*s).vcal).write_volatile(frame.vcal);
            (&raw mut (*s).vcal_lpf).write_volatile(vcal_lpf);
            (&raw mut (*s).vmotor_a).write_volatile(frame.vmotor_a);
            (&raw mut (*s).vmotor_b).write_volatile(frame.vmotor_b);
            (&raw mut (*s).current_trough).write_volatile(frame.current_trough);
        }
        // TODO: PID + mode dispatch + motor.write once the control loop lands.
    }
}

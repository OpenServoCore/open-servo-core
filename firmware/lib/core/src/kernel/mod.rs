//! The assembled kernel (spec "on_tick skeleton"): one `on_tick` per PWM
//! period carries all three rates - FAST every tick (`fast`: sensor
//! publish, window select, OC trip, back-EMF sums, TEL, ident, current PI,
//! motor write), MEDIUM once per DECIM_MED-tick period, spread over its
//! ticks one phase each (`medium`, `phase`: CONTROL and the edges, fusion,
//! trajectory and position, limits, velocity, vbus and detectors, slow,
//! publishes), SLOW every DECIM_SLOW periods inside `medium` (thermometer,
//! derate, undervolt/overtemp). The medium half leaves the fast half a
//! `fast::Command`; the fast measurement reaches the medium phase of the
//! same tick as `fast::Measured`. Only the CONTROL phase reads CONTROL, so
//! a host write (torque, mode, goals) takes effect at the next period's
//! CONTROL phase. Identification aggregates ride their own
//! /16 fast-tick window (`ident`), independent of DECIM_MED, and fold only
//! while CONTROL `ident_agg` asks for them. Tick-indexed
//! by design: a missed tick dilates time, nothing compensates and nothing
//! reads a wall clock. CONFIG and CALIB reach the tick through the kernel's
//! own snapshot (`config`), rebuilt at a medium boundary after a write, so
//! a configuration write takes effect within one medium tick.

mod config;
pub mod current;
pub mod duty_limit;
mod fast;
pub mod faults;
pub mod ident;
pub mod limits;
mod medium;
pub mod position;
pub mod trajectory;
pub mod velocity;

pub use current::{CurrentGains, CurrentLoop};
pub use limits::{IBand, LimitCfg, LimitState};
pub use medium::phase;
pub use position::{PosOut, PositionCfg};
pub use trajectory::{TrajCfg, TrajGen};
pub use velocity::{VelocityGains, VelocityLoop};

use core::sync::atomic::{Ordering, compiler_fence};

use self::config::KernelConfig;
use self::fast::{Command, Fast};
use self::medium::{Control, Medium};
use crate::tel::TelStream;
use crate::traits::{ControlIo, Motor};
use crate::{RegionStorageRaw, SensorFrame, Shared};

/// FAST -> MEDIUM decimation: MED_HZ = tick_hz / DECIM_MED (2 kHz at 20 kHz).
pub const DECIM_MED: u8 = 10;
/// MEDIUM -> SLOW decimation: SLOW_HZ = MED_HZ / DECIM_SLOW (62.5 Hz).
pub const DECIM_SLOW: u8 = 32;
/// Stall permit lease in SLOW ticks: 63 x 16 ms = 1.008 s from the grant.
pub const PERMIT_LEASE_TICKS: u8 = 63;

/// Finished constants the chip const-evals from `MOTOR_PWM_FREQ_HZ`, the
/// TIM1 ARR and the board's dividers, so core never divides at runtime or
/// install - every field is a compile-time quotient on the chip side
/// (`Precomputed`).
#[derive(Copy, Clone)]
pub struct KernelTiming {
    /// The TIM1 auto-reload the motor write programs; drive-window widths
    /// derive from it (`window::drive_ticks`).
    pub pwm_arr: u16,
    /// `(1 << bemf::RECIP_ARR_SHIFT) / pwm_arr`: duty-fraction reciprocal
    /// for the shared `v_mean` computation.
    pub recip_arr_q24: u32,
    /// FAST tick rate = `MOTOR_PWM_FREQ_HZ`, the same constant stamped into
    /// `CalibSense.tick_hz` at install; carried as the derivation anchor for
    /// the fields below.
    pub tick_hz: u16,
    /// `2^32 / MED_HZ` where `MED_HZ = tick_hz / DECIM_MED`: the medium
    /// integration step, `theta += q_mul(omega, dt_med_q32, 32)`.
    pub dt_med_q32: u32,
    /// `(MED_HZ << 16) / 1000`: ms -> medium ticks via `q_mul_u(ms, ., 16)`.
    pub med_ticks_per_ms_q16: u32,
    /// Rail-tap -> vmotor-tap counts, Q15 (`VbusEst::new`).
    pub vbus_scale_q15: u32,
}

/// Runs in the ADC DMA TC ISR (PFIC LOW); one `on_tick` per PWM period.
/// Single-writer contracts: the transport (PFIC HIGH) owns every
/// CONTROL/CONFIG/CALIB write and the position table array, and can preempt
/// this ISR mid-read, so the kernel only ever reads those - volatile via
/// `region_ptr` (`Shared::pos_lut_q4` for the array), never forming `&T`,
/// cross-field tearing accepted (each field is independently sane). CONTROL
/// is read once per period; CONFIG and CALIB only into `cfg`, when
/// `Shared::config_gen` moved. The kernel is the sole writer of TELEMETRY
/// sensors/estimates/mode/limits (`data_flags` excepted: boot and dispatch
/// write it, the kernel reads) and the `fault_flags` byte; TELEMETRY
/// `health` belongs to the chip side.
pub struct Kernel<I: ControlIo, T: TelStream = ()> {
    pub io: I,
    tel: T,
    timing: KernelTiming,
    cfg: KernelConfig,
    /// The `Shared::config_gen` value `cfg` was built at.
    config_gen: u8,
    /// The medium phase the next tick runs (`medium::phase`).
    phase: u8,
    booted: bool,
    /// Shared by both halves: a raise on either disables the same tick's
    /// drive.
    faults: faults::FaultLatch,
    fast: Fast,
    medium: Medium,
    /// Medium -> fast hand-off.
    cmd: Command,
}

impl<I: ControlIo> Kernel<I> {
    pub fn new(io: I, timing: KernelTiming) -> Self {
        Self::with_tel(io, (), timing)
    }
}

impl<I: ControlIo, T: TelStream> Kernel<I, T> {
    pub fn with_tel(io: I, tel: T, timing: KernelTiming) -> Self {
        Self {
            io,
            tel,
            timing,
            cfg: KernelConfig::default(),
            config_gen: 0,
            // the FIRST tick runs the CONTROL phase: the configuration and
            // the seeds exist before any other phase sees them
            phase: medium::phase::CONTROL,
            booted: false,
            faults: faults::FaultLatch::new(),
            fast: Fast::new(timing.pwm_arr),
            medium: Medium::new(&timing),
            cmd: Command::default(),
        }
    }

    /// Rebuild `cfg` from the table. `config_gen` is the generation read
    /// before the copy: a write landing during it leaves the counter ahead
    /// of what `cfg` records, so the next medium boundary rebuilds again.
    #[inline(never)]
    fn refresh(&mut self, shared: &Shared, config_gen: u8) {
        // the generation load stays ahead of the block copies
        compiler_fence(Ordering::Acquire);
        let cfg = KernelConfig::load(shared, &self.timing);
        // thermometer seed tracks the calib anchor: install writes and host
        // rewrites both land here
        let r0 = cfg.medium.therm_anchor.r0_q12;
        if r0 != self.cfg.medium.therm_anchor.r0_q12 {
            self.medium.seed_thermal(r0);
        }
        self.cfg = cfg;
        self.config_gen = config_gen;
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (type doc); volatile store.
        unsafe {
            (&raw mut (*p).telemetry.limits.window_floor_q15).write_volatile(cfg.fast.ol_floor_q15);
        }
    }

    /// First tick: the configuration, then the observer and the bias
    /// tracker seeded at the boot measurement.
    fn boot(&mut self, frame: &SensorFrame, shared: &Shared, config_gen: u8) {
        self.refresh(shared, config_gen);
        let ctl = Control::read(shared);
        self.medium
            .seed(medium::pos_q4(shared, ctl.lut_live, frame.pos));
        // the stream's first row linearizes like the rest, and the
        // aggregate's first window opens at the first tick
        self.cmd.lut_live = ctl.lut_live;
        self.cmd.ident_agg = ctl.ident_agg;
        let p = shared.table.region_ptr();
        // SAFETY: same volatile read contract; install stamped the boot rest
        // measurement here before the first tick, and from here on this
        // kernel is the field's sole writer.
        self.fast.seed_bias(unsafe {
            (&raw const (*p).telemetry.sensors.current_bias_counts).read_volatile()
        });
        self.booted = true;
    }

    /// Must complete well inside the kernel period (~50 us at 20 kHz).
    pub fn on_tick(&mut self, frame: SensorFrame, shared: &Shared) {
        let phase = self.phase;
        self.phase = if phase + 1 < DECIM_MED { phase + 1 } else { 0 };
        // the configuration the CONTROL phase's tick measures and drives with
        if phase == medium::phase::CONTROL {
            let config_gen = shared.config_gen();
            if !self.booted {
                self.boot(&frame, shared, config_gen);
            } else if config_gen != self.config_gen {
                self.refresh(shared, config_gen);
            }
        }
        let meas = self.fast.measure(
            &frame,
            &self.cfg.fast,
            &self.cmd,
            &mut self.faults,
            &mut self.tel,
            shared,
        );
        self.medium.step(
            phase,
            &frame,
            &meas,
            &self.cfg,
            shared,
            &mut self.faults,
            &mut self.fast,
            &mut self.cmd,
        );
        let out = self
            .fast
            .drive(&meas, &self.cfg.fast, &self.cmd, &self.faults);
        let (_sensors, motor) = self.io.parts();
        motor.write(out);
    }
}

#[cfg(test)]
mod tests;

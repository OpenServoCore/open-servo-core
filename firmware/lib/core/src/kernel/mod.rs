//! The assembled kernel (spec "on_tick skeleton"): one `on_tick` per PWM
//! period carries all three rates - FAST every tick (`fast`: sensor
//! publish, window select, OC trip, back-EMF sums, TEL, current PI,
//! motor write), MEDIUM once per DECIM_MED-tick period, spread over its
//! ticks one phase each (`medium`, `phase`: CONTROL and the edges, fusion,
//! trajectory and position, limits, velocity, vbus and detectors, slow,
//! publishes), SLOW every DECIM_SLOW periods inside `medium` in two stages
//! a period apart (undervolt and the thermometer's step on the due tick,
//! its temperature and overtemp on the next period's; the thermometer's
//! NTC on the last spare phase of the period before, the derate and the
//! install of a reference it froze on the spare phases of the period after
//! the second stage). The medium half leaves the fast half a
//! `fast::Command`; the fast measurement reaches the medium phase of the
//! same tick as `fast::Measured`. Only the CONTROL phase reads CONTROL, so
//! a host write (torque, mode, goals) takes effect at the next period's
//! CONTROL phase. The tick index is the sample clock: every executed tick is
//! one period, and nothing reads a wall clock. CONFIG and CALIB reach the
//! tick through the kernel's own copy (`config`), taken whole at
//! SPARE_A after a commit and never under one, so a configuration
//! write takes effect within a medium period, or one more when the tick
//! lands inside the next commit.

mod config;
pub mod current;
pub mod duty_limit;
mod fast;
pub mod faults;
pub mod limits;
mod medium;
pub mod position;
mod scheduler;
pub mod trajectory;
pub mod velocity;

pub use config::{KernelConfig, SPAN_END, SPAN_START};
pub use current::CurrentLoop;
pub use limits::{IBand, LimitState};
pub use position::PosOut;
pub use scheduler::table::{
    DECIM_MED, DECIM_SLOW, HEALTH_CLOSE_PHASE, HEALTH_CLOSE_TICKS, TICK_HZ, health_close_at,
    health_close_lands, phase,
};
pub use trajectory::TrajGen;
pub use velocity::VelocityLoop;

use core::sync::atomic::{Ordering, compiler_fence};

use self::fast::{Command, Fast};
use self::medium::{Control, Medium};
use crate::estimator::window;
use crate::tel::TelStream;
use crate::traits::{ControlIo, Motor};
use crate::{RegionStorageRaw, SensorFrame, Shared};

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
    /// integration step.
    pub dt_med_q32: u32,
    /// Rail-tap -> vmotor-tap counts, Q15 (`VbusEst::new`).
    pub vbus_scale_q15: u32,
    /// Board data: the shortest Slow-decay brake half whose trough shunt
    /// sample feeds the bias tracker (`window::trough_is_brake`).
    pub bias_brake_min_ticks: u16,
    /// Board data: the shortest Fast-decay window whose trough scan samples
    /// both terminal taps inside it. It raises CALIB `v_window_min_ticks` for
    /// Fast decay only (`window::v_floors`).
    pub v_trough_min_ticks: u16,
    /// Board data: the shortest window whose terminal differential the
    /// back-EMF observer takes. Under it the drive pulse comes out short of
    /// the commanded width by enough to bias the period-average voltage the
    /// observer scales by that width.
    pub bemf_min_ticks: u16,
    /// Board data: the shunt amplifier's settle gain by drive width
    /// (`window::i_from_frame`).
    pub i_settle_gain: window::SettleGain,
}

/// What the budget probe reads around a tick: before it, the medium phase
/// it runs, whether the boot tick has run and which SLOW stage it runs; on
/// either side, the count the configuration was taken at (a take moves it),
/// the thermometer's pending stages and whether the drive arm is Closed.
#[derive(Copy, Clone)]
pub struct ProbeTags {
    pub phase: u8,
    pub config_seq: u8,
    pub booted: bool,
    pub slow_due: bool,
    pub slow_finish: bool,
    /// `thermal::stage` bits.
    pub therm: u8,
    pub closed: bool,
}

/// Runs in the ADC DMA TC ISR (PFIC HIGH, above the bus); one `on_tick` per
/// PWM period. Single-writer contracts: the transport owns every
/// CONTROL/CONFIG/CALIB write and the position table array, and this ISR
/// can preempt it mid-commit, so the kernel only ever reads those - volatile via
/// `region_ptr` (`Shared::pos_lut_q4` for the array), never forming `&T`,
/// cross-field tearing accepted (each field is independently sane). CONTROL
/// is read once per period; CONFIG and CALIB only into `cfg`, between
/// commits (`Shared::config_seq`). The kernel is the sole writer of TELEMETRY
/// tick (its frame and `sample_tick` excepted: the ADC DMA and the tick ISR
/// land them ahead of the kernel), estimates, refs, mode and limits (`data_flags`
/// excepted: boot and dispatch write it, the kernel reads) and the
/// `fault_flags` byte; TELEMETRY-EXT `health` belongs to the chip side.
pub struct Kernel<I: ControlIo, T: TelStream = ()> {
    pub io: I,
    tel: T,
    timing: KernelTiming,
    cfg: KernelConfig,
    /// The `Shared::config_seq` value `cfg` was taken at.
    config_seq: u8,
    /// The medium phase the next tick runs (`phase`).
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
            cfg: KernelConfig::new(),
            config_seq: 0,
            // the FIRST tick runs the CONTROL phase: the configuration and
            // the seeds exist before any other phase sees them
            phase: phase::CONTROL,
            booted: false,
            faults: faults::FaultLatch::new(),
            fast: Fast::new(&timing),
            medium: Medium::new(&timing),
            cmd: Command::default(),
        }
    }

    /// The configuration, whole, then the thermometer's bound at it and the
    /// terminal floor at the OpenLoop decay. The rail clamp follows
    /// `duty_max_q15` at the next RAIL phase. The kernel preempts the bus, so
    /// no commit store lands inside the tick: at an even count the span is
    /// one committed set. At an odd one (the first tick only) the next even
    /// count differs from it and takes again.
    #[inline(never)]
    fn take_config(&mut self, shared: &Shared) {
        self.config_seq = shared.config_seq();
        // the count load stays ahead of the field reads
        compiler_fence(Ordering::Acquire);
        self.cfg.take(shared);
        self.medium.thermal.bound(&self.cfg.therm);
        let v_floor = self.cfg.v_floor_q15[self.cfg.limits.openloop_decay as usize];
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (type doc); volatile store.
        unsafe {
            (&raw mut (*p).telemetry_ext.limits_ext.window_v_floor_q15).write_volatile(v_floor);
        }
    }

    /// The first tick's configuration: the window floors, the take and the
    /// rail clamp. Out of line, so the tick body keeps its shape.
    #[inline(never)]
    fn configure(&mut self, shared: &Shared) {
        self.cfg.load_floors(shared, &self.timing);
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (type doc); volatile store.
        unsafe {
            (&raw mut (*p).telemetry.limits.window_floor_q15).write_volatile(self.cfg.ol_floor_q15);
        }
        self.take_config(shared);
        self.cmd.prepare_rail(&self.cfg);
    }

    /// First tick: the configuration, then the observer and the bias
    /// tracker seeded at the boot measurement.
    fn boot(&mut self, shared: &Shared) {
        let frame = shared.frame();
        self.configure(shared);
        let ctl = Control::read(shared);
        self.medium.seed(
            medium::pos_q4(shared, ctl.lut_live, frame.pos()),
            frame.ntc_raw(),
            &self.cfg,
        );
        let p = shared.table.region_ptr();
        // SAFETY: same volatile read contract; install stamped the boot rest
        // measurement here before the first tick, and from here on this
        // kernel is the field's sole writer.
        self.fast.seed_bias(unsafe {
            (&raw const (*p).telemetry.refs.current_bias_counts).read_volatile()
        });
        self.booted = true;
    }

    /// The logical duty the last tick commanded: the motor write carries
    /// only its width and the wiring's sign.
    pub fn duty_q15(&self) -> i16 {
        self.fast.duty_q15()
    }

    /// The kernel as the budget probe tags the tick around it.
    pub fn probe_tags(&self) -> ProbeTags {
        ProbeTags {
            phase: self.phase,
            config_seq: self.config_seq,
            booted: self.booted,
            slow_due: self.phase == phase::SLOW && self.medium.slow_periods + 1 >= DECIM_SLOW,
            slow_finish: self.phase == phase::SLOW && self.medium.slow_periods == 0,
            therm: self.medium.thermal.pending(),
            closed: self.faults.mask() == 0 && matches!(self.cmd.drive, fast::Drive::Closed { .. }),
        }
    }

    /// [`Self::on_tick`] on `frame` landed in the table's tick block first,
    /// as the chip lands each scan.
    pub fn on_scan(&mut self, frame: SensorFrame, shared: &Shared) {
        // SAFETY: the tick's context is the block's frame writer, and
        // nothing reads it meanwhile but this context.
        unsafe { frame.land(shared.tick_block()) };
        self.on_tick(shared);
    }

    /// Must complete well inside the kernel period (~50 us at 20 kHz). The
    /// frame is the table's tick block's (`Shared::frame`), which the TEL
    /// row copies from too.
    pub fn on_tick(&mut self, shared: &Shared) {
        let phase = self.phase;
        self.phase = if phase + 1 < DECIM_MED { phase + 1 } else { 0 };
        // Arm order is the spare phases' priority: a configuration take owns
        // its tick, and the thermometer's stage waits a period on its pending
        // bit, as it does through the SLOW finish's period: one slow-rate
        // stage per period keeps them from bunching on one TEL batch. A stage
        // tick holds the TX lane; a take does not, as one can run every
        // period.
        match phase {
            phase::CONTROL if !self.booted => self.boot(shared),
            phase::SPARE_A if config_due(shared.config_seq(), self.config_seq) => {
                self.take_config(shared)
            }
            phase::SPARE_A | phase::SPARE_B
                if self.medium.thermal.pending() != 0 && self.medium.slow_periods != 1 =>
            {
                let ran = self
                    .medium
                    .spare(phase, shared.frame().ntc_raw(), &self.cfg);
                if ran {
                    self.tel.hold();
                }
            }
            _ => {}
        }
        let meas = self.fast.measure(&self.cfg, &mut self.faults, shared);
        // No burst starts on the take's phase, nor while a take is pending.
        // Asked only at a start: a count read every tick costs every tick.
        self.tel
            .on_tick(|| phase == phase::SPARE_A || shared.config_seq() != self.config_seq);
        let staged = self.medium.step(
            phase,
            &meas,
            &self.cfg,
            shared,
            &mut self.faults,
            &mut self.fast,
            &mut self.cmd,
        );
        if staged {
            self.tel.hold();
        }
        let out = self.fast.drive(&meas, &self.cfg, &self.cmd, &self.faults);
        let (_sensors, motor) = self.io.parts();
        motor.write(out);
        self.tel.on_tick_end();
    }
}

/// A count the kernel has not taken, and even: no commit under way. On odd
/// the kernel keeps its set and looks again at the next period's
/// SPARE_A; it never waits on the bus.
fn config_due(seq: u8, taken: u8) -> bool {
    seq & 1 == 0 && seq != taken
}

#[cfg(test)]
mod tests;

//! The tick: one `on_tick` per PWM period, the spare phases' arbitration
//! between a configuration take and the thermometer's stages, the lane
//! holds, and the hand-off between the halves.

mod probe;
pub mod table;

pub use probe::ProbeTags;

use core::sync::atomic::{Ordering, compiler_fence};

use self::table::{DECIM_MED, phase};
use super::KernelConfig;
use super::phases::fast::{Command, Fast};
use super::phases::loops::faults::FaultLatch;
use super::phases::{self, Control, Medium};
use crate::estimator::window;
use crate::traits::{ControlIo, Motor, TelStream};
use crate::{RegionStorageRaw, SensorFrame, Shared};

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
    pub(in crate::kernel) tel: T,
    timing: KernelTiming,
    pub(in crate::kernel) cfg: KernelConfig,
    /// The `Shared::config_seq` value `cfg` was taken at.
    pub(in crate::kernel) config_seq: u8,
    /// The medium phase the next tick runs (`phase`).
    pub(in crate::kernel) phase: u8,
    booted: bool,
    /// Shared by both halves: a raise on either disables the same tick's
    /// drive.
    pub(in crate::kernel) faults: FaultLatch,
    pub(in crate::kernel) fast: Fast,
    pub(in crate::kernel) medium: Medium,
    /// Medium -> fast hand-off.
    pub(in crate::kernel) cmd: Command,
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
            faults: FaultLatch::new(),
            fast: Fast::new(&timing),
            medium: Medium::new(&timing),
            cmd: Command::default(),
        }
    }

    /// The configuration, whole, then the thermometer's bound at it and the
    /// terminal floor at the OpenLoop decay; a new `drive_polarity`
    /// unproves the drive direction check. The rail clamp follows
    /// `duty_max_q15` at the next RAIL phase. The kernel preempts the bus, so
    /// no commit store lands inside the tick: at an even count the span is
    /// one committed set. At an odd one (the first tick only) the next even
    /// count differs from it and takes again.
    #[inline(never)]
    #[rustc_align(4)]
    fn take_config(&mut self, shared: &Shared) {
        self.config_seq = shared.config_seq();
        // the count load stays ahead of the field reads
        compiler_fence(Ordering::Acquire);
        self.cfg.take(shared);
        self.medium.drive.take(self.cfg.limits.drive_polarity);
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
    #[rustc_align(4)]
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
            phases::pos_q4(shared, ctl.lut_live, frame.pos()),
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
        // period. The drive direction check takes a SPARE_B no thermometer
        // stage does, and its own stage runs there too, so the two never
        // share a tick; it holds no lane.
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
            phase::SPARE_B if self.medium.drive.unproven() => {
                let duty = self.fast.duty_q15();
                if self.medium.drive.check(self.medium.theta_q4(), duty) {
                    core::hint::cold_path();
                    self.medium.drive_stage(shared, duty, &mut self.faults);
                }
            }
            _ => {}
        }
        let meas = self.fast.measure(&self.cfg, &mut self.faults, shared);
        // No burst starts on the take's phase, nor while a take is pending.
        // Asked only at a start: a count read every tick costs every tick.
        self.tel
            .on_publish(|| phase == phase::SPARE_A || shared.config_seq() != self.config_seq);
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

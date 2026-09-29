//! The test servo + driver pump, for this crate's tests and, behind the
//! `testkit` feature, the host tools': a scripted plant that answers the
//! engine's commands the way the rig would, so experiments run end to end
//! in-process. Electrical model: stalled i = v/R; free-running i = fc; the
//! first windows after a duty change are inflated to imitate the L
//! transient the settle discard exists for.
//!
//! With `current_limit` set the fake governs OpenLoop duty the way the
//! kernel's limiter does: a goal above the window floor slews up from the
//! floor in steps of the ceiling, the ceiling stops rising once the current
//! reaches the band `[lim - lim/8, lim]` and drops when it is over, and it
//! never applies less than the base duty, so a reversal at speed draws
//! (base x rail + back-EMF) / R. A moving shaft holds the band's bottom
//! (back-EMF keeps pulling the current under it), a still one the step
//! nearest its top. A goal under the floor passes raw up to the stall-safe
//! duty, the shunt blind to it - its ident window is invalid and the
//! aggregate holds the last valid current, so a token brake at speed reads
//! nothing. With `lease_ms` set the stall permit is a lease: granted by a
//! write of true with torque on, extended by a rewrite, dropped by torque
//! off, false, or the lease running out. With `stall_ms` set the stall
//! timer runs too: a limiter that holds the goal back on a still shaft that
//! long folds the limit to `stall_yield`, and the fold releases once the
//! disturbance estimate falls under `stall_release`; one that would release
//! at once shows the verdict for a single MEDIUM tick.
//!
//! [`pump`] charges every transaction the time the bench bus takes
//! ([`Bus::BENCH`]) and the plant moves through it. The kernel loses
//! [`TICKS_LOST_PER_TRANSACTION`] to each one, as the bench board does, so
//! `sample_tick` and `agg_seq` run slow under polling while `host_ms` and
//! a TEL stream's ticks keep real time.

use super::{Cmd, Experiment, RigParams, SLEW_Q15_PER_TICK, TICKS_PER_WINDOW};
use crate::burst::{
    ArmSeen, CHAN_VBUS, CHAN_VMOTOR_A, CHAN_VMOTOR_B, Capture, Meta, SAMPLE_HCLK, SAMPLE_US,
    SAMPLES, frame_len, rejected,
};
use crate::frame::{TelFrame, TelemetrySnapshot};
use crate::limits::PermitLease;
use crate::lut::GridLut;
use crate::regs::{ALL, Reg, control};

/// The fake's fast tick rate, Hz: ten per MEDIUM tick at its `f_med`.
pub const TICK_HZ: f64 = 20_100.0;

/// The envelope the tests run in: a guard inside the fake's stops (200,
/// 4000), those stops as calibrated, and an abort over any current the
/// fake draws unless a test asks.
pub fn rig() -> RigParams {
    RigParams::new(Some((150, 3950)), 1100).with_stops((200, 4000))
}

macro_rules! mg90_2s {
    ($($n:literal),*) => {
        [$(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/burst/mg90-2s/burst-",
            $n,
            ".csv"
        ))),*]
    };
}

/// A burst run on the bench MG90 on 2S, the from-rest captures in run
/// order: 25% forward, 25% reverse, 40% forward, 40% reverse, four each,
/// the driven terminal sampled behind the shunt. Capture 11 reads its
/// winding low.
pub fn mg90_2s() -> Vec<Capture> {
    mg90_2s!(
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"
    )
    .iter()
    .map(|t| crate::burst::from_csv(t).expect("fixture"))
    .collect()
}

/// Board D as fitted: 60 mohm shunt at G 15, 6k8/3k3 terminal taps, the
/// 15k/10k rail tap.
pub fn board_d_scales() -> crate::exp::rl::Scales {
    let s = crate::units::SenseParams {
        shunt_r_mohm: 60,
        gain_milli: 15_000,
        vmotor_div_top: 6_800,
        vmotor_div_bot: 3_300,
        vdd_mv: 3_300,
        tick_hz: 20_100,
    };
    crate::exp::rl::Scales::from_sense(&s, 15_000, 10_000).expect("board D scales")
}

/// The bench MG90 as the fake models it on a rail of `vbus`: 4.9 ohm, the
/// firmware limiter at 280 counts above a 13.3% window floor, a 13%
/// breakaway and the measured b, between its stops 209..3849 and soft
/// limits 432..3626. The stall timer runs at settings that fold: yield 168,
/// released under 84, after 500 ms.
///
/// Its steady speed is the servo's measured one: 0.2079 x duty% - 0.831
/// counts/ms at 7.9 V (3204 vcounts) and 0.1299 x duty% - 0.852 at 4.39 V
/// (1780). The 2S line is slower than the USB one scaled by the rail, so
/// no single friction meets both: running friction and drag grow with the
/// rail, met at both, interpolated between them and carried beyond.
pub fn bench_mg90(vbus: u16) -> FakeServo {
    let mut s = FakeServo::new(7270.0 / 4096.0);
    s.ends = (209.0, 3849.0);
    s.vbus = vbus as f64;
    s.dynamic = true;
    s.ke = 0.1370;
    s.b = 0.296;
    let over_usb = (vbus as f64 - 1780.0).max(0.0);
    s.fc = 65.78 + 0.004481 * over_usb;
    s.fv = 6.770e-6 * over_usb;
    s.breakaway_q15 = 4259;
    s.soft = Some((432.0, 3626.0));
    s.lease_ms = Some(1008.0);
    s.current_limit = Some(280);
    s.transient_gain = 1.0;
    s.stall_ms = Some(500.0);
    s.stall_yield = 168;
    s.stall_release = 84;
    s
}

/// [`bench_mg90`] with the stall settings the bench servo has saved: yield
/// 545 and release 273, both over its 280 limit. The fold changes nothing
/// and a held stall reads under the release, so the verdict shows for one
/// MEDIUM tick per trip; a locked rotor at these settings trips every
/// 570 ms.
pub fn bench_mg90_saved(vbus: u16) -> FakeServo {
    let mut s = bench_mg90(vbus);
    s.stall_ms = Some(570.0);
    s.stall_yield = 545;
    s.stall_release = 273;
    s
}

/// What the firmware checks before it arms a burst: the volts it applies
/// against a cap, the time since the last arm, and a position strictly
/// inside the soft limits unless the permit is live.
#[derive(Copy, Clone, Debug)]
pub struct ArmRules {
    /// vcounts: 3200 mV behind the terminal dividers.
    pub v_max: u16,
    pub spacing_ms: f64,
    /// Millivolts a vcount, as the host reads the dividers.
    pub mv_per_count: f64,
}

impl ArmRules {
    /// 6k8 over 3k3 taps on a 3.3 V ADC.
    pub const FIRMWARE: ArmRules = ArmRules {
        v_max: 1298,
        spacing_ms: 100.0,
        mv_per_count: 10_100.0 / 3_300.0 * 3_300.0 / 4096.0,
    };
}

/// The observer reads a held stall's disturbance about a tenth under its
/// current, and a shaft that moves as the model predicts near zero.
const TAU_D_OF_HELD: f64 = 0.9;

/// Kernel ticks the servo never runs while one bus transaction is served:
/// the bus handlers preempt the kernel tick, and scans that complete inside
/// one collapse into a single pending tick. Measured on the bench MG90's
/// board: 5.1 ticks per snapshot, a region read and its ident re-read,
/// regressed against the host's clock over the polls of one identification.
pub const TICKS_LOST_PER_TRANSACTION: f64 = 2.5;

pub struct FakeServo {
    pub r: f64,
    pub vbus: f64,
    pub ke: f64,
    pub fc: f64,
    /// Viscous friction, ccounts per c/s (physical model only).
    pub fv: f64,
    pub free_speed: f64,
    /// Steady omega from the motor equation instead of the free_speed
    /// shortcut: omega = (|v| - R*fc) / (Ke + R*fv), signed by duty.
    pub physical_motion: bool,
    /// First-order dynamics for the inertia transient: omega integrates
    /// alpha = b * f_med * (i - fc*sgn - fv*omega) with i = (v - Ke*w)/R,
    /// so the planted `b` is exactly what the estimators must recover.
    /// Steady state matches `physical_motion` by construction.
    pub dynamic: bool,
    /// B, (c/s per medium tick) per ccount (dynamic model).
    pub b: f64,
    /// Medium rate, tick_hz / 10.
    pub f_med: f64,
    /// Static friction: no motion from rest below this |duty| (0 = none).
    pub breakaway_q15: i16,
    /// Reported pos gains +80 counts inside this zone (slip artifact).
    pub glitch_zone: Option<(f64, f64)>,
    /// A nonlinear pot: `pos` is the true position, the raw reading is the
    /// count this table linearizes back to it, and `pos_lin` streams the
    /// table's own Q4 word as the firmware would. None = a linear pot.
    pub pot: Option<GridLut>,
    /// True position, counts.
    pub pos: f64,
    pub ends: (f64, f64),
    /// Wiring convention: false inverts duty's effect on motion, so the
    /// endstop experiment must infer the flipped drive_polarity.
    pub drive_polarity: bool,
    /// Width of the uniform noise the pot's converter adds, raw counts.
    pub pos_noise: f64,
    pub fault_at_ms: Option<f64>,
    /// The Ke the servo carries from an earlier identification, the
    /// firmware's `omega_bemf_cps` divides by; None reads 0, a virgin
    /// servo's.
    pub ke_stored: Option<f64>,
    /// Kernel ticks one bus transaction costs.
    pub ticks_lost_per_txn: f64,
    /// Kernel ticks run since boot: `sample_tick`, and `agg_seq` in
    /// windows of them.
    ticks: f64,
    /// Latch a fault once this many [`Cmd::Burst`]s have been captured.
    pub fault_after_bursts: Option<u32>,
    pub bursts: u32,
    /// The kernel's soft endstop: outbound duty at or past a limit is zeroed
    /// unless the stall permit is live (and honored).
    pub soft: Option<(f64, f64)>,
    /// The permit as last written; [`FakeServo::permit_live`] says whether
    /// it is granted.
    pub permit: bool,
    pub honors_permit: bool,
    /// None: the permit is a level, live while written true.
    pub lease_ms: Option<f64>,
    permit_until: f64,
    /// OpenLoop current limit, counts; None is firmware without a limiter.
    pub current_limit: Option<u16>,
    /// How far over the held current a governed read lands, a fraction:
    /// window peaks over the band.
    pub hold_ripple: f64,
    /// Duty at the current window floor, q15: the least whose drive window
    /// spans 160 ticks of ARR 1200.
    pub floor_q15: i16,
    /// The duty ceiling's last reset: its value and when.
    ceil0: f64,
    t_ceil: f64,
    /// A locked shaft: travel stops at this position and never leaves it.
    pub jam: Option<f64>,
    /// A sticky stretch of travel, `(lo, hi, extra)`: the dynamic model's
    /// friction there is `extra` ccounts over `fc`, from rest and moving.
    pub sticky: Option<(f64, f64, f64)>,
    /// A load that pulls toward low counts whatever the shaft does - a
    /// weight on the horn - as ccounts of current (dynamic model).
    pub load: f64,
    /// The stall timer, ms; None never folds.
    pub stall_ms: Option<f64>,
    pub stall_yield: u16,
    /// The fold holds while the disturbance estimate reads this or more;
    /// 0 holds it until torque off.
    pub stall_release: u16,
    pinned_at: Option<f64>,
    /// The limit folded to the yield.
    folded: bool,
    /// When the stall timer last tripped.
    tripped_at: f64,
    /// None arms every burst; set, the firmware's arm rules apply.
    pub arm_rules: Option<ArmRules>,
    last_arm: f64,
    /// Dynamic-model substeps owed to the clock, a fraction of one.
    sub_carry: f64,
    /// Time the applied duty spent pushing the shaft into the end it sits
    /// at, ms.
    pub pressed_ms: f64,
    /// The ident aggregate's last window-valid current.
    i_valid: f64,
    pub torque: bool,
    pub duty: i16,
    pub tel_mask: u16,
    omega_dyn: f64,
    pub t_ms: f64,
    t_duty_change: f64,
    pub transient_windows: f64,
    pub transient_gain: f64,
    /// The plant a [`Cmd::Burst`] captures from; the burst's own mask
    /// replaces `chans`.
    pub burst: SynthBurst,
    lcg: u64,
}

impl FakeServo {
    pub fn new(r: f64) -> Self {
        Self {
            r,
            vbus: 1731.0,
            ke: 0.1731,
            fc: 20.0,
            fv: 0.0,
            free_speed: 10_000.0,
            physical_motion: false,
            dynamic: false,
            b: 0.1,
            f_med: TICK_HZ / 10.0,
            breakaway_q15: 0,
            glitch_zone: None,
            pot: None,
            pos: 2400.0,
            ends: (200.0, 4000.0),
            drive_polarity: true,
            pos_noise: 0.0,
            fault_at_ms: None,
            ke_stored: None,
            ticks_lost_per_txn: TICKS_LOST_PER_TRANSACTION,
            ticks: 0.0,
            fault_after_bursts: None,
            bursts: 0,
            soft: None,
            permit: false,
            honors_permit: true,
            lease_ms: None,
            permit_until: f64::NEG_INFINITY,
            current_limit: None,
            hold_ripple: 0.0,
            floor_q15: 4356,
            ceil0: 0.0,
            t_ceil: 0.0,
            jam: None,
            sticky: None,
            load: 0.0,
            stall_ms: None,
            stall_yield: 0,
            stall_release: 0,
            pinned_at: None,
            folded: false,
            tripped_at: f64::NEG_INFINITY,
            arm_rules: None,
            last_arm: f64::NEG_INFINITY,
            sub_carry: 0.0,
            pressed_ms: 0.0,
            i_valid: 0.0,
            torque: false,
            duty: 0,
            tel_mask: 0,
            omega_dyn: 0.0,
            t_ms: 0.0,
            t_duty_change: -1e9,
            transient_windows: 3.0,
            transient_gain: 1.5,
            // r 8 ohm keeps the whole default duty ladder inside the 0.4 A
            // envelope, so the plan is not pruned by accident
            burst: SynthBurst {
                r: 8.0,
                ..SynthBurst::board_d()
            },
            lcg: 0x9E3779B97F4A7C15,
        }
    }

    fn noise(&mut self) -> f64 {
        self.lcg = self
            .lcg
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        // uniform in [-0.5, 0.5) scaled by pos_noise
        ((self.lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * self.pos_noise
    }

    /// The raw count the pot reads at a true position, `noise` raw counts
    /// of the converter's own on top: the count whose linearization lands
    /// nearest it, found by bisection over the monotone table.
    fn read_pot(&self, counts: f64, noise: f64) -> u16 {
        let raw = match &self.pot {
            None => counts,
            Some(_) => self.raw_of(counts) as f64,
        };
        (raw + noise).round().clamp(0.0, 4095.0) as u16
    }

    fn raw_of(&self, counts: f64) -> u16 {
        let Some(lut) = &self.pot else {
            return counts.round().clamp(0.0, 4095.0) as u16;
        };
        let (mut lo, mut hi) = (0u16, 4095u16);
        while lo < hi {
            let mid = (lo + hi) / 2;
            if lut.counts(mid) < counts {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        if lo > 0 && (lut.counts(lo - 1) - counts).abs() < (lut.counts(lo) - counts).abs() {
            lo - 1
        } else {
            lo
        }
    }

    fn q4_of(&self, raw: u16) -> u16 {
        match &self.pot {
            Some(lut) => lut.q4(raw),
            None => raw << 4,
        }
    }

    /// Fast ticks a second: ten per MEDIUM tick.
    pub fn tick_hz(&self) -> f64 {
        self.f_med * 10.0
    }

    /// One ident aggregate window, ms.
    fn window_ms(&self) -> f64 {
        TICKS_PER_WINDOW as f64 * 1000.0 / self.tick_hz()
    }

    /// Time a bus transaction holds the kernel off for `lost` of its
    /// ticks: the plant runs through all `ms`, the tick counter only
    /// through what is left.
    pub fn busy_ms(&mut self, ms: f64, lost: f64) {
        let ran = self.ticks;
        self.advance_ms(ms);
        self.ticks -= lost.min(self.ticks - ran);
    }

    pub fn permit_live(&self) -> bool {
        match self.lease_ms {
            None => self.permit,
            Some(_) => self.permit && self.torque && self.t_ms < self.permit_until,
        }
    }

    fn endstop_blocks(&self) -> bool {
        let out = self.duty as f64 * if self.drive_polarity { 1.0 } else { -1.0 };
        self.soft
            .is_some_and(|(lo, hi)| (self.pos <= lo && out < 0.0) || (self.pos >= hi && out > 0.0))
            && !(self.permit_live() && self.honors_permit)
    }

    /// The duty the bridge actually sees: zero with torque off, zero driving
    /// outward at a soft limit the permit does not open, and the governed
    /// duty under the limiter.
    fn applied(&self) -> i16 {
        if !self.torque || self.endstop_blocks() {
            0
        } else {
            self.govern().0
        }
    }

    /// The kernel's duty ceiling now: up 128 q15 per fast tick from its
    /// last reset, never under the floor.
    fn ceiling(&self) -> f64 {
        // whole ticks only; the epsilon keeps a tick boundary from rounding down
        let ticks = ((self.t_ms - self.t_ceil) * self.f_med / 100.0 + 1e-6)
            .floor()
            .max(0.0);
        (self.ceil0 + 128.0 * ticks).clamp(self.floor_q15 as f64, 32767.0)
    }

    /// The limit in force: the current limit, or the yield once folded.
    fn limit(&self) -> Option<u16> {
        self.current_limit.map(|l| {
            if self.folded {
                l.min(self.stall_yield)
            } else {
                l
            }
        })
    }

    /// The duty the limiter applies for the goal, and whether the current,
    /// not the slew, holds it under the goal (`limit_flags` bit 0).
    fn govern(&self) -> (i16, bool) {
        let Some(lim) = self.limit() else {
            return (self.duty, false);
        };
        let sign = self.duty.signum() as i32;
        let goal = self.duty.unsigned_abs() as i32;
        let floor = self.floor_q15 as i32;
        let base = ((lim as f64 * self.r / self.vbus * 32767.0) as i32).min(floor);
        if goal < floor {
            return ((sign * goal.min(base)) as i16, false);
        }
        let slewed = goal.min(self.ceiling() as i32);
        // the low-side shunt sees drive current only, never the brake
        // current a low duty draws from a fast shaft
        let cur = |mag: i32| self.i_at((sign * mag) as i16) * sign as f64;
        let band = (lim - (lim >> 3)) as f64;
        if cur(slewed) < band {
            return ((sign * slewed) as i16, false);
        }
        // the largest duty under `slewed` whose current passes `ok`
        let largest = |ok: &dyn Fn(f64) -> bool| {
            let (mut lo, mut hi) = (0, slewed);
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if ok(cur(mid)) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            lo
        };
        let lim = lim as f64;
        let top = if cur(slewed) <= lim {
            slewed
        } else {
            largest(&|i| i <= lim)
        };
        // the ceiling's steps from where it last reset
        let step = SLEW_Q15_PER_TICK as i32;
        let from = (self.ceil0 as i32).max(floor);
        let on_grid = |d: i32| from + (d - from).div_euclid(step) * step;
        let held = if top < from {
            top
        } else if self.moving_at((sign * top) as i16) {
            let first = (on_grid(largest(&|i| i < band)) + step).max(from);
            first.min(top)
        } else if top == slewed || cur(on_grid(top)) < band {
            top
        } else {
            on_grid(top)
        };
        let applied = if held < floor { base.min(goal) } else { held };
        ((sign * applied) as i16, applied < goal)
    }

    /// The firmware's `limit_flags`: bit 0 the current holds the duty under
    /// the goal, bit 1 the stall timer folded the limit (or tripped within
    /// the last MEDIUM tick), bit 2 the endstop blocks, bit 3 the permit is
    /// live.
    pub fn limit_flags(&self) -> u8 {
        let driving = self.torque && self.duty != 0;
        let mut f = 0;
        if driving && self.endstop_blocks() {
            f |= 4;
        } else if driving && self.govern().1 {
            f |= 1;
        }
        if self.folded || self.t_ms - self.tripped_at < self.medium_ms() {
            f |= 2;
        }
        if self.permit_live() {
            f |= 8;
        }
        f
    }

    fn medium_ms(&self) -> f64 {
        1000.0 / self.f_med
    }

    /// The shaft moves with `duty` applied.
    fn moving_at(&self, duty: i16) -> bool {
        if self.dynamic {
            self.omega_dyn != 0.0
        } else {
            self.omega_at(duty) != 0.0
        }
    }

    fn pressing(&self) -> bool {
        let out = self.applied() as f64 * if self.drive_polarity { 1.0 } else { -1.0 };
        (self.pos <= self.ends.0 && out < 0.0) || (self.pos >= self.ends.1 && out > 0.0)
    }

    /// The current, not the slew, holds the goal back on a still shaft.
    fn pinned(&self) -> bool {
        if !self.torque || self.duty == 0 || self.endstop_blocks() || self.omega() != 0.0 {
            return false;
        }
        self.govern().1
    }

    /// The observer's disturbance estimate: a held stall's current, a tenth
    /// under; nothing on a shaft that moves or is not driven.
    fn tau_d(&self) -> f64 {
        let duty = self.applied();
        if duty == 0 || self.omega() != 0.0 {
            return 0.0;
        }
        TAU_D_OF_HELD * self.i_at(duty).abs()
    }

    /// Run the stall timer over time that began at `from_ms`. A live permit
    /// clears it; a fold releases once the disturbance falls under the
    /// release, and a trip that would release at once re-arms a full window
    /// a MEDIUM tick later.
    fn stall_timer(&mut self, from_ms: f64) {
        let Some(ms) = self.stall_ms else {
            return;
        };
        if self.permit_live() && self.honors_permit {
            self.folded = false;
            self.pinned_at = None;
            return;
        }
        if self.folded && self.tau_d() < self.stall_release as f64 {
            self.folded = false;
            self.pinned_at = None;
        }
        if self.folded || !self.pinned() {
            self.pinned_at = None;
            return;
        }
        let mut at = *self.pinned_at.get_or_insert(from_ms);
        while self.t_ms - at >= ms {
            let trip = at + ms;
            self.folded = true;
            if self.tau_d() >= self.stall_release as f64 {
                break;
            }
            self.folded = false;
            self.tripped_at = trip;
            at = trip + self.medium_ms();
        }
        self.pinned_at = (!self.folded).then_some(at);
    }

    /// What the host reads back to explain a refused arm.
    fn arm_seen(&mut self) -> ArmSeen {
        let o = self.read();
        let soft = self.soft.map_or((i32::MIN, i32::MAX), |(lo, hi)| {
            (lo.round() as i32, hi.round() as i32)
        });
        ArmSeen {
            vbus_counts: o.vbus_counts,
            mv_per_count: self.arm_rules.map_or(0.0, |r| r.mv_per_count),
            pos: o.pos,
            soft,
            limit_flags: o.limit_flags,
            fault_flags: o.fault_flags,
        }
    }

    /// The firmware's answer to a burst arm at `duty_q15`: Err names the
    /// rule that refused it. A refused arm starts no spacing.
    pub fn arm(&mut self, duty_q15: i16) -> Result<(), &'static str> {
        let Some(rules) = self.arm_rules else {
            return Ok(());
        };
        let volts = (duty_q15.unsigned_abs() as u32 * self.vbus as u32) >> 15;
        let inside = self
            .soft
            .is_none_or(|(lo, hi)| self.pos > lo && self.pos < hi);
        if !self.torque {
            Err("torque off")
        } else if volts > rules.v_max as u32 {
            Err("over the burst volts cap")
        } else if self.t_ms - self.last_arm < rules.spacing_ms {
            Err("inside the burst spacing")
        } else if !inside && !self.permit_live() {
            Err("outside the soft limits without the permit")
        } else {
            self.last_arm = self.t_ms;
            Ok(())
        }
    }

    fn at_jam(&self) -> bool {
        self.jam == Some(self.pos)
    }

    /// Where travel toward `to` ends: at the stops, or at the jam when it
    /// lies on the way.
    fn travel(&self, to: f64) -> f64 {
        let to = to.clamp(self.ends.0, self.ends.1);
        match self.jam {
            Some(j) if (self.pos - j) * (to - j) <= 0.0 => j,
            _ => to,
        }
    }

    fn omega(&self) -> f64 {
        if self.dynamic {
            return self.omega_dyn;
        }
        self.omega_at(self.applied())
    }

    /// Steady speed at an applied duty, for the models without dynamics.
    fn omega_at(&self, duty: i16) -> f64 {
        if duty == 0 || duty.unsigned_abs() < self.breakaway_q15 as u16 {
            return 0.0;
        }
        let vsign = duty.signum() as f64 * if self.drive_polarity { 1.0 } else { -1.0 };
        let stalled = (self.pos <= self.ends.0 && vsign < 0.0)
            || (self.pos >= self.ends.1 && vsign > 0.0)
            || self.at_jam();
        if stalled {
            return 0.0;
        }
        if self.physical_motion {
            let v = duty.unsigned_abs() as f64 / 32767.0 * self.vbus;
            let mag = ((v - self.r * self.fc) / (self.ke + self.r * self.fv)).max(0.0);
            mag * vsign
        } else {
            duty.unsigned_abs() as f64 / 32767.0 * self.free_speed * vsign
        }
    }

    /// The winding current the dynamic model carries right now: ohmic on
    /// the applied volts minus bemf. Friction is mechanical - it consumes
    /// torque, not extra current - so nothing else is added.
    fn i_dyn(&self) -> f64 {
        let duty = self.applied();
        if duty == 0 {
            return 0.0;
        }
        let v = duty as f64 / 32767.0 * self.vbus;
        (v - self.ke * self.omega_dyn) / self.r
    }

    /// The settled current an applied duty draws right now, before the L
    /// transient. Friction current only while the free-speed shortcut
    /// moves: stalled current is ohmic, and the physical and dynamic models
    /// need no extra term - their (v - ke*omega)/r IS the winding current
    /// at every instant.
    fn i_at(&self, duty: i16) -> f64 {
        if duty == 0 {
            return 0.0;
        }
        let v = duty as f64 / 32767.0 * self.vbus;
        let omega = if self.dynamic {
            self.omega_dyn
        } else {
            self.omega_at(duty)
        };
        let fric = if omega != 0.0 && !self.physical_motion && !self.dynamic {
            self.fc * duty.signum() as f64
        } else {
            0.0
        };
        (v - self.ke * omega) / self.r + fric
    }

    pub fn write(&mut self, reg: Reg, value: i32) {
        if reg == control::TORQUE_ENABLE {
            let on = value != 0;
            if on && !self.torque {
                self.reset_ceiling(self.floor_q15 as f64);
            }
            if !on {
                self.folded = false;
                self.pinned_at = None;
            }
            self.torque = on;
        } else if reg == control::GOAL_DUTY {
            let goal = value as i16;
            // the ceiling carries through a goal change in the same direction
            // and restarts from the floor on a reversal or a start from rest
            let from = if self.duty != 0 && goal.signum() == self.duty.signum() {
                self.ceiling().min(self.duty.unsigned_abs() as f64)
            } else {
                self.floor_q15 as f64
            };
            self.reset_ceiling(from);
            self.duty = goal;
            self.t_duty_change = self.t_ms;
        } else if reg == control::TEL_MASK {
            self.tel_mask = value as u16;
        } else if reg == control::STALL_PERMIT {
            self.permit = value != 0;
            if let Some(lease) = self.lease_ms {
                self.permit_until = if self.permit && self.torque {
                    self.t_ms + lease
                } else {
                    f64::NEG_INFINITY
                };
            }
        }
        if !self.torque {
            self.permit_until = f64::NEG_INFINITY;
        }
    }

    fn reset_ceiling(&mut self, from: f64) {
        self.ceil0 = from;
        self.t_ceil = self.t_ms;
    }

    /// One dynamic-model integration substep.
    fn substep(&mut self, dt: f64) {
        let i = self.i_dyn() - self.load;
        let w = self.omega_dyn;
        let fc = match self.sticky {
            Some((lo, hi, extra)) if (lo..=hi).contains(&self.pos) => self.fc + extra,
            _ => self.fc,
        };
        let fric = if w != 0.0 {
            fc * w.signum() + self.fv * w
        } else if i.abs() > fc && self.applied().unsigned_abs() >= self.breakaway_q15 as u16 {
            fc * i.signum()
        } else {
            i // no net torque below stiction: alpha = 0
        };
        let alpha = self.b * self.f_med * (i - fric);
        let w2 = w + alpha * dt;
        // coasting friction never reverses the spin through zero
        self.omega_dyn = if self.applied() == 0 {
            if w != 0.0 && w.signum() != w2.signum() {
                0.0
            } else {
                w2
            }
        } else {
            w2
        };
        self.pos = self.travel(self.pos + self.omega_dyn * dt);
        if (self.pos <= self.ends.0 && self.omega_dyn < 0.0)
            || (self.pos >= self.ends.1 && self.omega_dyn > 0.0)
            || self.at_jam()
        {
            self.omega_dyn = 0.0;
        }
    }

    /// One fast-tick advance shared by [`advance`] and [`stream`].
    fn tick(&mut self, dt: f64) {
        if self.dynamic {
            self.substep(dt);
        } else {
            self.pos = self.travel(self.pos + self.omega() * dt);
        }
    }

    pub fn advance(&mut self, ms: u32) {
        self.advance_ms(ms as f64);
    }

    pub fn advance_ms(&mut self, ms: f64) {
        if ms <= 0.0 {
            return;
        }
        let from = self.t_ms;
        self.advance_plant(ms);
        if self.pressing() {
            self.pressed_ms += self.t_ms - from;
        }
        self.stall_timer(from);
    }

    fn advance_plant(&mut self, ms: f64) {
        self.ticks += ms * self.tick_hz() / 1000.0;
        if self.dynamic {
            // tick-sized substeps keep the ~tens-of-ms tau integration exact
            let dt = 1.0 / (self.f_med * 10.0);
            let owed = self.sub_carry + ms / 1000.0 / dt;
            let n = owed.floor();
            self.sub_carry = owed - n;
            let t0 = self.t_ms;
            for k in 0..n as u64 {
                self.substep(dt);
                self.t_ms = t0 + (k + 1) as f64 * dt * 1000.0;
            }
            self.t_ms = t0;
        } else {
            let dt = ms / 1000.0;
            self.pos = self.travel(self.pos + self.omega() * dt);
        }
        self.t_ms += ms;
    }

    /// One armed TEL burst: `samples` fast ticks integrated from t0 (the
    /// arm instant - any goal write is already applied), one frame per tick
    /// with the mask-selected fields. Mask 0 streams nothing, like the
    /// firmware's disarmed producer.
    pub fn stream(&mut self, samples: u16, sink: &mut Vec<TelFrame>) {
        let dt = 1.0 / (self.f_med * 10.0);
        let t0 = self.t_ms;
        for k in 0..samples {
            self.tick(dt);
            self.t_ms = t0 + (k + 1) as f64 * dt * 1000.0;
            if self.pressing() {
                self.pressed_ms += dt * 1000.0;
            }
            if self.tel_mask == 0 {
                continue;
            }
            let noise = self.noise();
            let duty = self.applied();
            let driving = duty != 0;
            let sel = |bit: u16| self.tel_mask & bit != 0;
            let i = if self.dynamic {
                self.i_dyn()
            } else {
                let v = duty as f64 / 32767.0 * self.vbus;
                if driving {
                    (v - self.ke * self.omega()) / self.r
                } else {
                    0.0
                }
            };
            let raw = self.read_pot(self.pos, noise);
            sink.push(TelFrame {
                tick: k as u64,
                window_valid: driving,
                pos: sel(1 << 0).then_some(raw),
                current: sel(1 << 1).then(|| i.round() as i16),
                current_trough: sel(1 << 2).then_some(512),
                duty_q15: sel(1 << 3).then_some(duty),
                vdiff: sel(1 << 4).then(|| {
                    if driving {
                        (self.vbus * duty.signum() as f64) as i16
                    } else {
                        0
                    }
                }),
                vbus: sel(1 << 5).then_some(self.vbus as u16),
                // raw terminal fakes: driven side carries the rail, the
                // other sits low; raw current rides a 512-count bias and
                // is a MAGNITUDE - the low-side shunt sees drive current
                // the same way whichever way the bridge is pointed
                // (window.rs applies the direction sign downstream)
                current_raw: sel(1 << 6).then(|| (512.0 + i.abs()).round() as u16),
                vmotor_a: sel(1 << 7).then_some(if duty > 0 { self.vbus as u16 } else { 0 }),
                vmotor_b: sel(1 << 8).then_some(if duty < 0 { self.vbus as u16 } else { 0 }),
                vbus_raw: sel(1 << 9).then_some(self.vbus as u16),
                ntc_raw: sel(1 << 10).then_some(2048),
                pos_lin: sel(1 << 11).then(|| self.q4_of(raw)),
            });
        }
        self.t_ms = t0 + samples as f64 * dt * 1000.0;
        self.ticks += samples as f64;
        self.stall_timer(t0);
    }

    pub fn read(&mut self) -> TelemetrySnapshot {
        let duty = self.applied();
        let driving = duty != 0;
        let blind = self.current_limit.is_some() && duty.unsigned_abs() < self.floor_q15 as u16;
        let (i, vdiff) = if driving && blind {
            (self.i_valid, self.vbus * duty.signum() as f64)
        } else if driving {
            let mut i = self.i_at(duty);
            if (self.t_ms - self.t_duty_change) / self.window_ms() < self.transient_windows {
                i *= self.transient_gain;
            }
            if self.limit_flags() & 1 != 0 {
                i *= 1.0 + self.hold_ripple;
            }
            self.i_valid = i;
            (i, self.vbus * duty.signum() as f64)
        } else {
            (0.0, 0.0)
        };
        let fault = matches!(self.fault_at_ms, Some(at) if self.t_ms >= at)
            || matches!(self.fault_after_bursts, Some(n) if self.bursts >= n);
        let glitch = match self.glitch_zone {
            Some((lo, hi)) if (lo..=hi).contains(&self.pos) => 80.0,
            _ => 0.0,
        };
        let noise = self.noise();
        let pos = self.read_pot(self.pos + glitch, noise);
        let omega_bemf = match self.ke_stored {
            Some(ke) if driving => {
                let v = duty as f64 / 32767.0 * self.vbus;
                (v - self.r * self.i_at(duty)) / ke
            }
            _ => 0.0,
        };
        TelemetrySnapshot {
            host_ms: self.t_ms,
            omega_bemf_cps: omega_bemf.round().clamp(-32768.0, 32767.0) as i16,
            sample_tick: self.ticks as u64 as u32,
            fault_flags: if fault { 32 } else { 0 },
            fault_code: if fault { 6 } else { 0 },
            pos,
            current: (512.0 + i).round() as u16,
            current_bias_counts: 512,
            vbus_counts: self.vbus as u16,
            i_mean_counts: i.round() as i16,
            vdiff_mean: vdiff.round() as i16,
            duty_mean_q15: duty,
            duty_applied_q15: duty,
            i_lim_counts: self.limit().unwrap_or(0),
            limit_flags: self.limit_flags(),
            window_floor_q15: self.floor_q15 as u16,
            agg_seq: (self.ticks as u64 / TICKS_PER_WINDOW as u64) as u16,
            ..Default::default()
        }
    }
}

/// A switched RL winding sampled the way the firmware burst samples it: a
/// centre-aligned PWM period of `2 * arr` HCLK, the drive window straddling
/// the crest, the shunt live only during ON, and a first-order amplifier
/// lag on the edge. Separate from [`FakeServo`], which has no electrical
/// model at microsecond resolution.
///
/// The bridge is explicit when `rds` and `r_shunt` are set: during ON the
/// driven terminal sits one high-side drop under the rail node and the idle
/// terminal one low-side drop plus the shunt above ground; during the brake
/// both low sides short the winding and nothing crosses the shunt. With
/// both zero, `r` is the whole loop, the model the older tests use.
///
/// `c_island` is decoupling inside the shunt's ground island, fed from the
/// rail node through `r_feed`: it supplies part of each pulse and is
/// repaid through the shunt during OFF, so the shunt carries the island's
/// feed current rather than the winding's.
#[derive(Clone, Debug)]
pub struct SynthBurst {
    /// Winding resistance, ohms; the whole loop when `rds` and `r_shunt`
    /// are zero.
    pub r: f64,
    pub l: f64,
    /// Open-circuit source voltage.
    pub v_rail: f64,
    /// Source resistance behind `c_bulk`; zero is a stiff rail.
    pub r_src: f64,
    pub c_bulk: f64,
    pub c_island: f64,
    pub r_feed: f64,
    /// Per bridge FET, ohms.
    pub rds: f64,
    pub r_shunt: f64,
    /// Brush drop against the winding current, volts.
    pub v0: f64,
    /// Back-EMF growth after the step latches, volts per ms.
    pub emf_v_per_ms: f64,
    /// The chopping leg's low side never turns on: its body diode carries
    /// the whole OFF phase.
    pub body_diode: bool,
    pub arr: u16,
    pub bias: f64,
    /// Shunt amplifier edge time constant, microseconds.
    pub settle_us: f64,
    /// Terminal tap RC, microseconds.
    pub tap_us: f64,
    /// Terminal divider bias node, raw counts.
    pub vb: f64,
    /// Tap B's code over tap A's with the winding at rest, counts: the two
    /// dividers never match.
    pub split: f64,
    /// Peak-to-peak measurement noise, counts.
    pub noise_counts: f64,
    pub amps_per_count: f64,
    pub v_rail_per_count: f64,
    pub v_term_per_count: f64,
    pub adc_lsb_v: f64,
    pub step_index: u16,
    /// Samples from the step to the crest whose update event latches the
    /// new compare value. The window straddling that crest is a HALF
    /// window - the bench captures show it and so must this.
    pub latch_delay: usize,
    pub chans: u8,
}

/// Body diode forward drop, volts.
const DIODE_V: f64 = 0.7;

/// Periods a from-a-hold capture runs at its pre-step duty before sample 0,
/// so the pre-step half and the pre-arm rail read are settled.
const PRIME_PERIODS: usize = 40;

/// Where the servo's scan samples the rail: slot 5 of the crest scan,
/// 2.16 us after the crest, in conversion periods.
const PRE_ARM_RAIL_SAMPLES: f64 = 2.0;

impl SynthBurst {
    /// Board D as fitted: 60 mohm shunt at G 15, 15k/10k rail tap, 6k8/3k3
    /// terminal taps to a 779-count bias node, a 4.37 V USB rail, ARR 1200.
    pub fn board_d() -> Self {
        let lsb = 3.3 / 4096.0;
        Self {
            r: 4.0,
            l: 0.6e-3,
            v_rail: 4.37,
            r_src: 0.0,
            c_bulk: 100e-6,
            c_island: 0.0,
            r_feed: 0.0,
            rds: 0.0,
            r_shunt: 0.0,
            v0: 0.0,
            emf_v_per_ms: 0.0,
            body_diode: false,
            arr: 1200,
            bias: 112.0,
            settle_us: 0.9,
            tap_us: 0.33,
            vb: 779.0,
            split: 0.0,
            noise_counts: 2.0,
            amps_per_count: lsb / (15.0 * 0.060),
            v_rail_per_count: lsb * 2.5,
            v_term_per_count: lsb * 10_100.0 / 3_300.0,
            adc_lsb_v: lsb,
            step_index: 485,
            latch_delay: 21,
            chans: 0,
        }
    }

    /// Board D's bridge made explicit: DRV8212P FETs and the 60 mohm shunt.
    pub fn with_bridge(self) -> Self {
        Self {
            rds: 0.140,
            r_shunt: 0.060,
            ..self
        }
    }

    /// Samples per PWM period at the burst's own sample clock.
    pub fn period(&self) -> f64 {
        2.0 * self.arr as f64 / SAMPLE_HCLK
    }

    pub fn capture(&self, step_q15: i16, pre_q15: i16) -> Capture {
        const SUBSTEPS: usize = 8;
        let p = self.period();
        let crest0 = self.step_index as f64 + self.latch_delay as f64;
        let duty_at = |k: f64| {
            let q = if k >= 0.0 { step_q15 } else { pre_q15 };
            (q as f64 / 32767.0).abs()
        };
        // (on, duty of the half period x falls in, offset from its crest)
        let phase = |x: f64| {
            let k = ((x - crest0) / p).round();
            let u = x - (crest0 + k * p);
            let d = if u >= 0.0 {
                duty_at(k)
            } else {
                duty_at(k - 1.0)
            };
            (u.abs() <= d * p / 2.0, d, u)
        };
        let dt = SAMPLE_US * 1e-6 / SUBSTEPS as f64;
        let a_amp = 1.0 - (-SAMPLE_US / SUBSTEPS as f64 / self.settle_us).exp();
        let a_tap = 1.0 - (-SAMPLE_US / SUBSTEPS as f64 / self.tap_us).exp();
        let vb_v = self.vb * self.adc_lsb_v;
        let fwd = step_q15 >= 0;
        let slots: Vec<u8> = [CHAN_VMOTOR_A, CHAN_VMOTOR_B, CHAN_VBUS]
            .into_iter()
            .filter(|b| self.chans & b != 0)
            .collect();
        let fl = frame_len(self.chans);

        let mut pre_arm_rail = self.v_rail;
        // winding current, rail node, island, amplifier, tap A, tap B
        let step = |x: f64, st: &mut [f64; 6]| {
            let [i, vc, isl, m, ta, tb] = st;
            let (on, d, _) = phase(x);
            let drive = on && d != 0.0;
            let e = self.emf_v_per_ms * ((x - crest0).max(0.0) * SAMPLE_US * 1e-3);
            let v0 = if *i > 0.0 { self.v0 } else { 0.0 };
            let i_bridge = if drive { *i } else { 0.0 };
            // (bridge supply above PGND, shunt current)
            let (vm, i_sh) = if self.c_island > 0.0 {
                (*isl, (*vc - *isl) / (self.r_feed + self.r_shunt))
            } else {
                (*vc - i_bridge * self.r_shunt, i_bridge)
            };
            let pgnd = i_sh * self.r_shunt;
            // terminals against ground, in the drive frame: hi is the
            // chopping leg
            let (hi, lo) = if d == 0.0 {
                (vb_v, vb_v)
            } else if on {
                (vm + pgnd - *i * self.rds, pgnd + *i * self.rds)
            } else if self.body_diode && *i > 0.0 {
                (-DIODE_V, pgnd + *i * self.rds)
            } else {
                (pgnd - *i * self.rds, pgnd + *i * self.rds)
            };
            if d != 0.0 {
                *i += (hi - lo - *i * self.r - v0 - e) / self.l * dt;
                if !on {
                    *i = i.max(0.0);
                }
            }
            if self.c_island > 0.0 {
                *isl += (i_sh - i_bridge) / self.c_island * dt;
            }
            if self.r_src > 0.0 {
                *vc += ((self.v_rail - *vc) / self.r_src - i_sh) / self.c_bulk * dt;
            }
            *m += (self.bias + i_sh / self.amps_per_count - *m) * a_amp;
            let (va, vbv) = if fwd { (hi, lo) } else { (lo, hi) };
            *ta += (va - *ta) * a_tap;
            *tb += (vbv - *tb) * a_tap;
        };
        let mut st = [0.0, self.v_rail, self.v_rail, self.bias, vb_v, vb_v];
        if pre_q15 != 0 {
            let x0 = -(PRIME_PERIODS as f64) * p;
            let n = (PRIME_PERIODS as f64 * p * SUBSTEPS as f64) as usize;
            for s in 0..n {
                let x = x0 + s as f64 / SUBSTEPS as f64;
                step(x, &mut st);
                let (_, _, u) = phase(x);
                if (PRE_ARM_RAIL_SAMPLES..PRE_ARM_RAIL_SAMPLES + 1.0 / SUBSTEPS as f64).contains(&u)
                {
                    pre_arm_rail = st[1];
                }
            }
        }
        let mut lcg = 0x2545F4914F6CDD1Du64;
        let mut noise = || {
            lcg = lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * self.noise_counts
        };
        let tap_code = |v: f64| self.vb + (v - vb_v) / self.v_term_per_count;
        let mut samples = Vec::with_capacity(SAMPLES);
        for n in 0..SAMPLES {
            for s in 0..SUBSTEPS {
                let x = n as f64 + s as f64 / SUBSTEPS as f64;
                step(x, &mut st);
            }
            let code = match n % fl {
                0 => st[3],
                slot => match slots[slot - 1] {
                    CHAN_VMOTOR_A => tap_code(st[4]),
                    CHAN_VMOTOR_B => tap_code(st[5]) + self.split,
                    _ => st[1] / self.v_rail_per_count,
                },
            };
            samples.push((code + noise()).round().clamp(0.0, 4095.0) as u16);
        }
        Capture {
            samples,
            meta: Meta {
                pre_q15,
                step_q15,
                step_index: self.step_index,
                start_cnt: 1094,
                pwm_arr: self.arr,
                start_dir: 1,
                restore_dir: 0,
                vbus_raw: (pre_arm_rail / self.v_rail_per_count).round() as u16,
                bias: self.bias.round() as u16,
                chans: self.chans,
                frame_len: fl as u8,
                vmotor_bias: self.vb.round() as u16,
                ..Meta::default()
            },
        }
    }
}

/// A pot bent by one full sine over its travel, +/-120 counts, identity
/// at and beyond the fake's stops (200, 4000): local gain 0.8x..1.2x, so a
/// raw-count fit over part of the travel is biased by the gain there.
pub fn bent_pot() -> GridLut {
    let mut lut = GridLut::IDENTITY;
    let (lo, hi) = (13usize, 250usize);
    for (k, c) in lut.points.iter_mut().enumerate().take(hi).skip(lo + 1) {
        let x = (k - lo) as f64 / (hi - lo) as f64;
        *c = (120.0 * (core::f64::consts::TAU * x).sin()).round() as i16;
    }
    lut
}

fn reg_name(reg: Reg) -> &'static str {
    ALL.iter()
        .find(|(_, r)| *r == reg)
        .map(|(n, _)| *n)
        .unwrap_or("?")
}

/// What the pump charges each transaction, ms, the fake servo moving
/// through it. A snapshot is sampled halfway through its read and a write
/// applies halfway through its own.
#[derive(Copy, Clone, Debug)]
pub struct Bus {
    /// One telemetry snapshot: the region read and the ident re-read.
    pub read_ms: f64,
    pub write_ms: f64,
    /// A requested pause takes this many times as long.
    pub pause_scale: f64,
    /// Seeds which reads are slow: [`SLOW_READS_PCT`] in a hundred take
    /// [`SLOW_READ_MS`] more.
    pub slow_reads: Option<u64>,
}

pub const SLOW_READS_PCT: u64 = 3;
pub const SLOW_READ_MS: f64 = 8.0;

/// Transactions a burst capture makes before its samples start (four
/// field reads, three held writes and the commit) and after (the done
/// poll, the tail, the page walk and the release).
const BURST_ARM_TXNS: f64 = 8.0;
const BURST_READBACK_TXNS: f64 = 6.0;

impl Bus {
    /// The bench bus, from a recorded run: a snapshot 3.5 ms, a write
    /// 1.7 ms, a 30 ms pause 39.5 ms from one read to the next.
    pub const BENCH: Bus = Bus {
        read_ms: 3.5,
        write_ms: 1.7,
        pause_scale: 1.2,
        slow_reads: None,
    };

    /// Every transaction instant: only for a pin about the experiment's
    /// logic, not its timing.
    pub const ZERO_LATENCY: Bus = Bus {
        read_ms: 0.0,
        write_ms: 0.0,
        pause_scale: 1.0,
        slow_reads: None,
    };

    /// A snapshot every `ms`: the bench ladder read one every 1.9 ms.
    pub fn with_read_ms(self, ms: f64) -> Self {
        Self {
            read_ms: ms,
            ..self
        }
    }

    pub fn with_slow_reads(self, seed: u64) -> Self {
        Self {
            slow_reads: Some(seed),
            ..self
        }
    }
}

/// Drive an experiment against the fake servo on the bench bus
/// ([`pump_on`]).
pub fn pump<E: Experiment>(exp: &mut E, servo: &mut FakeServo, max_steps: u32) -> Vec<String> {
    pump_on(exp, servo, max_steps, Bus::BENCH)
}

fn charge_write(servo: &mut FakeServo, lease: &mut PermitLease, bus: &Bus, reg: Reg, value: i32) {
    let lost = servo.ticks_lost_per_txn / 2.0;
    servo.busy_ms(bus.write_ms / 2.0, lost);
    servo.write(reg, value);
    lease.wrote(reg, value, servo.t_ms);
    servo.busy_ms(bus.write_ms / 2.0, lost);
}

/// Drive an experiment against the fake servo over `bus`; returns the
/// command log ("write <field> <value>" entries, "stream <samples> [<field>
/// <value>]" per burst, plus a trailing marker on overrun). A Stream arm
/// applies its goal at t0, synthesizes the burst's per-tick frames from the
/// plant, and hands them back through `push_tel` - the driver contract. A
/// held stall permit is rewritten on the fake clock the way the CLI's pump
/// rewrites it on the wall clock, pauses sliced so none outlasts a refresh.
/// A burst arm the servo refuses ends the run the way the CLI's does: the
/// error, then its guard's writes.
pub fn pump_on<E: Experiment>(
    exp: &mut E,
    servo: &mut FakeServo,
    max_steps: u32,
    bus: Bus,
) -> Vec<String> {
    let mut log = Vec::new();
    let mut pending: Option<TelemetrySnapshot> = None;
    let mut frames = Vec::new();
    let mut lease = PermitLease::default();
    let mut lcg = bus.slow_reads.unwrap_or(0);
    let keep = |lease: &mut PermitLease, servo: &mut FakeServo, log: &mut Vec<String>| {
        if lease.due(servo.t_ms) {
            charge_write(servo, lease, &bus, control::STALL_PERMIT, 1);
            log.push("write stall_permit 1".into());
        }
    };
    for _ in 0..max_steps {
        match exp.step(pending.take().as_ref()) {
            Cmd::Write { reg, value } => {
                charge_write(servo, &mut lease, &bus, reg, value);
                log.push(format!("write {} {}", reg_name(reg), value));
            }
            Cmd::Read => {
                let mut ms = bus.read_ms;
                if bus.slow_reads.is_some() {
                    lcg = lcg
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    if (lcg >> 33) % 100 < SLOW_READS_PCT {
                        ms += SLOW_READ_MS;
                    }
                }
                // the region read, sampled at its end, then the re-read
                let lost = servo.ticks_lost_per_txn;
                servo.busy_ms(ms / 2.0, lost);
                pending = Some(servo.read());
                servo.busy_ms(ms / 2.0, lost);
            }
            Cmd::Pause { ms } => {
                let mut left = ms;
                while left > 0 {
                    let slice = lease.slice(left);
                    servo.advance_ms(slice as f64 * bus.pause_scale);
                    left -= slice;
                    keep(&mut lease, servo, &mut log);
                }
            }
            Cmd::Stream { samples, goal } => {
                // the goal and the arm go out held; the commit starts both
                let lost = 3.0 * servo.ticks_lost_per_txn;
                servo.busy_ms(2.0 * bus.write_ms, lost);
                match goal {
                    Some((reg, value)) => {
                        servo.write(reg, value);
                        lease.wrote(reg, value, servo.t_ms);
                        log.push(format!("stream {} {} {}", samples, reg_name(reg), value));
                    }
                    None => log.push(format!("stream {samples}")),
                }
                frames.clear();
                servo.stream(samples, &mut frames);
                exp.push_tel(&frames);
                keep(&mut lease, servo, &mut log);
            }
            Cmd::Burst {
                duty_q15,
                pre_q15,
                chans,
                seated,
            } => {
                let lost = BURST_ARM_TXNS * servo.ticks_lost_per_txn;
                servo.busy_ms(BURST_ARM_TXNS * bus.write_ms, lost);
                if servo.arm(duty_q15).is_err() {
                    let seen = servo.arm_seen();
                    log.push(format!("error: {}", rejected(duty_q15, &seen)));
                    for reg in [
                        control::GOAL_DUTY,
                        control::TORQUE_ENABLE,
                        control::STALL_PERMIT,
                        control::TEL_MASK,
                    ] {
                        charge_write(servo, &mut lease, &bus, reg, 0);
                        log.push(format!("write {} 0", reg_name(reg)));
                    }
                    return log;
                }
                let pos = servo.pos.round() as u16;
                log.push(format!(
                    "burst {duty_q15} pre {pre_q15} chans {chans} pos {pos}{}",
                    if seated { " seated" } else { "" }
                ));
                let plant = SynthBurst {
                    chans,
                    ..servo.burst.clone()
                };
                let mut cap = plant.capture(duty_q15, pre_q15);
                cap.meta.pos = pos;
                cap.meta.seated = seated;
                exp.push_burst(&cap);
                servo.bursts += 1;
                servo.advance(2);
                let lost = BURST_READBACK_TXNS * servo.ticks_lost_per_txn;
                servo.busy_ms(BURST_READBACK_TXNS * bus.read_ms / 2.0, lost);
                keep(&mut lease, servo, &mut log);
            }
            Cmd::Done => return log,
        }
    }
    log.push("OVERRUN".into());
    log
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shaft locked at mid travel and at a stop, 64% asked both ways:
    /// the duty climbs from the floor at the kernel's slew, stops rising
    /// once the current reaches the band under the limit, and the still
    /// shaft holds on the ceiling's step nearest the limit - inside the
    /// band, never exactly at it. A shaft that moves holds the band's
    /// bottom.
    #[test]
    fn fake_stall_holds_inside_the_band() {
        const LIM: u16 = 150;
        let band = (LIM - LIM / 8) as f64;
        for (jam, pos, sign) in [
            (Some(2400.0), 2400.0, 1),
            (Some(2400.0), 2400.0, -1),
            (None, 4000.0, 1),
            (None, 200.0, -1),
        ] {
            let mut s = FakeServo::new(3.37);
            s.current_limit = Some(LIM);
            s.jam = jam;
            s.pos = pos;
            s.tel_mask = (1 << 1) | (1 << 3);
            s.write(control::TORQUE_ENABLE, 1);
            s.write(control::GOAL_DUTY, sign * pct(64));
            assert_eq!(s.limit_flags(), 0, "a plain slew is not governed");
            let mut tel = Vec::new();
            s.stream(400, &mut tel);
            let duty: Vec<i16> = tel.iter().map(|f| f.duty_q15.unwrap()).collect();
            assert_eq!(duty[0], sign as i16 * (s.floor_q15 + 128), "first tick");
            assert!(duty.windows(2).all(|w| (w[1] - w[0]).abs() <= 128));
            assert!(
                tel.iter().all(|f| f.current.unwrap().unsigned_abs() <= LIM),
                "current over the limit"
            );
            s.advance(50);
            let o = s.read();
            let exact = (LIM as f64 * s.r / s.vbus * 32767.0) as i16;
            let under = exact - o.duty_applied_q15 * sign as i16;
            assert!(
                (1..=128).contains(&under),
                "{jam:?} {sign}: applied {} for {exact}",
                o.duty_applied_q15
            );
            let i = o.i_mean_counts.abs() as f64;
            assert!(i >= band && i < LIM as f64, "{jam:?} {sign}: {i}");
            assert_eq!(o.i_lim_counts, LIM);
            assert_eq!(o.pos, pos as u16);
            assert_eq!(s.limit_flags(), 1);
        }

        // the bench servo from rest at 64%: the climb draws the band's
        // bottom, within one step of the ceiling over it
        let mut s = bench_mg90(3204);
        s.pos = 1000.0;
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, pct(64));
        let step = 128.0 / 32767.0 * s.vbus / s.r;
        for _ in 0..8 {
            s.advance(5);
            let o = s.read();
            assert_eq!(o.limit_flags & 1, 1, "governed");
            let i = o.i_mean_counts as f64;
            assert!(i >= 245.0 && i < 245.0 + step, "climb at {i}");
        }
        assert!(s.pos > 1010.0, "the shaft climbs");
    }

    /// The bench fixture's stall settings fold: a locked shaft pinned for
    /// the stall time folds to the yield, and the fold holds while the
    /// disturbance reads over the release - a freed shaft the yield cannot
    /// start included - and releases once the drive stops, torque still on.
    /// The saved settings fold nothing: the verdict shows for one MEDIUM
    /// tick per trip, and the limit never moves.
    #[test]
    fn fake_fold_releases_like_the_kernel() {
        let mut s = bench_mg90(3204);
        s.pos = 2000.0;
        s.jam = Some(2000.0);
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, pct(64));
        s.advance(480);
        assert_eq!(s.limit_flags(), 1, "not yet");
        s.advance(40);
        assert_eq!(s.limit_flags(), 3);
        let o = s.read();
        assert_eq!(o.i_lim_counts, 168);
        assert!(o.i_mean_counts <= 168);
        s.advance(2000);
        assert_eq!(s.limit_flags() & 2, 2, "held while the stall holds");
        // freed, the shaft cannot break away on the yield: still held
        s.jam = None;
        s.advance(100);
        assert_eq!(s.limit_flags() & 2, 2);
        s.write(control::GOAL_DUTY, 0);
        s.advance(1);
        assert_eq!(s.limit_flags() & 2, 0, "released with the drive off");
        assert_eq!(s.read().i_lim_counts, 280);
        assert!(s.torque);
        s.write(control::GOAL_DUTY, pct(64));
        s.advance(100);
        assert_eq!(s.limit_flags() & 2, 0, "the free shaft runs, unfolded");
        assert!(s.pos > 2100.0);

        let mut s = bench_mg90_saved(3204);
        s.pos = 2000.0;
        s.jam = Some(2000.0);
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, pct(64));
        let (mut shown, mut trips, mut last) = (0, 0, false);
        for _ in 0..12_000 {
            s.advance_ms(0.25);
            let o = s.read();
            assert_eq!(o.i_lim_counts, 280);
            assert_eq!(o.limit_flags & 1, 1, "governed throughout");
            let now = o.limit_flags & 2 != 0;
            shown += now as u32;
            trips += (now && !last) as u32;
            last = now;
        }
        assert_eq!(trips, 5, "3 s at a trip every 570 ms");
        assert!(shown <= 2 * trips, "{shown} reads of 12000 saw it");
    }

    /// Reversed at speed, the limiter cannot apply less than its base -
    /// the window floor on the bench servo - so the winding draws the
    /// floor's volts plus the back-EMF, well over the limit.
    #[test]
    fn fake_reversal_cannot_go_under_the_base() {
        let mut s = bench_mg90(3204);
        s.pos = 1000.0;
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, pct(30));
        s.advance(150);
        let w = s.omega_dyn;
        assert!(w > 3000.0, "{w}");
        s.write(control::GOAL_DUTY, -pct(30));
        let o = s.read();
        assert_eq!(o.duty_applied_q15, -s.floor_q15);
        let want = (s.floor_q15 as f64 / 32767.0 * s.vbus + s.ke * w) / s.r;
        assert!(
            (o.i_mean_counts as f64 + want).abs() < 1.0,
            "{} for {want}",
            o.i_mean_counts
        );
        assert!(o.i_mean_counts < -350);
        assert_eq!(o.limit_flags & 1, 1);
    }

    /// Asked to, the fake refuses a burst arm the firmware would: over the
    /// volts cap, inside the spacing, or outside the soft limits without
    /// the permit. The pump then ends the run as the CLI does: the reason
    /// the host reads back, then the guard's writes.
    #[test]
    fn fake_arm_can_be_rejected() {
        let mut s = bench_mg90(3204);
        s.arm_rules = Some(ArmRules::FIRMWARE);
        s.pos = 2000.0;
        assert_eq!(s.arm(q15(25)), Err("torque off"));
        s.write(control::TORQUE_ENABLE, 1);
        assert_eq!(s.arm(q15(45)), Err("over the burst volts cap"));
        assert_eq!(s.arm(-q15(40)), Ok(()));
        s.advance(99);
        assert_eq!(s.arm(q15(25)), Err("inside the burst spacing"));
        s.advance(1);
        assert_eq!(s.arm(q15(25)), Ok(()));
        s.advance(100);
        s.pos = 3626.0;
        assert_eq!(
            s.arm(q15(25)),
            Err("outside the soft limits without the permit")
        );
        s.write(control::STALL_PERMIT, 1);
        assert_eq!(s.arm(q15(25)), Ok(()));

        let mut exp = Steps::new(vec![
            Cmd::Write {
                reg: control::TORQUE_ENABLE,
                value: 1,
            },
            Cmd::Burst {
                duty_q15: pct(45) as i16,
                pre_q15: 0,
                chans: 0,
                seated: false,
            },
            Cmd::Read,
        ]);
        let mut s = bench_mg90(3204);
        s.arm_rules = Some(ArmRules::FIRMWARE);
        let log = pump(&mut exp, &mut s, 100);
        assert_eq!(
            log,
            [
                "write torque_enable 1",
                "error: the servo refused the burst: 45.0% of its 7.90 V rail applies 3.553 V, \
                 over the 3.2 V a burst may",
                "write goal_duty 0",
                "write torque_enable 0",
                "write stall_permit 0",
                "write tel_mask 0",
            ]
        );
        assert!(exp.seen.is_empty(), "the run went no further");
        assert!(!s.torque);
    }

    /// Every transaction costs the bench bus's time and the plant moves
    /// through it: a 30 ms pause lands 39.5 ms from one read to the next, a
    /// rung's 2 ms poll 5.9 ms, and seeded slow reads add 8 ms to about 3
    /// in 100 - the same ones every time.
    #[test]
    fn fake_pump_charges_the_bus_time() {
        let script = || {
            Steps::new(vec![
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                },
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: pct(10),
                },
                Cmd::Read,
                Cmd::Pause { ms: 30 },
                Cmd::Read,
                Cmd::Pause { ms: 2 },
                Cmd::Read,
            ])
        };
        let mut exp = script();
        let mut s = FakeServo::new(3.37);
        pump(&mut exp, &mut s, 100);
        assert!((s.t_ms - (2.0 * 1.7 + 3.0 * 3.5 + 36.0 + 2.4)).abs() < 1e-9);
        let t: Vec<f64> = exp.seen.iter().map(|o| o.host_ms).collect();
        assert!((t[1] - t[0] - 39.5).abs() < 1e-9, "{t:?}");
        assert!((t[2] - t[1] - 5.9).abs() < 1e-9, "{t:?}");
        // 1 count/ms from the goal write's midpoint to the first sample
        assert_eq!(exp.seen[0].pos, 2403);

        let mut exp = script();
        let mut s = FakeServo::new(3.37);
        pump_on(&mut exp, &mut s, 100, Bus::ZERO_LATENCY);
        assert_eq!(s.t_ms, 32.0);
        assert_eq!(exp.seen[0].pos, 2400);

        let reads = |seed| {
            let mut exp = Steps::new(vec![Cmd::Read; 1000]);
            let mut s = FakeServo::new(3.37);
            pump_on(&mut exp, &mut s, 2000, Bus::BENCH.with_slow_reads(seed));
            s.t_ms
        };
        let slow = (reads(7) - 3500.0) / SLOW_READ_MS;
        assert!((10.0..=60.0).contains(&slow), "{slow} slow reads");
        assert_eq!(slow.fract(), 0.0);
        assert_eq!(reads(7), reads(7));
        assert_ne!(reads(7), reads(8));
    }

    /// Every read holds the kernel off for the ticks the bench board loses
    /// to a snapshot. Polled back to back at the bench ladder's 1.9 ms, the
    /// tick and window counters run at 0.87 of the host's clock; a pause
    /// loses nothing, and a TEL stream's ticks are the kernel's own.
    #[test]
    fn fake_servo_loses_ticks_to_bus_traffic() {
        let bus = Bus::BENCH.with_read_ms(1.9);
        let mut exp = Steps::new(vec![Cmd::Read; 501]);
        let mut s = FakeServo::new(3.37);
        pump_on(&mut exp, &mut s, 2000, bus);
        let (a, b) = (exp.seen[0], exp.seen[500]);
        let host = (b.host_ms - a.host_ms) * s.tick_hz() / 1000.0;
        let ticks = b.sample_tick.wrapping_sub(a.sample_tick) as f64;
        let per_read = (host - ticks) / 500.0;
        assert!(
            (per_read - 2.0 * TICKS_LOST_PER_TRANSACTION).abs() < 0.05,
            "{per_read} ticks lost a snapshot"
        );
        assert!((ticks / host - 0.869).abs() < 0.002, "{}", ticks / host);
        let windows = b.agg_seq.wrapping_sub(a.agg_seq) as f64;
        assert!((windows * TICKS_PER_WINDOW as f64 - ticks).abs() <= 16.0);

        let mut exp = Steps::new(vec![Cmd::Read, Cmd::Pause { ms: 1000 }, Cmd::Read]);
        let mut s = FakeServo::new(3.37);
        pump_on(&mut exp, &mut s, 100, bus);
        let (a, b) = (exp.seen[0], exp.seen[1]);
        assert_eq!(b.host_ms - a.host_ms, 1000.0 * bus.pause_scale + 1.9);
        let host = (b.host_ms - a.host_ms) * s.tick_hz() / 1000.0;
        let lost = host - b.sample_tick.wrapping_sub(a.sample_tick) as f64;
        assert!(
            (lost - 2.0 * TICKS_LOST_PER_TRANSACTION).abs() <= 1.0,
            "{lost} lost across a pause"
        );

        let before = s.read().sample_tick;
        s.tel_mask = 1;
        let mut tel = Vec::new();
        s.stream(3000, &mut tel);
        assert_eq!(tel.len(), 3000);
        assert_eq!(s.read().sample_tick - before, 3000, "a stream is tick-true");
    }

    /// The bench fixture runs as fast as the servo: from 20% to 64% its
    /// steady speed is within 3% of the lines measured on the bench MG90 at
    /// 7.9 V and at 4.39 V, the current limit in force.
    #[test]
    fn bench_fixture_follows_the_measured_speed_line() {
        for (vbus, slope, intercept) in [(3204u16, 0.2079, -0.831), (1780, 0.1299, -0.852)] {
            for d in (20..=64).step_by(4) {
                let mut s = bench_mg90(vbus);
                s.ends = (-1e9, 1e9);
                s.soft = None;
                s.pos = 0.0;
                s.write(control::TORQUE_ENABLE, 1);
                s.write(control::GOAL_DUTY, pct(d));
                s.advance(1500);
                let p0 = s.pos;
                s.advance(100);
                let v = (s.pos - p0) / 100.0;
                let want = slope * d as f64 + intercept;
                assert!(
                    (v / want - 1.0).abs() < 0.03,
                    "{vbus} at {d}%: {v:.2} counts/ms, the line {want:.2}"
                );
                assert_eq!(s.limit_flags(), 0, "{vbus} at {d}%: governed at speed");
            }
        }
    }

    /// A scripted run: one command per step, every snapshot kept.
    struct Steps {
        cmds: std::collections::VecDeque<Cmd>,
        seen: Vec<TelemetrySnapshot>,
    }

    impl Steps {
        fn new(cmds: Vec<Cmd>) -> Self {
            Self {
                cmds: cmds.into(),
                seen: Vec::new(),
            }
        }
    }

    impl Experiment for Steps {
        fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
            self.seen.extend(obs.copied());
            self.cmds.pop_front().unwrap_or(Cmd::Done)
        }
    }

    /// A locked shaft held at the limit for the stall time folds to the
    /// yield and says so; torque off clears it. A free shaft never folds.
    #[test]
    fn fake_stall_folds_to_the_yield() {
        let mut s = FakeServo::new(3.37);
        s.current_limit = Some(150);
        s.stall_ms = Some(500.0);
        s.stall_yield = 90;
        s.jam = Some(s.pos);
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, pct(64));
        s.advance(20);
        s.advance(460);
        assert_eq!(s.limit_flags(), 1, "not yet");
        s.advance(40);
        assert_eq!(s.limit_flags(), 3);
        let o = s.read();
        assert_eq!(o.i_lim_counts, 90);
        assert!(o.i_mean_counts.abs() <= 90);
        s.write(control::TORQUE_ENABLE, 0);
        assert_eq!(s.limit_flags() & 2, 0);

        let mut s = FakeServo::new(3.37);
        s.current_limit = Some(150);
        s.stall_ms = Some(500.0);
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, pct(10));
        s.advance(1000);
        assert_eq!(s.limit_flags(), 0);
    }

    /// A goal the limit can hold passes, and without a limit nothing is
    /// governed.
    #[test]
    fn fake_passes_what_the_limit_holds() {
        let mut s = FakeServo::new(3.37);
        s.current_limit = Some(150);
        s.jam = Some(s.pos);
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, pct(10));
        s.advance(50);
        assert_eq!(s.read().duty_applied_q15, pct(10) as i16);
        assert_eq!(s.limit_flags(), 0);
        s.current_limit = None;
        s.write(control::GOAL_DUTY, pct(64));
        assert_eq!(s.read().duty_applied_q15, pct(64) as i16);
    }

    /// The lease: a write of true with torque on grants it, a rewrite
    /// extends it, and the grant lapses a lease after the last write, on
    /// torque off, or never starts with torque off.
    #[test]
    fn fake_permit_expires() {
        let mut s = FakeServo::new(3.37);
        s.lease_ms = Some(1000.0);
        s.ends = (230.0, 4000.0);
        s.soft = Some((230.0, 3970.0));
        s.pos = 230.0;
        let outward = -pct(10);
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, outward);
        assert_eq!(
            s.read().duty_applied_q15,
            0,
            "no permit: the endstop blocks"
        );
        assert_eq!(s.limit_flags(), 4);

        s.write(control::STALL_PERMIT, 1);
        assert_eq!(s.read().duty_applied_q15, outward as i16);
        assert_eq!(s.limit_flags(), 8);
        s.advance(900);
        s.write(control::STALL_PERMIT, 1);
        s.advance(900);
        assert!(s.permit_live(), "a rewrite extends the lease");
        s.advance(200);
        assert!(!s.permit_live());
        assert_eq!(s.read().duty_applied_q15, 0, "the lease ran out");
        assert_eq!(s.limit_flags(), 4);
        assert!(s.permit, "the request byte still reads true");

        s.write(control::STALL_PERMIT, 1);
        assert!(s.permit_live());
        s.write(control::TORQUE_ENABLE, 0);
        s.write(control::TORQUE_ENABLE, 1);
        assert!(!s.permit_live(), "torque off drops the grant");
        s.write(control::TORQUE_ENABLE, 0);
        s.write(control::STALL_PERMIT, 1);
        s.write(control::TORQUE_ENABLE, 1);
        assert!(
            !s.permit_live(),
            "a permit written with torque off never grants"
        );

        s.lease_ms = None;
        s.advance(5000);
        assert!(s.permit_live(), "the level permit holds while written true");
    }

    fn pct(p: i32) -> i32 {
        p * 32767 / 100
    }

    fn q15(p: i32) -> i16 {
        pct(p) as i16
    }
}

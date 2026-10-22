//! i_lim composition (control-theory "Limits"): the current clamp is the one
//! choke point every command passes through, so all protection lives here,
//! each term owning one physical quantity. i_lim = min of the peak cap
//! (`current_limit_counts`, the nameplate stall current), the I2t budget's
//! ceiling (copper, a one-node winding model on the held current), the
//! thermometer's derate (copper, measured), the rail fold (a sagging
//! source) and the application's `goal_current`; the press fold to
//! `i_cont_counts` and the directional endstop come after. `fold` runs at
//! MEDIUM and is compare/min only; the thermal ceilings are recomputed at
//! the SLOW cadence and cached, the rail's at RAIL.

use crate::kernel::{DECIM_MED, DECIM_SLOW, TICK_HZ};
use crate::math::recip_div;
use crate::regions::config::{ConfigLimits, ConfigPosSoft, ConfigThermal, StallResponse};

/// `limit_flags` bits: what shaped the command. A term bit names a term
/// whose ceiling sits under the cap and is the composed `i_lim_counts`.
pub mod flag {
    /// The command sat at the current ceiling: OpenLoop's duty ceiling held
    /// under the goal, or a closed loop's current reference at the band.
    pub const CEILING: u8 = 1 << 0;
    /// The press or collision verdict folded the limit to `i_cont_counts`.
    pub const YIELD: u8 = 1 << 1;
    /// A soft limit closed one side of the band.
    pub const ENDSTOP: u8 = 1 << 2;
    /// The stall permit lease is live.
    pub const PERMIT: u8 = 1 << 3;
    /// The I2t budget's ceiling binds.
    pub const I2T: u8 = 1 << 4;
    /// The winding thermometer's derate binds.
    pub const DERATE: u8 = 1 << 5;
    /// The rail fold binds.
    pub const RAIL: u8 = 1 << 6;
    /// `goal_current` binds.
    pub const GOAL: u8 = 1 << 7;
}

/// The SLOW period, the I2t update's step.
pub const SLOW_PERIOD_MS: u32 = 1000 * DECIM_MED as u32 * DECIM_SLOW as u32 / TICK_HZ;
const _: () = assert!((1000 * DECIM_MED as u32 * DECIM_SLOW as u32).is_multiple_of(TICK_HZ));

/// The longest leak: 2^12 SLOW periods, 65.5 s.
pub const TAU_SHIFT_MAX: u32 = 12;
/// Heat state guard bits: the leak's quantum at the longest leak is a
/// sixteenth of a count squared.
const HEAT_GUARD: u32 = 4;
/// The heat the state can hold: 12-bit shunt codes under a settle gain
/// under 2.0 square to under this, and the guard keeps it inside i32.
const HEAT_MAX: u32 = (i32::MAX as u32) >> HEAT_GUARD;
/// The fold's fraction width: the budget left, under the ramp, is brought
/// to this many bits before the one u32 multiply.
const RAMP_Q: u32 = 16;
/// The rail fold's Q.
const RAIL_SHIFT: u32 = 8;

/// `floor(log2(x))`, 0 at 0: a branchless five-step search, out of line,
/// since the chip has no count-leading-zeros and LLVM's inline sequence
/// is ~40 instructions per site.
#[inline(never)]
#[rustc_align(4)]
fn floor_log2(mut x: u32) -> u32 {
    let mut k = 0;
    let mut s = 16;
    while s != 0 {
        let step = (x >> s != 0) as u32 * s;
        x >>= step;
        k += step;
        s >>= 1;
    }
    k
}

/// The leak shift for `thermal_tau_ms`: the nearest power of two of SLOW
/// periods, so the EWMA's time constant lands within an octave of the
/// winding's, 0 (one period) for anything under two.
pub fn tau_shift(tau_ms: u16) -> u8 {
    let periods = tau_ms as u32 / SLOW_PERIOD_MS;
    if periods < 2 {
        return 0;
    }
    let mut k = floor_log2(periods);
    if periods - (1 << k) > (1 << k) >> 1 {
        k += 1;
    }
    k.min(TAU_SHIFT_MAX) as u8
}

/// `i_cont_counts` as the kernel runs it: 0 reads as the cap (no budget,
/// the press at the cap). The configuration take writes this into the
/// kernel's own copy of the block, so nothing on the tick path tests it.
pub fn i_cont_or_cap(cfg: &ConfigLimits) -> u16 {
    if cfg.i_cont_counts == 0 {
        cfg.current_limit_counts
    } else {
        cfg.i_cont_counts
    }
}

/// `floor(log2(i_cont))`, the octave the I2t fold's ramp comes from,
/// derived at the configuration take; at most 15.
pub fn i_cont_log2(i_cont: u16) -> u8 {
    floor_log2(i_cont as u32) as u8
}

/// The two I2t shifts packed in one byte (`KernelConfig::i2t_shifts`): the
/// leak in the high nibble, `i_cont`'s octave in the low.
pub const fn pack_shifts(tau_shift: u8, i_cont_log2: u8) -> u8 {
    tau_shift << 4 | i_cont_log2
}
const _: () = assert!(TAU_SHIFT_MAX < 16);

/// The signed current band every command clamps into: `lo <= i_ref <= hi`.
/// Symmetric `+-i_lim` away from the walls; a soft-limit wall collapses its
/// inward side to 0 while the outward side keeps the composed limit.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct IBand {
    pub lo: i32,
    pub hi: i32,
}

impl IBand {
    pub fn clamp(&self, i: i32) -> i32 {
        i.max(self.lo).min(self.hi)
    }

    /// The composed magnitude the band was built from (every term and the
    /// press fold; the endstop is positional and lives in the sides). This
    /// is what telemetry publishes as `i_lim_counts`. Each side is zero
    /// or that magnitude, so OR recovers it without a branch.
    pub fn lim(&self) -> u16 {
        self.hi as u16 | (-self.lo) as u16
    }
}

/// The MEDIUM-rate inputs of one `fold`.
#[derive(Copy, Clone, Default)]
pub struct FoldIn {
    /// The held measured current, magnitude.
    pub i_held_abs: u32,
    pub omega_hat_abs_cps: u32,
    pub tau_d_abs_counts: u16,
    pub theta_hat_counts: i32,
    pub stall_permit: bool,
    /// `goal_current`'s magnitude in the closed loops, 0 for no request;
    /// the caller passes 0 in Current mode, where the goal is the
    /// reference, and in OpenLoop, whose knob is the duty.
    pub goal_abs: u16,
}

/// One `fold`'s verdict: the band and the term bits (`flag`) of the
/// ceiling that binds.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Fold {
    pub band: IBand,
    pub terms: u8,
}

/// Stall timer + fold flag, the heat state and the cached ceilings. The
/// caches boot non-binding (u16::MAX) so a fresh state is not zero-current
/// for the first SLOW period; the base min still caps them at the cap.
pub struct LimitState {
    stall_ticks: u32,
    /// EWMA of the held current squared, counts^2 with `HEAT_GUARD` bits.
    heat_qg: i32,
    i2t_cache: u16,
    derate_cache: u16,
    rail_cache: u16,
    stalled: bool,
    fault_pending: bool,
}

impl LimitState {
    pub const fn new() -> Self {
        Self {
            stall_ticks: 0,
            heat_qg: 0,
            i2t_cache: u16::MAX,
            derate_cache: u16::MAX,
            rail_cache: u16::MAX,
            stalled: false,
            fault_pending: false,
        }
    }

    /// SLOW-rate: one step of the heat, `heat += (i^2 - heat) >> tau_shift`,
    /// on the held current (0 with nothing driving). One multiply.
    pub fn heat(&mut self, i_held: i16, tau_shift: u8) {
        let i = i_held.unsigned_abs() as u32;
        let x_qg = ((i * i).min(HEAT_MAX) << HEAT_GUARD) as i32;
        self.heat_qg += (x_qg - self.heat_qg) >> tau_shift;
    }

    /// SLOW-rate: the I2t ceiling. The budget is `i_cont^2`; the cap while
    /// the heat sits under the knee, then linear to `i_cont` at the budget,
    /// where a hold at `i_cont` keeps the heat (first order, monotone, no
    /// overshoot while the leak is slower than a few periods). The ramp is
    /// a power of two from `i_cont`'s own octave `i_cont_log2`, between an
    /// eighth and a half of the budget, so the ceiling never chatters at
    /// the knee.
    pub fn update_i2t(&mut self, cfg: &ConfigLimits, i_cont_log2: u8) {
        let (cap, i_cont) = (cfg.current_limit_counts, cfg.i_cont_counts);
        let budget = i_cont as u32 * i_cont as u32;
        // 2^(2k) <= budget < 2^(2k + 2): the ramp 2^(2k - 1) sits in
        // [budget / 8, budget / 2]
        let ramp_shift = (2 * i_cont_log2 as u32).saturating_sub(1);
        let heat = (self.heat_qg >> HEAT_GUARD) as u32;
        // the budget left, at most the ramp (a whole ramp is the cap);
        // brought under 2^RAMP_Q, its product with the fold's span fits u32
        let left =
            budget.saturating_sub(heat).min(1 << ramp_shift) >> ramp_shift.saturating_sub(RAMP_Q);
        let over = (cap as u32).saturating_sub(i_cont as u32);
        self.i2t_cache = (i_cont as u32 + ((over * left) >> ramp_shift.min(RAMP_Q))) as u16;
    }

    /// SLOW-rate: the thermometer's derate ceiling. Foldback: full below
    /// derate_start_cc, zero at cutoff_cc, linear between - self-regulating
    /// (less current -> less heating -> equilibrium at "weaker but still
    /// holding"). A degenerate span (cutoff <= start, no table cross-gate)
    /// degrades to a step at cutoff. At 62.5 Hz the reciprocal's ~10
    /// multiplies are noise.
    pub fn update_derate(&mut self, t_winding_cc: i16, cfg: &ConfigLimits, th: &ConfigThermal) {
        let span = th.cutoff_cc as i32 - th.derate_start_cc as i32;
        self.derate_cache = if t_winding_cc >= th.cutoff_cc {
            0
        } else if t_winding_cc < th.derate_start_cc || span <= 0 {
            u16::MAX
        } else {
            // head in [1, span], both <= 65535: num fits u32 exactly
            let head = (th.cutoff_cc as i32 - t_winding_cc as i32) as u32;
            let num = cfg.current_limit_counts as u32 * head;
            recip_div(num, span as u32).min(cfg.current_limit_counts as u32) as u16
        };
    }

    /// RAIL-rate: the rail fold, `slope_q8` counts per vcount over the
    /// undervolt floor (`rail_fold_q8`; 0 = off), zero at the floor. One
    /// multiply; two u16 factors never overflow.
    pub fn update_rail(&mut self, vbus_counts: u16, floor: u16, slope_q8: u16) {
        let v = vbus_counts.saturating_sub(floor) as u32;
        self.rail_cache = if slope_q8 == 0 {
            u16::MAX
        } else {
            ((v * slope_q8 as u32) >> RAIL_SHIFT).min(u16::MAX as u32) as u16
        };
    }

    // Select the flags with each minimum so the detector path need not
    // keep all four ceilings live. The packed result fits one register.
    #[inline(always)]
    fn ceiling(&self, cap: u16, goal_abs: u16) -> u32 {
        let mut lim = cap;
        let mut terms = 0;
        for (ceiling, bit) in [
            (self.i2t_cache, flag::I2T),
            (self.derate_cache, flag::DERATE),
            (self.rail_cache, flag::RAIL),
            (if goal_abs == 0 { u16::MAX } else { goal_abs }, flag::GOAL),
        ] {
            if ceiling < lim {
                lim = ceiling;
                terms = bit;
            } else if ceiling == lim && lim < cap {
                terms |= bit;
            }
        }
        lim as u32 | ((terms as u32) << 16)
    }

    /// One MEDIUM-tick composition -> the signed current band `[lo, hi]`
    /// every command clamps into, and the term that binds. Compare/min
    /// only, no multiplies.
    ///
    /// The endstop is a DIRECTIONAL bound, not a magnitude fold: at a soft
    /// limit only the inward side collapses to 0 and retreat keeps the
    /// composed limit. Folding a magnitude gated by the command's sign
    /// deadlocks at the wall - the clamp zeroes i_ref, zero reads as an
    /// inward push, and the door never reopens (caught on the bench).
    ///
    /// Press: the held current over `i_cont` with the pot observer's
    /// |omega_hat| under stall_omega_max for stall_time_ticks, so a jam at
    /// any duty over the continuous rating trips and the cap never moves
    /// the trip point; or a tau_d spike past stall_tau_trip_counts
    /// (collision). Yield folds to `i_cont` until the held current reads
    /// under stall_release_counts: the current, not tau_d, because tau_d
    /// is held at zero on an unidentified servo and would unfold at once.
    /// Fault only pends - latching is kernel fault policy, and the kernel
    /// disables on latch, so folding here too would double-act.
    ///
    /// `stall_permit` is the deliberate-stall escape hatch, the kernel's live
    /// lease (`ControlLifecycle::stall_permit`): it lifts the detectors and
    /// the endstop, never a ceiling.
    pub fn fold(&mut self, inp: &FoldIn, cfg: &ConfigLimits, soft: &ConfigPosSoft) -> Fold {
        if inp.stall_permit {
            self.stalled = false;
            self.stall_ticks = 0;
        } else {
            // release before the triggers: a relax re-arms a full stall
            // window instead of instantly re-tripping off the stale timer
            if self.stalled && inp.i_held_abs < cfg.stall_release_counts as u32 {
                self.stalled = false;
                self.stall_ticks = 0;
            }
            let mut trip = inp.tau_d_abs_counts > cfg.stall_tau_trip_counts;
            if inp.i_held_abs > cfg.i_cont_counts as u32
                && inp.omega_hat_abs_cps < cfg.stall_omega_max_cps as u32
            {
                self.stall_ticks = self.stall_ticks.saturating_add(DECIM_MED as u32);
                trip |= self.stall_ticks >= cfg.stall_time_ticks;
            } else {
                self.stall_ticks = 0;
            }
            if trip {
                match cfg.stall_response {
                    StallResponse::Yield => self.stalled = true,
                    StallResponse::Fault => self.fault_pending = true,
                }
            }
        }
        let ceiling = self.ceiling(cfg.current_limit_counts, inp.goal_abs);
        let mut lim = ceiling as u16;
        let terms = (ceiling >> 16) as u8;
        if self.stalled {
            lim = lim.min(cfg.i_cont_counts);
        }
        // endstop: positive current moves counts up (drive polarity is
        // applied at the motor output), so the max wall zeroes hi and the
        // min wall zeroes lo; the permit lifts both walls
        let mut band = IBand {
            lo: -(lim as i32),
            hi: lim as i32,
        };
        if !inp.stall_permit {
            if inp.theta_hat_counts >= soft.pos_max_soft_counts {
                band.hi = 0;
            }
            if inp.theta_hat_counts <= soft.pos_min_soft_counts {
                band.lo = 0;
            }
        }
        Fold { band, terms }
    }

    /// Torque-enable ack: drop the pending fault, unfold, re-arm a full
    /// stall window. The heat and the caches survive - they are thermal
    /// and rail state, not latches, and clearing them would hand a hot
    /// winding one SLOW period of full current.
    pub fn ack(&mut self) {
        self.fault_pending = false;
        self.stalled = false;
        self.stall_ticks = 0;
    }

    pub fn stalled(&self) -> bool {
        self.stalled
    }

    pub fn stall_fault_pending(&self) -> bool {
        self.fault_pending
    }
}

impl Default for LimitState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fields the block reads, flat; the helpers hand them over as the
    /// table's blocks, `i_cont_counts` as the take writes it.
    #[derive(Copy, Clone)]
    struct LimitCfg {
        current_limit_counts: u16,
        stall_response: StallResponse,
        stall_omega_max_cps: u16,
        stall_time_ticks: u32,
        i_cont_counts: u16,
        stall_release_counts: u16,
        stall_tau_trip_counts: u16,
        thermal_tau_ms: u16,
        derate_start_cc: i16,
        cutoff_cc: i16,
        v_undervolt_counts: u16,
        rail_fold_q8: u16,
        pos_min_soft_counts: i32,
        pos_max_soft_counts: i32,
    }

    impl LimitCfg {
        fn blocks(&self) -> (ConfigLimits, ConfigThermal, ConfigPosSoft) {
            let mut limits = ConfigLimits {
                current_limit_counts: self.current_limit_counts,
                stall_response: self.stall_response,
                stall_omega_max_cps: self.stall_omega_max_cps,
                stall_time_ticks: self.stall_time_ticks,
                i_cont_counts: self.i_cont_counts,
                stall_release_counts: self.stall_release_counts,
                stall_tau_trip_counts: self.stall_tau_trip_counts,
                thermal_tau_ms: self.thermal_tau_ms,
                ..ConfigLimits::new()
            };
            limits.i_cont_counts = i_cont_or_cap(&limits);
            let thermal = ConfigThermal {
                derate_start_cc: self.derate_start_cc,
                cutoff_cc: self.cutoff_cc,
                v_undervolt_counts: self.v_undervolt_counts,
                ..ConfigThermal::new()
            };
            let soft = ConfigPosSoft {
                pos_min_soft_counts: self.pos_min_soft_counts,
                pos_max_soft_counts: self.pos_max_soft_counts,
            };
            (limits, thermal, soft)
        }
    }

    /// `fold` on a still shaft at `i_held`, mid-range, no load.
    fn fold_at(st: &mut LimitState, i_held: u32, omega: u32, cfg: &LimitCfg) -> IBand {
        fold(
            st,
            &FoldIn {
                i_held_abs: i_held,
                omega_hat_abs_cps: omega,
                ..Default::default()
            },
            cfg,
        )
        .band
    }

    fn fold(st: &mut LimitState, inp: &FoldIn, cfg: &LimitCfg) -> Fold {
        let (limits, _, soft) = cfg.blocks();
        st.fold(inp, &limits, &soft)
    }

    fn derate(st: &mut LimitState, t_winding_cc: i16, cfg: &LimitCfg) {
        let (limits, th, _) = cfg.blocks();
        st.update_derate(t_winding_cc, &limits, &th);
    }

    fn rail(st: &mut LimitState, vbus: u16, cfg: &LimitCfg) {
        st.update_rail(vbus, cfg.v_undervolt_counts, cfg.rail_fold_q8);
    }

    /// One SLOW period at `i_held`: the heat step on the due tick and the
    /// I2t ceiling at the finish.
    fn slow_period(st: &mut LimitState, i_held: i16, cfg: &LimitCfg) {
        let (limits, _, _) = cfg.blocks();
        st.heat(i_held, tau_shift(limits.thermal_tau_ms));
        st.update_i2t(&limits, i_cont_log2(limits.i_cont_counts));
    }

    const CFG: LimitCfg = LimitCfg {
        current_limit_counts: 1200,
        stall_response: StallResponse::Yield,
        stall_omega_max_cps: 500,
        stall_time_ticks: 100,
        i_cont_counts: 300,
        stall_release_counts: 150,
        stall_tau_trip_counts: 1200,
        thermal_tau_ms: 2048,
        derate_start_cc: 8000,
        cutoff_cc: 10000,
        v_undervolt_counts: 700,
        // 6 counts per vcount: the cap 200 vcounts over the floor
        rail_fold_q8: 6 << 8,
        pos_min_soft_counts: -4000,
        pos_max_soft_counts: 4000,
    };

    /// Moving, no load: nothing but the ceilings can bind.
    fn free(st: &mut LimitState, cfg: &LimitCfg) -> Fold {
        fold(
            st,
            &FoldIn {
                omega_hat_abs_cps: 1000,
                ..Default::default()
            },
            cfg,
        )
    }

    fn free_fold(st: &mut LimitState, cfg: &LimitCfg) -> u16 {
        free(st, cfg).band.hi as u16
    }

    #[test]
    fn cold_state_current_limit_binds() {
        let mut st = LimitState::new();
        let f = free(&mut st, &CFG);
        assert_eq!(f.band.hi, 1200);
        assert_eq!(f.band.lim(), 1200);
        assert_eq!(f.terms, 0);
        assert!(!st.stalled());
        assert!(!st.stall_fault_pending());
    }

    #[test]
    fn shifts_pack_and_the_octave_is_the_floor() {
        assert_eq!(i_cont_log2(0), 0);
        assert_eq!(i_cont_log2(1), 0);
        assert_eq!(i_cont_log2(300), 8);
        assert_eq!(i_cont_log2(u16::MAX), 15);
        let s = pack_shifts(12, 15);
        assert_eq!((s >> 4, s & 0xF), (12, 15));
    }

    #[test]
    fn tau_shift_rounds_to_the_nearest_octave() {
        assert_eq!(SLOW_PERIOD_MS, 16);
        assert_eq!(tau_shift(0), 0);
        assert_eq!(tau_shift(16), 0);
        assert_eq!(tau_shift(32), 1);
        assert_eq!(tau_shift(2048), 7);
        // 4000 ms is 250 periods, nearer 256 than 128
        assert_eq!(tau_shift(4000), 8);
        assert_eq!(tau_shift(4096), 8);
        assert_eq!(tau_shift(u16::MAX), TAU_SHIFT_MAX as u8);
    }

    #[test]
    fn derate_endpoints_and_midpoint() {
        let mut st = LimitState::new();
        derate(&mut st, 7999, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 1200);
        // onset: head == span, quotient = current_limit up to truncation
        derate(&mut st, 8000, &CFG);
        assert!(free_fold(&mut st, &CFG) >= 1199);
        // midpoint -> half, and the derate names itself
        derate(&mut st, 9000, &CFG);
        let f = free(&mut st, &CFG);
        assert!((599..=600).contains(&f.band.hi), "mid={}", f.band.hi);
        assert_eq!(f.terms, flag::DERATE);
        // cutoff and beyond -> zero
        derate(&mut st, 10000, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 0);
        derate(&mut st, 12000, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 0);
        // recovery is just the next SLOW recompute
        derate(&mut st, 2500, &CFG);
        let f = free(&mut st, &CFG);
        assert_eq!((f.band.hi, f.terms), (1200, 0));
    }

    #[test]
    fn derate_degenerate_span_steps_at_cutoff() {
        let cfg = LimitCfg {
            derate_start_cc: 10000,
            ..CFG
        };
        let mut st = LimitState::new();
        derate(&mut st, 9999, &cfg);
        assert_eq!(free_fold(&mut st, &cfg), 1200);
        derate(&mut st, 10000, &cfg);
        assert_eq!(free_fold(&mut st, &cfg), 0);
    }

    /// a jam at 40% of the cap but over i_cont trips after
    /// stall_time_ticks; the same jam under i_cont never does.
    #[test]
    fn press_over_i_cont_trips_under_it_never() {
        let mut st = LimitState::new();
        for _ in 0..9 {
            assert_eq!(fold_at(&mut st, 480, 0, &CFG).hi, 1200);
            assert!(!st.stalled());
        }
        let b = fold_at(&mut st, 480, 0, &CFG);
        assert_eq!((b.hi, b.lim()), (300, 300));
        assert!(st.stalled());
        assert!(!st.stall_fault_pending());
        let mut st = LimitState::new();
        for _ in 0..1000 {
            assert_eq!(fold_at(&mut st, 299, 0, &CFG).hi, 1200);
        }
        assert!(!st.stalled());
        // at i_cont is not over it
        for _ in 0..1000 {
            assert_eq!(fold_at(&mut st, 300, 0, &CFG).hi, 1200);
        }
        assert!(!st.stalled());
    }

    /// the cap moves the ceiling, never the trip point.
    #[test]
    fn raising_the_cap_does_not_move_the_trip() {
        for cap in [554u16, 1108, 2000] {
            let cfg = LimitCfg {
                current_limit_counts: cap,
                ..CFG
            };
            let mut st = LimitState::new();
            for _ in 0..9 {
                assert_eq!(fold_at(&mut st, 301, 0, &cfg).hi, cap as i32);
            }
            assert_eq!(fold_at(&mut st, 301, 0, &cfg).hi, 300, "cap {cap}");
            let mut st = LimitState::new();
            for _ in 0..100 {
                assert_eq!(fold_at(&mut st, 300, 0, &cfg).hi, cap as i32);
            }
        }
    }

    #[test]
    fn movement_resets_timer() {
        let mut st = LimitState::new();
        for _ in 0..9 {
            fold_at(&mut st, 480, 0, &CFG);
        }
        // one fast tick clears the count; so does one under i_cont
        fold_at(&mut st, 480, 500, &CFG);
        for _ in 0..9 {
            assert_eq!(fold_at(&mut st, 480, 0, &CFG).hi, 1200);
        }
        fold_at(&mut st, 200, 0, &CFG);
        for _ in 0..9 {
            assert_eq!(fold_at(&mut st, 480, 0, &CFG).hi, 1200);
        }
        assert_eq!(fold_at(&mut st, 480, 0, &CFG).hi, 300);
    }

    /// The fold holds while the held current sits at the fold (an OpenLoop
    /// hold sits in `[7/8 i_cont, i_cont]`), lets go under the release,
    /// and a fresh window is required after.
    #[test]
    fn release_on_current_under_release_restores_and_rearms() {
        let mut st = LimitState::new();
        for _ in 0..10 {
            fold_at(&mut st, 480, 0, &CFG);
        }
        assert!(st.stalled());
        assert_eq!(fold_at(&mut st, 263, 0, &CFG).hi, 300);
        assert_eq!(fold_at(&mut st, 150, 1000, &CFG).hi, 300);
        assert_eq!(fold_at(&mut st, 149, 1000, &CFG).hi, 1200);
        assert!(!st.stalled());
        for _ in 0..9 {
            assert_eq!(fold_at(&mut st, 480, 0, &CFG).hi, 1200);
        }
        assert_eq!(fold_at(&mut st, 480, 0, &CFG).hi, 300);
    }

    #[test]
    fn collision_folds_immediately() {
        let mut st = LimitState::new();
        // under i_cont and moving: only the tau_d spike path can trip
        let hit = FoldIn {
            i_held_abs: 100,
            omega_hat_abs_cps: 2000,
            tau_d_abs_counts: 1201,
            ..Default::default()
        };
        let f = fold(&mut st, &hit, &CFG);
        assert_eq!(f.band.hi, 300);
        assert!(st.stalled());
        assert_eq!(f.terms, 0, "the fold is the verdict, not a term");
        // at the threshold is not a spike
        let mut st = LimitState::new();
        let touch = FoldIn {
            tau_d_abs_counts: 1200,
            ..hit
        };
        assert_eq!(fold(&mut st, &touch, &CFG).band.hi, 1200);
        assert!(!st.stalled());
        // the trip off (the boot value) never fires
        let cfg = LimitCfg {
            stall_tau_trip_counts: u16::MAX,
            ..CFG
        };
        let mut st = LimitState::new();
        let spike = FoldIn {
            tau_d_abs_counts: u16::MAX,
            ..hit
        };
        assert_eq!(fold(&mut st, &spike, &cfg).band.hi, 1200);
        assert!(!st.stalled());
    }

    #[test]
    fn fault_mode_pends_without_folding() {
        let cfg = LimitCfg {
            stall_response: StallResponse::Fault,
            ..CFG
        };
        let mut st = LimitState::new();
        for _ in 0..10 {
            assert_eq!(fold_at(&mut st, 480, 0, &cfg).hi, 1200);
        }
        assert!(st.stall_fault_pending());
        assert!(!st.stalled());
        // collision pends too, still no fold
        let mut st = LimitState::new();
        let hit = FoldIn {
            omega_hat_abs_cps: 2000,
            tau_d_abs_counts: 1201,
            ..Default::default()
        };
        assert_eq!(fold(&mut st, &hit, &cfg).band.hi, 1200);
        assert!(st.stall_fault_pending());
        assert!(!st.stalled());
    }

    /// a step to the cap over i_cont runs on the cap for the
    /// budget, then the ceiling folds toward i_cont without a reversal; at
    /// rest the budget refills and the ceiling recovers the same way.
    #[test]
    fn i2t_burst_folds_monotone_and_recovers() {
        let cfg = LimitCfg {
            thermal_tau_ms: 1024,
            ..CFG
        };
        let mut st = LimitState::new();
        let mut trace = [0u16; 600];
        for l in trace.iter_mut() {
            slow_period(&mut st, 1200, &cfg);
            *l = free_fold(&mut st, &cfg);
        }
        // the knee: 1200^2 reaches 300^2 - 2^15 = 57232 in ~2.7 periods
        assert!(trace[..2].iter().all(|&l| l == 1200), "{:?}", &trace[..4]);
        assert!(trace[8] < 1200, "{:?}", &trace[..10]);
        assert!(trace.windows(2).all(|w| w[1] <= w[0]), "monotone fold");
        assert_eq!(trace[599], 300, "folds to i_cont, never under");
        assert_eq!(free(&mut st, &cfg).terms, flag::I2T);
        // a hold at i_cont keeps the heat at the budget: the ceiling stays
        for _ in 0..600 {
            slow_period(&mut st, 300, &cfg);
            assert_eq!(free_fold(&mut st, &cfg), 300);
        }
        // at rest the budget refills: a monotone recovery to the cap
        for l in trace.iter_mut() {
            slow_period(&mut st, 0, &cfg);
            *l = free_fold(&mut st, &cfg);
        }
        assert!(trace.windows(2).all(|w| w[1] >= w[0]), "monotone recovery");
        assert_eq!(trace[599], 1200);
        assert_eq!(free(&mut st, &cfg).terms, 0);
    }

    /// The budget's duration at the cap scales with the leak: a 4 s
    /// constant holds the cap four times as long as a 1 s one.
    #[test]
    fn i2t_extreme_currents_keep_the_ceiling_in_range() {
        for cap in [0, 1, 255, 32768, u16::MAX] {
            for rating in [0, 1, 256, 8192, u16::MAX] {
                let cfg = LimitCfg {
                    current_limit_counts: cap,
                    i_cont_counts: rating,
                    thermal_tau_ms: 16,
                    ..CFG
                };
                let mut st = LimitState::new();
                for held in [i16::MIN, i16::MAX, 0] {
                    slow_period(&mut st, held, &cfg);
                    let limit = free_fold(&mut st, &cfg);
                    assert!(limit <= cap, "cap={cap} rating={rating} held={held}");
                    assert!(limit >= i_cont_or_cap(&cfg.blocks().0).min(cap));
                    assert!(st.heat_qg >= 0);
                }
                assert_eq!(free_fold(&mut st, &cfg), cap, "cold recovery");
            }
        }
    }

    #[test]
    fn i2t_budget_scales_with_tau() {
        let at_cap = |tau_ms: u16| {
            let cfg = LimitCfg {
                thermal_tau_ms: tau_ms,
                ..CFG
            };
            let mut st = LimitState::new();
            (0..2000)
                .take_while(|_| {
                    slow_period(&mut st, 1200, &cfg);
                    free_fold(&mut st, &cfg) == 1200
                })
                .count()
        };
        let (short, long) = (at_cap(1024), at_cap(4096));
        assert!(short >= 2 && long >= 4 * short - 1, "{short} vs {long}");
    }

    /// i_cont 0 reads as the cap: no budget, the press at the cap.
    #[test]
    fn i_cont_zero_is_the_cap() {
        let cfg = LimitCfg {
            i_cont_counts: 0,
            ..CFG
        };
        assert_eq!(cfg.blocks().0.i_cont_counts, 1200);
        let mut st = LimitState::new();
        for _ in 0..2000 {
            slow_period(&mut st, 1200, &cfg);
            let f = free(&mut st, &cfg);
            assert_eq!((f.band.hi, f.terms), (1200, 0));
        }
        for _ in 0..100 {
            assert_eq!(fold_at(&mut st, 1200, 0, &cfg).hi, 1200);
        }
        assert!(!st.stalled());
        for _ in 0..10 {
            fold_at(&mut st, 1201, 0, &cfg);
        }
        assert!(st.stalled());
    }

    /// the rail fold narrows as vbus enters the band and releases
    /// on recovery; the fault at the floor is the slow block's.
    #[test]
    fn rail_fold_narrows_in_the_band_and_releases() {
        let mut st = LimitState::new();
        rail(&mut st, 3000, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 1200);
        rail(&mut st, 900, &CFG);
        let f = free(&mut st, &CFG);
        assert_eq!((f.band.hi, f.terms), (1200, 0));
        rail(&mut st, 800, &CFG);
        let f = free(&mut st, &CFG);
        assert_eq!((f.band.hi, f.terms), (600, flag::RAIL));
        rail(&mut st, 750, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 300);
        rail(&mut st, 700, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 0);
        rail(&mut st, 100, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 0);
        rail(&mut st, 3000, &CFG);
        let f = free(&mut st, &CFG);
        assert_eq!((f.band.hi, f.terms), (1200, 0));
        // a saturated slope far over the floor never binds
        let steep = LimitCfg {
            rail_fold_q8: u16::MAX,
            ..CFG
        };
        rail(&mut st, u16::MAX, &steep);
        assert_eq!(free_fold(&mut st, &steep), 1200);
        // slope 0: the fold is off
        let off = LimitCfg {
            rail_fold_q8: 0,
            ..CFG
        };
        rail(&mut st, 100, &off);
        assert_eq!(free_fold(&mut st, &off), 1200);
    }

    /// the goal binds under the cap, clips to the cap over it,
    /// and 0 is the cap.
    #[test]
    fn goal_binds_under_the_cap_and_clips_over_it() {
        let mut st = LimitState::new();
        let at = |st: &mut LimitState, goal_abs: u16| {
            fold(
                st,
                &FoldIn {
                    omega_hat_abs_cps: 1000,
                    goal_abs,
                    ..Default::default()
                },
                &CFG,
            )
        };
        let f = at(&mut st, 400);
        assert_eq!((f.band.hi, f.terms), (400, flag::GOAL));
        let f = at(&mut st, 5000);
        assert_eq!((f.band.hi, f.terms), (1200, 0));
        let f = at(&mut st, 0);
        assert_eq!((f.band.hi, f.terms), (1200, 0));
        // a goal over the fold leaves the fold standing on the verdict
        for _ in 0..10 {
            fold_at(&mut st, 480, 0, &CFG);
        }
        let held = FoldIn {
            i_held_abs: 263,
            goal_abs: 400,
            ..Default::default()
        };
        let f = fold(&mut st, &held, &CFG);
        assert_eq!(f.band.hi, 300);
        assert_eq!(
            f.terms,
            flag::GOAL,
            "the goal binds the min, the fold rides under"
        );
    }

    #[test]
    fn endstop_directional_all_combos() {
        let mut st = LimitState::new();
        let at = |st: &mut LimitState, theta: i32| {
            fold(
                st,
                &FoldIn {
                    omega_hat_abs_cps: 1000,
                    theta_hat_counts: theta,
                    ..Default::default()
                },
                &CFG,
            )
            .band
        };
        // at max: the inward (positive) side collapses,
        // retreat keeps the composed limit - one band carries both verdicts
        let b = at(&mut st, 4000);
        assert_eq!((b.lo, b.hi), (-1200, 0));
        assert_eq!(b.lim(), 1200);
        // mirrored at min
        let b = at(&mut st, -4000);
        assert_eq!((b.lo, b.hi), (0, 1200));
        assert_eq!(b.lim(), 1200);
        // past the wall keeps protecting
        assert_eq!(at(&mut st, 5000).hi, 0);
        // mid-range: symmetric
        let b = at(&mut st, 0);
        assert_eq!((b.lo, b.hi), (-1200, 1200));
        // past the min wall mirrors it
        let b = at(&mut st, -5000);
        assert_eq!((b.lo, b.hi), (0, 1200));
        // the deadlock regression: a zeroed command must not re-read as an
        // inward push - the band's retreat side stays open regardless
        let b = at(&mut st, 4000);
        assert_eq!(b.clamp(0), 0);
        assert_eq!(b.clamp(-120), -120);
        assert_eq!(b.clamp(120), 0);
    }

    /// every term names itself, ties name both, and the tightest
    /// term wins.
    #[test]
    fn min_composition_names_the_tightest_term() {
        let mut st = LimitState::new();
        derate(&mut st, 9990, &CFG); // head 10/2000 -> ~6, way under i_cont
        for _ in 0..10 {
            fold_at(&mut st, 480, 0, &CFG);
        }
        assert!(st.stalled());
        let f = fold(
            &mut st,
            &FoldIn {
                i_held_abs: 480,
                ..Default::default()
            },
            &CFG,
        );
        assert!((5..=6).contains(&f.band.hi), "lim={}", f.band.hi);
        assert_eq!(f.terms, flag::DERATE);
        // the goal at the rail's value: a tie names both
        let mut tie = LimitState::new();
        rail(&mut tie, 800, &CFG);
        let at_rail = free(&mut tie, &CFG);
        assert_eq!((at_rail.band.hi, at_rail.terms), (600, flag::RAIL));
        let f = fold(
            &mut tie,
            &FoldIn {
                omega_hat_abs_cps: 1000,
                goal_abs: 600,
                ..Default::default()
            },
            &CFG,
        );
        assert_eq!((f.band.hi, f.terms), (600, flag::RAIL | flag::GOAL));
        // endstop zero beats everything
        assert_eq!(
            fold(
                &mut st,
                &FoldIn {
                    i_held_abs: 480,
                    theta_hat_counts: 4000,
                    ..Default::default()
                },
                &CFG
            )
            .band
            .hi,
            0
        );
    }

    /// The whole point of the permit: pushing into a wall keeps its current.
    /// Without it the inward side collapses to 0 and the drive is dead, and
    /// no soft-limit value can express "stall here" when the stop sits AT
    /// the position rail.
    #[test]
    fn permit_keeps_the_band_open_at_both_walls() {
        let mut st = LimitState::new();
        let at = |st: &mut LimitState, theta: i32, permit: bool| {
            fold(
                st,
                &FoldIn {
                    theta_hat_counts: theta,
                    stall_permit: permit,
                    ..Default::default()
                },
                &CFG,
            )
            .band
        };
        assert_eq!(at(&mut st, 4000, false).hi, 0);
        assert_eq!(at(&mut st, 4000, true).hi, 1200);
        assert_eq!(at(&mut st, -4000, false).lo, 0);
        assert_eq!(at(&mut st, -4000, true).lo, -1200);
    }

    /// Over i_cont and still is the press signature, and holding against a
    /// stop is exactly that. Under the permit the timer must never trip,
    /// however long it is held - a bench run presses for seconds before
    /// the first rung.
    #[test]
    fn permit_never_trips_the_stall_timer() {
        let mut st = LimitState::new();
        let press = FoldIn {
            i_held_abs: 1200,
            stall_permit: true,
            ..Default::default()
        };
        let mut last = Fold::default();
        for _ in 0..(CFG.stall_time_ticks * 10) {
            last = fold(&mut st, &press, &CFG);
        }
        assert!(!st.stalled());
        assert!(!st.stall_fault_pending());
        assert_eq!(last.band.lim(), 1200);
    }

    /// Granted while already folded, the permit releases rather than leaving
    /// the yield latched: the tool sets the bit after the servo has been
    /// fighting the stop, not before.
    #[test]
    fn permit_releases_an_existing_stall() {
        let mut st = LimitState::new();
        for _ in 0..CFG.stall_time_ticks {
            fold_at(&mut st, 1200, 0, &CFG);
        }
        assert!(st.stalled());
        let press = FoldIn {
            i_held_abs: 1200,
            stall_permit: true,
            ..Default::default()
        };
        assert_eq!(fold(&mut st, &press, &CFG).band.hi, 1200);
        assert!(!st.stalled());
    }

    /// the permit drops the detectors and the endstop and nothing
    /// else. A permit that also dropped a ceiling would cook a winding or
    /// brown the rail out, so the cap, the I2t budget, the thermometer and
    /// the rail still bind under it.
    #[test]
    fn permit_lifts_detectors_never_ceilings() {
        let cfg = LimitCfg {
            thermal_tau_ms: 1024,
            ..CFG
        };
        let press = FoldIn {
            i_held_abs: 1200,
            stall_permit: true,
            ..Default::default()
        };
        let mut st = LimitState::new();
        derate(&mut st, CFG.cutoff_cc, &cfg);
        assert_eq!(fold(&mut st, &press, &cfg).band.hi, 0);
        let mut st = LimitState::new();
        for _ in 0..600 {
            slow_period(&mut st, 1200, &cfg);
        }
        let f = fold(&mut st, &press, &cfg);
        assert_eq!((f.band.hi, f.terms), (300, flag::I2T));
        let mut st = LimitState::new();
        rail(&mut st, 800, &cfg);
        assert_eq!(fold(&mut st, &press, &cfg).band.hi, 600);
        let mut st = LimitState::new();
        let capped = LimitCfg {
            current_limit_counts: 554,
            ..cfg
        };
        assert_eq!(fold(&mut st, &press, &capped).band.hi, 554);
    }
}

//! Fault latch + detectors (spec "Fault raise points"). Any set bit forces
//! MotorCmd::Disabled regardless of torque_enable (`CODE_DRIVE_REVERSED`:
//! MotorCmd::Brake); the torque_enable 0->1 edge is the only ack - it
//! clears the mask and the detector state, and a still-present condition
//! re-latches through the normal detectors.

use crate::kernel::DECIM_MED;
use crate::pos_lut::GRID_SHIFT;

pub const BIT_OVER_CURRENT: u8 = 1 << 0;
pub const BIT_OVER_TEMP: u8 = 1 << 1;
pub const BIT_STALL: u8 = 1 << 2;
pub const BIT_POSITION_ERROR: u8 = 1 << 3;
pub const BIT_SENSOR: u8 = 1 << 4;
pub const BIT_UNDER_VOLT: u8 = 1 << 5;
/// The stored set does not describe this servo. `CODE_DATA`: closed loop
/// refused on the data state (`data_state` module), raised at entry while
/// `data_flags` names a reason, and every tick a Velocity or Position loop
/// runs on a zero Ke; raised on use, never on state alone.
/// `CODE_DRIVE_REVERSED`: the shaft turned against the drive
/// ([`DriveCheck`]), a motor wired the other way round from
/// `drive_polarity`.
pub const BIT_DATA: u8 = 1 << 6;
/// The bus fell silent under torque for `bus_deadman_ticks`: the experiment
/// image's host-loss guard (kernel sec 2.7).
pub const BIT_DEADMAN: u8 = 1 << 7;

/// `fault_code` values: bit index + 1, then the kinds that share a bit;
/// 0 = no fault since the last ack.
pub const CODE_NONE: u8 = 0;
pub const CODE_OVER_CURRENT: u8 = 1;
pub const CODE_OVER_TEMP: u8 = 2;
pub const CODE_STALL: u8 = 3;
pub const CODE_POSITION_ERROR: u8 = 4;
pub const CODE_SENSOR: u8 = 5;
pub const CODE_UNDER_VOLT: u8 = 6;
pub const CODE_DATA: u8 = 7;
pub const CODE_DRIVE_REVERSED: u8 = 8;
pub const CODE_DEADMAN: u8 = 9;

/// Latched mask + the LATEST newly-latched kind (`fault_code`). Re-raising
/// an already-set bit does not touch the code, so a persisting first fault
/// cannot mask a later second one.
pub struct FaultLatch {
    mask: u8,
    last_code: u8,
}

impl FaultLatch {
    pub const fn new() -> Self {
        Self {
            mask: 0,
            last_code: CODE_NONE,
        }
    }

    pub fn raise(&mut self, bit: u8, code: u8) {
        if self.mask & bit == 0 {
            self.mask |= bit;
            self.last_code = code;
        }
    }

    /// Ack (torque_enable 0->1): mask and code both clear.
    pub fn clear(&mut self) {
        self.mask = 0;
        self.last_code = CODE_NONE;
    }

    pub fn mask(&self) -> u8 {
        self.mask
    }

    pub fn code(&self) -> u8 {
        self.last_code
    }
}

impl Default for FaultLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// The overcurrent consecutive-sample count (FAST).
pub struct OcDetector {
    ticks: u8,
}

impl OcDetector {
    pub const fn new() -> Self {
        Self { ticks: 0 }
    }

    /// Ack: the window re-arms.
    pub fn reset(&mut self) {
        self.ticks = 0;
    }

    /// `over` = Some(|i_meas| > oc_trip_counts) for a VALID window, None
    /// when the window is invalid. Invalid samples hold the count - a masked
    /// window must not launder a live overcurrent - and a valid below-trip
    /// sample re-arms. Returns true when the window fills.
    pub fn sample(&mut self, over: Option<bool>, trip_ticks: u8) -> bool {
        match over {
            Some(true) => {
                self.ticks = self.ticks.saturating_add(1);
                self.ticks >= trip_ticks
            }
            Some(false) => {
                self.ticks = 0;
                false
            }
            None => false,
        }
    }
}

impl Default for OcDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// MEDIUM detector counters: position-error persistence timer,
/// sensor-delta screen with its last raw sample.
pub struct Detectors {
    pos_err_ticks: u32,
    sensor_bad: u8,
    sensor_last_pos: u16,
    sensor_seen: bool,
}

impl Detectors {
    pub const fn new() -> Self {
        Self {
            pos_err_ticks: 0,
            sensor_bad: 0,
            sensor_last_pos: 0,
            sensor_seen: false,
        }
    }

    /// Ack: every counter re-arms; the sensor screen re-primes off the next
    /// sample instead of comparing across the disabled gap.
    pub fn reset(&mut self) {
        self.pos_err_ticks = 0;
        self.sensor_bad = 0;
        self.sensor_seen = false;
    }

    /// MEDIUM: raw pot delta screen; `bad_count` consecutive jumps trip.
    pub fn sensor_sample(&mut self, pos: u16, delta_max: u16, bad_count: u8) -> bool {
        let delta = (pos as i32 - self.sensor_last_pos as i32).unsigned_abs();
        let jump = self.sensor_seen && delta > delta_max as u32;
        self.sensor_last_pos = pos;
        self.sensor_seen = true;
        if jump {
            self.sensor_bad = self.sensor_bad.saturating_add(1);
            self.sensor_bad >= bad_count
        } else {
            self.sensor_bad = 0;
            false
        }
    }

    /// MEDIUM: tracking-error persistence; trips once `trip_ticks` kernel
    /// ticks pass over the threshold without a break.
    pub fn pos_err_sample(&mut self, over: bool, trip_ticks: u32) -> bool {
        if over {
            self.pos_err_ticks = self.pos_err_ticks.saturating_add(DECIM_MED as u32);
            self.pos_err_ticks >= trip_ticks
        } else {
            self.pos_err_ticks = 0;
            false
        }
    }
}

impl Default for Detectors {
    fn default() -> Self {
        Self::new()
    }
}

/// One leg of the drive direction check, linearized counts: about 1 deg
/// on the bench MG90, clear of the gear lash and the pot noise. A leg the
/// drive's way proves the wiring; two in a row against it, 40 counts, are
/// a reversed motor where a load cannot explain them. Two legs sit past the
/// sag of a load at the current limit on twice the bench inertia, which
/// trips legs of 16 (DES `gravity_sag_at_the_limit_never_reads_reversed`).
pub const DRIVE_LEG_COUNTS: u16 = 20;
const LEG_Q4: u32 = (DRIVE_LEG_COUNTS as u32) << GRID_SHIFT;

/// `DriveCheck::state`: the wiring is proven under `DriveCheck::polarity`
/// (0, the test a proven servo pays).
const PROVEN: u8 = 0;
/// Holding the position against the anchor.
const TRACK: u8 = 1;
/// The stages, each one check: a leg ended, its end to confirm.
const LEG_END: u8 = 2;
/// A leg confirmed, to judge.
const JUDGE: u8 = 3;
/// Anchor at the position.
const ANCHOR: u8 = 4;

/// Whether a check in `state` runs a stage (`DriveCheck::stage`).
pub const fn stage_due(state: u8) -> bool {
    state >= LEG_END
}

/// The drive direction check, once per medium period while the wiring is
/// unproven. A plain check holds the observer's position against an
/// anchor: an idle drive (zero duty) drags the anchor along, a driven shaft
/// ends a leg once it is `DRIVE_LEG_COUNTS` away. The stages follow, a check
/// each: the leg's end counts only where the pot has come half a leg from
/// the leg's start, and the next leg starts a leg on; then the leg is judged
/// against the drive's direction. The observer's model walks its position
/// off a locked rotor and leaves the pot behind, whatever offset the two
/// hold at rest; on a running shaft it trails the pot, a reversed motor's
/// by up to a leg and a half, which a leg must not lose. A leg the pot has
/// not followed keeps its start, so a walk back is no leg either. A
/// leg the drive's way proves the wiring under the present `drive_polarity`.
/// A leg against it after another is a reversed motor where a load cannot
/// explain it: under a closed loop, whose reference holds against any load
/// it can win, or at a soft limit with the drive pointing back in.
pub struct DriveCheck {
    state: u8,
    /// `drive_polarity` at the last configuration.
    polarity: bool,
    /// The last leg's direction, -1 or 1.
    leg: i8,
    /// The last leg ran against the drive.
    against: bool,
    /// Linearized Q4 counts.
    anchor: i32,
}

impl DriveCheck {
    pub const fn new() -> Self {
        Self {
            state: ANCHOR,
            polarity: false,
            leg: 0,
            against: false,
            anchor: 0,
        }
    }

    pub fn unproven(&self) -> bool {
        self.state != PROVEN
    }

    /// The budget probe's tag: a check in this state runs a stage when
    /// [`stage_due`], and every stage leaves it.
    pub fn state(&self) -> u8 {
        self.state
    }

    /// A configuration under `polarity`: a wiring proven under another is
    /// unproven again.
    pub fn take(&mut self, polarity: bool) {
        if self.polarity != polarity {
            self.polarity = polarity;
            self.state = ANCHOR;
        }
    }

    /// The drive changed hands (torque, a fault, a mode): travel counts
    /// afresh.
    pub fn restart(&mut self) {
        if self.state != PROVEN {
            self.state = ANCHOR;
        }
    }

    /// The plain check, unproven: `pos_q4` the position, linearized Q4,
    /// against the anchor under `duty_q15` the applied duty. Whether a
    /// stage is due instead (`stage`).
    #[inline(always)]
    pub fn check(&mut self, pos_q4: i32, duty_q15: i16) -> bool {
        if self.state != TRACK {
            return true;
        }
        if duty_q15 == 0 {
            self.anchor = pos_q4;
            self.against = false;
        } else if ((pos_q4 - self.anchor + LEG_Q4 as i32 - 1) as u32) >= 2 * LEG_Q4 - 1 {
            // |travel| >= LEG_Q4 in one compare
            self.state = LEG_END;
        }
        false
    }

    /// A stage: `pos_q4` as `check`; `pot_q4` the pot, linearized Q4;
    /// `drive` the drive's direction by its sign (the closed loops'
    /// velocity command, else the duty); `closed` a closed loop drives;
    /// `walled` the band's sides closed, `[lo, hi]` (`IBand`). Whether the
    /// wiring is reversed.
    #[inline(always)]
    pub fn stage(
        &mut self,
        pos_q4: i32,
        pot_q4: impl FnOnce() -> i32,
        drive: impl FnOnce() -> i32,
        closed: bool,
        walled: [bool; 2],
    ) -> bool {
        match self.state {
            LEG_END => {
                self.state = TRACK;
                let up = pos_q4 > self.anchor;
                let moved = pot_q4() - self.anchor;
                if (if up { moved } else { -moved }) < (LEG_Q4 / 2) as i32 {
                    return false;
                }
                self.leg = if up { 1 } else { -1 };
                self.anchor += if up { LEG_Q4 as i32 } else { -(LEG_Q4 as i32) };
                self.state = JUDGE;
                false
            }
            JUDGE => {
                self.state = TRACK;
                let s = drive();
                let along = if self.leg < 0 { -s } else { s };
                if along > 0 {
                    self.state = PROVEN;
                    return false;
                }
                let against = along < 0;
                let reversed =
                    against && self.against && (closed || walled[(self.leg > 0) as usize]);
                self.against = against;
                reversed
            }
            _ => {
                self.state = TRACK;
                self.anchor = pos_q4;
                self.against = false;
                false
            }
        }
    }
}

impl Default for DriveCheck {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latch_latest_code_first_bit_wins_nothing() {
        let mut l = FaultLatch::new();
        assert_eq!((l.mask(), l.code()), (0, CODE_NONE));
        l.raise(BIT_OVER_CURRENT, CODE_OVER_CURRENT);
        assert_eq!((l.mask(), l.code()), (BIT_OVER_CURRENT, CODE_OVER_CURRENT));
        // re-raise of a set bit: code untouched
        l.raise(BIT_OVER_CURRENT, CODE_OVER_CURRENT);
        l.raise(BIT_STALL, CODE_STALL);
        assert_eq!(l.mask(), BIT_OVER_CURRENT | BIT_STALL);
        assert_eq!(l.code(), CODE_STALL);
        // persisting first fault does not clobber the latest latch
        l.raise(BIT_OVER_CURRENT, CODE_OVER_CURRENT);
        assert_eq!(l.code(), CODE_STALL);
        l.clear();
        assert_eq!((l.mask(), l.code()), (0, CODE_NONE));
    }

    #[test]
    fn oc_consecutive_valid_only() {
        let mut d = OcDetector::new();
        for _ in 0..3 {
            assert!(!d.sample(Some(true), 4));
        }
        // invalid holds the count, does not reset
        assert!(!d.sample(None, 4));
        assert!(d.sample(Some(true), 4));
        // valid below-trip re-arms
        let mut d = OcDetector::new();
        for _ in 0..3 {
            d.sample(Some(true), 4);
        }
        assert!(!d.sample(Some(false), 4));
        for _ in 0..3 {
            assert!(!d.sample(Some(true), 4));
        }
        assert!(d.sample(Some(true), 4));
    }

    #[test]
    fn sensor_screen_primes_and_counts() {
        let mut d = Detectors::new();
        // first sample only primes, no matter how large
        assert!(!d.sensor_sample(4000, 10, 2));
        assert!(!d.sensor_sample(4020, 10, 2));
        assert!(d.sensor_sample(4000, 10, 2));
        // clean delta re-arms
        let mut d = Detectors::new();
        d.sensor_sample(100, 10, 2);
        assert!(!d.sensor_sample(200, 10, 2));
        assert!(!d.sensor_sample(205, 10, 2));
        assert!(!d.sensor_sample(300, 10, 2));
        assert!(d.sensor_sample(400, 10, 2));
        // reset re-primes
        d.reset();
        assert!(!d.sensor_sample(0, 10, 2));
    }

    #[test]
    fn pos_err_timer() {
        let mut d = Detectors::new();
        let trip = 10 * DECIM_MED as u32;
        for _ in 0..9 {
            assert!(!d.pos_err_sample(true, trip));
        }
        assert!(!d.pos_err_sample(false, trip));
        for _ in 0..9 {
            assert!(!d.pos_err_sample(true, trip));
        }
        assert!(d.pos_err_sample(true, trip));
    }

    const LEG: i32 = DRIVE_LEG_COUNTS as i32;
    /// The band's closed sides past soft limits at 500 and 3500, `[lo, hi]`.
    fn walls(pos: i32) -> [bool; 2] {
        [pos <= 500, pos >= 3500]
    }

    fn q4(counts: i32) -> i32 {
        counts << GRID_SHIFT
    }

    /// One check as the kernel runs it.
    fn check(c: &mut DriveCheck, pos: i32, duty: i16, sign: i32, closed: bool) -> bool {
        c.unproven()
            && c.check(q4(pos), duty)
            && c.stage(q4(pos), || q4(pos), || sign, closed, walls(pos))
    }

    /// A check under `drive_polarity` 1, anchored at `at`.
    fn started(at: i32) -> DriveCheck {
        let mut c = DriveCheck::new();
        c.take(true);
        assert!(!check(&mut c, at, 1000, 0, true));
        c
    }

    /// The position walks `per` a check from `from` under a driven duty and
    /// a drive of `sign`; the verdict, the legs judged and the position at
    /// the verdict.
    fn walk(
        c: &mut DriveCheck,
        sign: i32,
        from: i32,
        per: i32,
        ticks: i32,
        closed: bool,
    ) -> (bool, u32, i32) {
        let mut legs = 0;
        for n in 0..=ticks {
            let pos = from + n * per;
            legs += (c.state == JUDGE) as u32;
            if check(c, pos, 1000, sign, closed) {
                return (true, legs, pos);
            }
            if !c.unproven() {
                break;
            }
        }
        (false, legs, from + ticks * per)
    }

    #[test]
    fn a_leg_the_drive_way_proves_the_wiring() {
        for sign in [1, -1] {
            let mut c = started(2000);
            assert_eq!(walk(&mut c, sign, 2000, sign, LEG - 1, true).1, 0);
            assert!(c.unproven());
            assert_eq!(
                walk(&mut c, sign, 2000 + sign * (LEG - 1), sign, 3, true).1,
                1
            );
            assert!(!c.unproven());
        }
    }

    #[test]
    fn two_legs_against_a_closed_loop_are_reversed() {
        for sign in [1, -1] {
            let mut c = started(2000);
            let (reversed, legs, at) = walk(&mut c, sign, 2000, -sign, 3 * LEG, true);
            assert!(reversed && legs == 2, "{legs}");
            // a leg ends at LEG; its end and its verdict take a check each
            assert_eq!((at - 2000) * -sign, 2 * LEG + 2);
            let mut c = started(2000);
            let (reversed, legs, at) = walk(&mut c, sign, 2000, -7 * sign, 20, true);
            assert!(reversed && legs == 2, "{legs}");
            assert_eq!((at - 2000) * -sign, 56, "legs of 20 at 7 a check");
            assert!(c.unproven());
        }
    }

    #[test]
    fn an_open_drive_pushed_back_mid_travel_is_not_judged() {
        let mut c = started(2000);
        let (reversed, legs, _) = walk(&mut c, 1, 2000, -1, 400, false);
        assert!(!reversed && legs >= 19, "{legs}");
        assert!(c.unproven());
    }

    /// At a soft limit an open drive pointing back in that loses ground is
    /// reversed within a leg past it; pointing out, travel against it is
    /// the shaft coming back.
    #[test]
    fn an_open_drive_losing_ground_at_a_wall_is_reversed() {
        let mut c = started(600);
        let (reversed, _, at) = walk(&mut c, 1, 600, -1, 200, false);
        assert!(reversed && (500 - LEG..=500).contains(&at), "{at}");
        let mut c = started(3400);
        let (reversed, _, at) = walk(&mut c, -1, 3400, 1, 200, false);
        assert!(reversed && (3500..=3500 + LEG).contains(&at), "{at}");
        let mut c = started(3700);
        assert!(!walk(&mut c, 1, 3700, -1, 100, false).0);
        let mut c = started(300);
        assert!(!walk(&mut c, -1, 300, 1, 100, false).0);
    }

    #[test]
    fn an_idle_drive_a_restart_or_no_drive_counts_afresh() {
        let mut c = started(2000);
        assert_eq!(walk(&mut c, 1, 2000, -1, LEG + 2, true).1, 1);
        assert!(!check(&mut c, 1977, 0, 1, true));
        assert!(!walk(&mut c, 1, 1977, -1, LEG + 2, true).0);
        let mut c = started(2000);
        assert_eq!(walk(&mut c, 1, 2000, -1, LEG + 2, true).1, 1);
        c.restart();
        assert!(!walk(&mut c, 1, 1977, -1, LEG + 3, true).0);
        let mut c = started(2000);
        assert_eq!(walk(&mut c, 1, 2000, -1, LEG + 2, true).1, 1);
        assert!(!walk(&mut c, 0, 1977, -1, LEG + 3, true).0);
    }

    #[test]
    fn a_new_polarity_unproves_a_proven_wiring() {
        let mut c = started(2000);
        walk(&mut c, 1, 2000, 1, LEG + 2, true);
        assert!(!c.unproven());
        c.restart();
        c.take(true);
        assert!(!c.unproven());
        c.take(false);
        assert!(c.unproven());
    }

    /// One check with the pot at `pot`; whether it judged a leg.
    fn check_pot(c: &mut DriveCheck, pos: i32, pot: i32, sign: i32) -> (bool, bool) {
        let reversed =
            c.check(q4(pos), 1000) && c.stage(q4(pos), || q4(pot), || sign, true, walls(pos));
        (reversed, c.state == JUDGE)
    }

    /// The observer walking off a still pot, out three legs and back past
    /// its start either way, ends no leg, whatever offset the two hold at
    /// rest within half a leg.
    #[test]
    fn a_position_the_pot_disagrees_with_ends_no_leg() {
        for off in [-9, -4, 0, 4, 9] {
            let mut c = started(2000);
            let out_and_back = (0..=60).chain((-60..60).rev()).chain(-60..=0);
            for n in out_and_back {
                let pos = 2000 + n;
                assert_eq!(
                    check_pot(&mut c, pos, 2000 + off, -1),
                    (false, false),
                    "{off} {n}"
                );
            }
            assert!(c.unproven());
        }
    }

    /// A running shaft's pot leads the observer, a reversed motor's by up to
    /// a leg and a half: the leg ends where the observer says, and two
    /// against the drive latch as they do with the two agreeing.
    #[test]
    fn a_pot_ahead_of_the_observer_ends_the_leg() {
        for sign in [1, -1] {
            for lead in [0, 10, 25, 30] {
                for off in [-9, 9] {
                    let mut c = started(2000);
                    let mut legs = 0;
                    let at = (0..=3 * LEG).map(|n| 2000 - sign * n).find(|&pos| {
                        let (reversed, judged) =
                            check_pot(&mut c, pos, pos - sign * lead + off, sign);
                        legs += judged as u32;
                        reversed
                    });
                    assert_eq!(at, Some(2000 - sign * (2 * LEG + 2)), "{sign} {lead} {off}");
                    assert_eq!(legs, 2, "{sign} {lead} {off}");
                }
            }
        }
    }
}

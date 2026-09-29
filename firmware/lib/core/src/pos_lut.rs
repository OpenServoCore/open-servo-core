//! Position linearization on a fixed grid over the 12-bit ADC domain: 256
//! intervals of 16 raw counts, 257 i16 corrections against the identity
//! ramp (point k at raw k * 16, the last fixed 0). All-zero is the identity.
//! The output is linearized counts in Q4 (`raw << 4` at identity, `raw +
//! c[k]` at a point), so counts stay counts downstream. `notebooks/oscnb/
//! poslut.py` mirrors `index`, `interp_q4` and `validate` bit for bit; the
//! mg90-a test below pins the two against each other.
//!
//! The table lives in `Shared` RAM behind a paged window in CONTROL
//! (`ControlPosLut`): the host STOREs it `PAGE_POINTS` at a time, COMMITs,
//! and the kernel applies it only while `pos_lut_state` reads LIVE. SAVE
//! persists the effective table beside the calibration in the CALIB image
//! (`persist` module) and boot loads it back LIVE; anything short of LIVE
//! at SAVE drops to the identity first, so a reboot never applies a table
//! the kernel did not.

use crate::data_state::{STAMP_MISMATCH, job};
use crate::regions::control::addr::pos_lut::POS_LUT_CMD;
use crate::{RegionStorage, Shared};

pub const ADC_BITS: u32 = 12;
pub const GRID_SHIFT: u32 = 4;
pub const GRID: usize = 1 << GRID_SHIFT;
pub const INTERVALS: usize = 1 << (ADC_BITS - GRID_SHIFT);
pub const POINTS: usize = INTERVALS + 1;
/// Points per window page: 64 B, so page, command and points ride one WRITE.
pub const PAGE_POINTS: usize = 32;
/// Pages covering the host-written points; the fixed last point has none.
pub const PAGES: usize = INTERVALS / PAGE_POINTS;
const ADC_MASK: u16 = (1 << ADC_BITS) - 1;
const FRAC_MASK: u16 = GRID as u16 - 1;
const GAIN_MAX: i32 = 16;

/// `pos_lut_state` values. Plain consts, not an `Enum` derive: the field is
/// RO, so no discriminant validation ever runs on it.
pub mod state {
    pub const IDENTITY: u8 = 0;
    /// Pages landed since the last COMMIT; the kernel applies the identity.
    pub const LOADING: u8 = 1;
    pub const LIVE: u8 = 2;
    pub const REJECT_TORQUE: u8 = 3;
    pub const REJECT_ENDS: u8 = 4;
    pub const REJECT_SHAPE: u8 = 5;
}

/// `pos_lut_cmd` values; the field's `le` rule admits nothing above `MAX`.
/// A committed write carrying one runs it and reads back `NONE`.
pub mod cmd {
    pub const NONE: u8 = 0;
    pub const STORE: u8 = 1;
    pub const FETCH: u8 = 2;
    pub const COMMIT: u8 = 3;
    pub const MAX: u8 = COMMIT;
}

/// Why a table cannot be applied.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reject {
    /// A nonzero point at or beyond a stop: the stops must map to themselves.
    Ends,
    /// An interval with local gain outside `[1/16, 16)` of nominal: not
    /// monotone, or garbage that would overflow the Q4 word.
    Shape,
}

/// The interval a raw sample falls in, `<= INTERVALS - 1`.
#[inline(always)]
pub fn index(raw: u16) -> usize {
    ((raw & ADC_MASK) >> GRID_SHIFT) as usize
}

/// Linearized counts in Q4. The ADC mask keeps `index + 1 <= INTERVALS`,
/// so both point loads are provably in range and no bounds check remains.
#[inline(always)]
pub fn interp_q4(raw: u16, points: &[i16; POINTS]) -> u16 {
    let i = index(raw);
    lerp_q4(raw, points[i], points[i + 1])
}

/// The interpolation alone, `c0`/`c1` the points either side of `raw`'s
/// interval; the kernel loads them volatile off the raw table pointer.
#[inline(always)]
pub fn lerp_q4(raw: u16, c0: i16, c1: i16) -> u16 {
    let raw = raw & ADC_MASK;
    let f = (raw & FRAC_MASK) as i32;
    (((raw as i32 + c0 as i32) << GRID_SHIFT) + (c1 as i32 - c0 as i32) * f) as u16
}

/// What the all-zero table yields: `raw << GRID_SHIFT`.
#[inline(always)]
pub fn identity_q4(raw: u16) -> u16 {
    (raw & ADC_MASK) << GRID_SHIFT
}

/// Physics sanity only, never quality: zero at and beyond the stops (every
/// point `k <= (raw_min + 15) >> 4` and `k >= raw_max >> 4`, so both stops
/// map to themselves whether or not they sit on a point; stops unset admits
/// only the identity), then every interval's Q4 gain `16 + c[k+1] - c[k]`
/// in `1..16 * 16`. Ends are judged before shape, as the host mirror does.
pub fn validate(points: &[i16; POINTS], raw_min: u16, raw_max: u16) -> Result<(), Reject> {
    let lo = (raw_min as usize + GRID - 1) >> GRID_SHIFT;
    let hi = raw_max as usize >> GRID_SHIFT;
    if points
        .iter()
        .enumerate()
        .any(|(k, &c)| (k <= lo || k >= hi) && c != 0)
    {
        return Err(Reject::Ends);
    }
    let shape = points.iter().zip(&points[1..]).all(|(&c0, &c1)| {
        let d = GRID as i32 + c1 as i32 - c0 as i32;
        (1..GAIN_MAX * GRID as i32).contains(&d)
    });
    if shape { Ok(()) } else { Err(Reject::Shape) }
}

impl Reject {
    const fn state(self) -> u8 {
        match self {
            Reject::Ends => state::REJECT_ENDS,
            Reject::Shape => state::REJECT_SHAPE,
        }
    }
}

/// [`validate`] as the `pos_lut_state` a COMMIT lands.
pub fn verdict(points: &[i16; POINTS], raw_min: u16, raw_max: u16) -> u8 {
    match validate(points, raw_min, raw_max) {
        Ok(()) => state::LIVE,
        Err(r) => r.state(),
    }
}

impl Shared {
    /// The kernel's per-tick read while `pos_lut_state` is LIVE: two volatile
    /// point loads off the raw pointer, no `&` across the ISR boundary. HIGH
    /// dispatch (the sole writer) can preempt between the two loads, but
    /// STORE and COMMIT are torque-gated, so a mixed read only ever reaches
    /// a disabled servo, whose observer reseeds at the next enable.
    #[inline(always)]
    pub fn pos_lut_q4(&self, raw: u16) -> u16 {
        let i = index(raw);
        let k = self.pos_lut_ptr().cast::<i16>();
        // SAFETY: `index` keeps i + 1 <= INTERVALS < POINTS, both loads stay
        // inside the static array; single-writer contract in the fn doc.
        let (c0, c1) = unsafe { (k.add(i).read_volatile(), k.add(i + 1).read_volatile()) };
        lerp_q4(raw, c0, c1)
    }

    /// Run the command a committed write left in `pos_lut_cmd`, then clear it.
    /// STORE copies the window into the array's page and leaves LOADING;
    /// FETCH copies that page back into the window; COMMIT leaves LOADING
    /// (the kernel applies the identity) and posts the main-loop job that
    /// validates the array against the stops, lands LIVE or a REJECT and
    /// runs the stamp checkpoint - validation plus the CRC outlast the
    /// reply deadline. STORE and COMMIT are torque-gated: from LIVE a
    /// refusal leaves LIVE standing, since the state is what the kernel
    /// applies and a refusal must not move it under a running loop; from
    /// any other state it reads REJECT_TORQUE. Leaving LIVE by STORE marks
    /// the stamp stale the way a covered write does, and a STORE cancels a
    /// posted COMMIT: the array is loading again, and only the next COMMIT
    /// judges it. HIGH dispatch only; one copy behind both commit sites.
    #[inline(never)]
    pub fn pos_lut_after_commit(&self, addr: u16, len: u16) {
        if addr > POS_LUT_CMD || addr.saturating_add(len) <= POS_LUT_CMD {
            return;
        }
        let (ran, stale) = self.with_pos_lut_mut(|k| {
            self.table.with_mut(|t| {
                let torque = t.control.lifecycle.torque_enable;
                let w = &mut t.control.pos_lut;
                let at = w.pos_lut_page as usize * PAGE_POINTS;
                let refused = if w.pos_lut_state == state::LIVE {
                    state::LIVE
                } else {
                    state::REJECT_TORQUE
                };
                let ran = w.pos_lut_cmd;
                let mut stale = false;
                match ran {
                    cmd::STORE if torque => w.pos_lut_state = refused,
                    cmd::STORE => {
                        stale = w.pos_lut_state == state::LIVE;
                        w.pos_lut_state = state::LOADING;
                        if let Some(dst) = k.get_mut(at..) {
                            for (d, s) in dst.iter_mut().zip(&w.pos_lut_points) {
                                *d = *s;
                            }
                        }
                    }
                    cmd::FETCH => {
                        if let Some(src) = k.get(at..) {
                            for (d, s) in w.pos_lut_points.iter_mut().zip(src) {
                                *d = *s;
                            }
                        }
                    }
                    cmd::COMMIT if torque => w.pos_lut_state = refused,
                    cmd::COMMIT => {
                        stale = true;
                        w.pos_lut_state = state::LOADING;
                    }
                    _ => {}
                }
                w.pos_lut_cmd = cmd::NONE;
                (if torque { cmd::NONE } else { ran }, stale)
            })
        });
        match ran {
            cmd::STORE => self.data_touch(0, job::LUT_COMMIT),
            cmd::COMMIT => self.data_touch(job::LUT_COMMIT, 0),
            _ => {}
        }
        if stale {
            self.table
                .with_mut(|t| t.telemetry.mode.data_flags |= STAMP_MISMATCH);
        }
    }

    /// Settle the array to what the kernel applies before SAVE persists
    /// it: a load in progress or a rejected array is the identity, so it
    /// becomes one. HIGH dispatch only, torque off.
    pub fn pos_lut_settle(&self) {
        let live = self
            .table
            .with(|t| t.control.pos_lut.pos_lut_state == state::LIVE);
        if !live {
            self.with_pos_lut_mut(|k| k.fill(0));
            self.table
                .with_mut(|t| t.control.pos_lut.pos_lut_state = state::IDENTITY);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::crc::osc_crc_continue;

    const ZERO: [i16; POINTS] = [0; POINTS];

    // mg90-a on the 2S session, stops 209/3849, covered 542..3520 (bringup
    // captures/mg90/pos-lut-mg90-a-grid.json; the fixed last point appended).
    const MG90_A_MIN: u16 = 209;
    const MG90_A_MAX: u16 = 3849;
    const MG90_A: [i16; INTERVALS] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
        0, 0, 0, 4, -1, -5, -10, -16, -21, -23, -21, -14, -12, -11, -11, -14, //
        -22, -25, -28, -28, -25, -27, -28, -29, -31, -30, -22, -23, -24, -20, -18, -20, //
        -21, -29, -32, -35, -34, -33, -34, -33, -28, -27, -25, -25, -27, -30, -28, -25, //
        -28, -24, -23, -23, -25, -23, -6, 3, 2, -2, -6, -8, -8, -3, -3, -7, //
        -12, -14, -16, -17, -17, -12, 2, 15, 19, 16, 15, 17, 22, 25, 28, 30, //
        28, 29, 29, 28, 25, 21, 19, 16, 14, 13, 12, 11, 6, 10, 20, 31, //
        33, 38, 42, 42, 39, 35, 30, 26, 23, 22, 23, 24, 22, 26, 27, 30, //
        34, 38, 41, 44, 42, 42, 42, 41, 43, 44, 45, 44, 43, 44, 45, 44, //
        42, 42, 45, 47, 53, 59, 66, 71, 70, 70, 69, 65, 60, 56, 51, 47, //
        44, 45, 43, 43, 42, 40, 39, 38, 39, 37, 35, 34, 33, 30, 27, 23, //
        20, 18, 14, 9, 7, 8, 13, 16, 17, 18, 16, 13, 7, 6, 5, 6, //
        3, 2, 7, 10, 10, 8, 13, 12, 9, 6, 2, -1, 0, 0, 0, 0, //
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    ];
    // osc-CRC-16 over the 4096 Q4 words LE, from poslut.GridLut.q4 in Python.
    const MG90_A_Q4_CRC: u16 = 0x8F97;
    const MG90_A_Q4_SAMPLES: [(u16, u16); 9] = [
        (209, 3344),
        (232, 3712),
        (541, 8656),
        (1023, 16033),
        (1024, 16048),
        (2048, 33296),
        (3072, 49472),
        (3849, 61584),
        (4095, 65520),
    ];

    fn mg90_a() -> [i16; POINTS] {
        let mut k = ZERO;
        k[..INTERVALS].copy_from_slice(&MG90_A);
        k
    }

    fn table(edits: &[(usize, i16)]) -> [i16; POINTS] {
        let mut k = ZERO;
        for &(i, c) in edits {
            k[i] = c;
        }
        k
    }

    /// A step of `rise` at point 101 tapering back to zero one Q4 count under
    /// nominal per interval, so only the step's interval is off nominal.
    fn step(rise: i16) -> [i16; POINTS] {
        let mut k = ZERO;
        let mut c = rise;
        let mut i = 101;
        while c > 0 {
            k[i] = c;
            c -= GRID as i16 - 1;
            i += 1;
        }
        k
    }

    #[test]
    fn zero_table_is_identity_exhaustive() {
        for raw in 0..=ADC_MASK {
            assert_eq!(interp_q4(raw, &ZERO), raw << GRID_SHIFT);
        }
    }

    #[test]
    fn points_land_exactly() {
        let k = mg90_a();
        for i in 0..INTERVALS {
            let raw = (i * GRID) as u16;
            assert_eq!(interp_q4(raw, &k), (raw as i32 + k[i] as i32) as u16 * 16);
        }
    }

    #[test]
    fn endpoints_map_to_themselves() {
        let k = mg90_a();
        for stop in [MG90_A_MIN, 232, MG90_A_MAX] {
            assert_eq!(interp_q4(stop, &k), stop << GRID_SHIFT);
        }
    }

    #[test]
    fn valid_tables_are_monotone() {
        let k = mg90_a();
        let mut prev = interp_q4(0, &k);
        for raw in 1..=ADC_MASK {
            let q = interp_q4(raw, &k);
            assert!(q > prev, "raw {raw}: {q} <= {prev}");
            prev = q;
        }
    }

    #[test]
    fn mg90_a_validates_against_its_stops() {
        let k = mg90_a();
        assert_eq!(validate(&k, MG90_A_MIN, MG90_A_MAX), Ok(()));
        assert_eq!(validate(&k, 232, MG90_A_MAX), Ok(()));
    }

    #[test]
    fn mg90_a_matches_the_python_reference() {
        let k = mg90_a();
        let mut crc = 0;
        for raw in 0..=ADC_MASK {
            crc = osc_crc_continue(crc, &interp_q4(raw, &k).to_le_bytes());
        }
        assert_eq!(crc, MG90_A_Q4_CRC);
        for (raw, q4) in MG90_A_Q4_SAMPLES {
            assert_eq!(interp_q4(raw, &k), q4, "raw {raw}");
        }
    }

    #[test]
    fn validate_rejects_ends() {
        // stops off a point: 209 -> points 0..=14 are the low inset, 3849 -> 240..
        for k in [13, 14, 240, 255, 256] {
            assert_eq!(validate(&table(&[(k, 1)]), 209, 3849), Err(Reject::Ends));
        }
        for k in [15, 239] {
            assert_eq!(validate(&table(&[(k, 1)]), 209, 3849), Ok(()));
        }
        // stops on a point: the point itself is inset on both sides
        for k in [15, 16, 240] {
            assert_eq!(validate(&table(&[(k, 1)]), 256, 3840), Err(Reject::Ends));
        }
        for k in [17, 239] {
            assert_eq!(validate(&table(&[(k, 1)]), 256, 3840), Ok(()));
        }
        // stops unset: only the identity passes
        assert_eq!(validate(&ZERO, 0, 0), Ok(()));
        assert_eq!(validate(&table(&[(100, 1)]), 0, 0), Err(Reject::Ends));
        // ends are judged before shape
        assert_eq!(validate(&table(&[(0, -100)]), 209, 3849), Err(Reject::Ends));
    }

    #[test]
    fn validate_rejects_nonmonotone() {
        assert_eq!(
            validate(&table(&[(100, 16)]), 209, 3849),
            Err(Reject::Shape)
        );
        assert_eq!(validate(&table(&[(100, 15)]), 209, 3849), Ok(()));
        let mut k = mg90_a();
        k[100] = k[99] - GRID as i16;
        assert_eq!(validate(&k, MG90_A_MIN, MG90_A_MAX), Err(Reject::Shape));
    }

    #[test]
    fn validate_rejects_gain_16x() {
        assert_eq!(validate(&step(240), 209, 3849), Err(Reject::Shape));
        assert_eq!(validate(&step(239), 209, 3849), Ok(()));
    }

    #[test]
    fn validate_accepts_sg90_class_6x_interval() {
        assert_eq!(validate(&step(80), 209, 3849), Ok(()));
    }

    #[test]
    fn interp_never_indexes_past_the_last_point() {
        let k = table(&[(255, -3), (256, 1)]);
        assert_eq!(interp_q4(4095, &k), ((4095 - 3) << 4) + 4 * 15);
        for raw in [0x1234, 0xF234, 0xFFFF] {
            assert_eq!(interp_q4(raw, &ZERO), interp_q4(raw & ADC_MASK, &ZERO));
        }
    }

    #[test]
    fn interp_wraps_like_the_u16_cast() {
        assert_eq!(interp_q4(0, &table(&[(0, -1)])), 0xFFF0);
        assert_eq!(interp_q4(4095, &table(&[(255, 1)])), 65521);
    }

    /// The kernel's volatile read off the array is `interp_q4` for every
    /// raw count, and the identity is `raw << 4` past the ADC span too.
    #[test]
    fn kernel_read_matches_interp_exhaustive() {
        let sh = Shared::new();
        let k = mg90_a();
        sh.with_pos_lut_mut(|a| *a = k);
        for raw in 0..=u16::MAX {
            assert_eq!(sh.pos_lut_q4(raw), interp_q4(raw, &k), "raw {raw}");
            assert_eq!(identity_q4(raw), interp_q4(raw, &ZERO), "raw {raw}");
        }
    }

    // The window: what the dispatcher's post-commit step runs.

    use crate::data_state::STAMP_MISMATCH;
    use crate::regions::control::addr::pos_lut::{POS_LUT_PAGE, POS_LUT_POINTS, POS_LUT_STATE};
    use crate::stamp;

    fn servo() -> Shared {
        let sh = Shared::new();
        sh.table.with_mut(|t| {
            t.calib.pot.raw_min = MG90_A_MIN;
            t.calib.pot.raw_max = MG90_A_MAX;
            t.calib.motor.recip_ke_q = 3700;
            t.calib.motor.ke_vpc_q = 1150;
            t.calib.stamp.plant_stamp = stamp::compute(t, None);
        });
        sh.data_state_checkpoint();
        assert_eq!(flags(&sh), 0);
        sh
    }

    fn flags(sh: &Shared) -> u8 {
        sh.table.with(|t| t.telemetry.mode.data_flags)
    }

    fn pos_lut_state(sh: &Shared) -> u8 {
        sh.table.with(|t| t.control.pos_lut.pos_lut_state)
    }

    fn window(sh: &Shared) -> [i16; PAGE_POINTS] {
        sh.table.with(|t| t.control.pos_lut.pos_lut_points)
    }

    /// One committed window write: page, command and points as one span.
    fn command(sh: &Shared, page: u8, c: u8, points: &[i16; PAGE_POINTS]) {
        sh.table.with_mut(|t| {
            t.control.pos_lut.pos_lut_page = page;
            t.control.pos_lut.pos_lut_cmd = c;
            t.control.pos_lut.pos_lut_points = *points;
        });
        sh.pos_lut_after_commit(POS_LUT_PAGE, 2 + 2 * PAGE_POINTS as u16);
        assert_eq!(sh.table.with(|t| t.control.pos_lut.pos_lut_cmd), cmd::NONE);
        sh.data_job_service();
    }

    fn store_all(sh: &Shared, k: &[i16; POINTS]) {
        for page in 0..PAGES {
            let mut w = [0; PAGE_POINTS];
            w.copy_from_slice(&k[page * PAGE_POINTS..][..PAGE_POINTS]);
            command(sh, page as u8, cmd::STORE, &w);
            assert_eq!(pos_lut_state(sh), state::LOADING);
        }
    }

    fn commit(sh: &Shared) -> u8 {
        command(sh, 0, cmd::COMMIT, &[0; PAGE_POINTS]);
        pos_lut_state(sh)
    }

    #[test]
    fn store_commit_goes_live_and_fetch_reads_back() {
        let sh = servo();
        let k = mg90_a();
        assert_eq!(pos_lut_state(&sh), state::IDENTITY);
        store_all(&sh, &k);
        assert_eq!(flags(&sh), 0, "loading applies the identity");
        assert_eq!(commit(&sh), state::LIVE);
        sh.with_pos_lut(|live| assert_eq!(live, &k));
        assert_eq!(flags(&sh), STAMP_MISMATCH, "the hashed points changed");
        sh.table.with_mut(|t| {
            t.calib.stamp.plant_stamp = stamp::compute(t, Some(&MG90_A));
        });
        sh.data_state_after_commit(crate::regions::calib::addr::stamp::PLANT_STAMP, 2);
        sh.data_job_service();
        assert_eq!(flags(&sh), 0);
        for page in 0..PAGES {
            command(&sh, page as u8, cmd::FETCH, &[0; PAGE_POINTS]);
            assert_eq!(window(&sh), k[page * PAGE_POINTS..][..PAGE_POINTS]);
        }
        assert_eq!(pos_lut_state(&sh), state::LIVE, "fetch moves nothing");
    }

    #[test]
    fn zero_table_committed_live_keeps_the_stamp() {
        let sh = servo();
        store_all(&sh, &ZERO);
        assert_eq!(commit(&sh), state::LIVE);
        assert_eq!(flags(&sh), 0);
    }

    #[test]
    fn commit_rejects_and_the_kernel_stays_identity() {
        let sh = servo();
        // a nonzero point inside the low inset of stop 209
        let mut ends = mg90_a();
        ends[14] = 1;
        store_all(&sh, &ends);
        assert_eq!(commit(&sh), state::REJECT_ENDS);
        assert_eq!(flags(&sh), 0, "identity hashes as before");
        let mut shape = mg90_a();
        shape[100] = shape[99] - GRID as i16;
        store_all(&sh, &shape);
        assert_eq!(commit(&sh), state::REJECT_SHAPE);
        assert_eq!(flags(&sh), 0);
        // a rejected array is still there to fix page by page
        store_all(&sh, &mg90_a());
        assert_eq!(commit(&sh), state::LIVE);
        assert_eq!(flags(&sh), STAMP_MISMATCH);
        // the stops moved into the table: LIVE falls back and the stamp
        // over the array no longer holds
        sh.table.with_mut(|t| {
            t.calib.stamp.plant_stamp = stamp::compute(t, Some(&MG90_A));
        });
        sh.data_state_checkpoint();
        assert_eq!(flags(&sh), 0);
        sh.table.with_mut(|t| t.calib.pot.raw_max = 3000);
        assert_eq!(commit(&sh), state::REJECT_ENDS);
        assert_eq!(flags(&sh), STAMP_MISMATCH);
    }

    #[test]
    fn torque_refuses_store_and_commit() {
        let sh = servo();
        let k = mg90_a();
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        let mut w = [0; PAGE_POINTS];
        w.copy_from_slice(&k[64..96]);
        command(&sh, 2, cmd::STORE, &w);
        assert_eq!(pos_lut_state(&sh), state::REJECT_TORQUE);
        sh.with_pos_lut(|a| assert_eq!(a, &ZERO));
        assert_eq!(commit(&sh), state::REJECT_TORQUE);
        assert_eq!(flags(&sh), 0);
        command(&sh, 2, cmd::FETCH, &[0; PAGE_POINTS]);
        assert_eq!(window(&sh), [0; PAGE_POINTS], "fetch is not gated");

        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        store_all(&sh, &k);
        assert_eq!(commit(&sh), state::LIVE);
        // under torque a live table stands: a refusal never moves what the
        // kernel applies
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        command(&sh, 2, cmd::STORE, &[7; PAGE_POINTS]);
        assert_eq!(pos_lut_state(&sh), state::LIVE);
        sh.with_pos_lut(|a| assert_eq!(a, &k));
        assert_eq!(commit(&sh), state::LIVE);
        assert_eq!(flags(&sh), STAMP_MISMATCH, "unchanged since the commit");
    }

    #[test]
    fn store_out_of_live_marks_the_stamp_stale() {
        let sh = servo();
        let k = mg90_a();
        store_all(&sh, &k);
        assert_eq!(commit(&sh), state::LIVE);
        sh.table.with_mut(|t| {
            t.calib.stamp.plant_stamp = stamp::compute(t, Some(&MG90_A));
        });
        sh.data_state_checkpoint();
        assert_eq!(flags(&sh), 0);
        let mut w = [0; PAGE_POINTS];
        w.copy_from_slice(&k[..PAGE_POINTS]);
        command(&sh, 0, cmd::STORE, &w);
        assert_eq!(pos_lut_state(&sh), state::LOADING);
        assert_eq!(flags(&sh), STAMP_MISMATCH);
        // the same table back: the checkpoint matches again
        assert_eq!(commit(&sh), state::LIVE);
        assert_eq!(flags(&sh), 0);
    }

    #[test]
    fn checkpoint_hashes_the_array_only_while_live() {
        let sh = servo();
        let k = mg90_a();
        store_all(&sh, &k);
        sh.data_state_checkpoint();
        assert_eq!(flags(&sh), 0, "loading hashes the identity");
        assert_eq!(commit(&sh), state::LIVE);
        assert_eq!(flags(&sh), STAMP_MISMATCH);
    }

    #[test]
    fn only_a_write_covering_the_command_runs_it() {
        let sh = servo();
        sh.table.with_mut(|t| {
            t.control.pos_lut.pos_lut_cmd = cmd::STORE;
            t.control.pos_lut.pos_lut_points[0] = 5;
        });
        sh.pos_lut_after_commit(POS_LUT_PAGE, 1);
        sh.pos_lut_after_commit(POS_LUT_POINTS, 64);
        sh.pos_lut_after_commit(POS_LUT_STATE, 1);
        assert_eq!(pos_lut_state(&sh), state::IDENTITY);
        sh.pos_lut_after_commit(POS_LUT_CMD, 1);
        assert_eq!(pos_lut_state(&sh), state::LOADING);
        sh.with_pos_lut(|a| assert_eq!(a[0], 5));
    }

    /// The commit site only marks and posts: the verdict and the
    /// checkpoint land when the main loop services the job.
    #[test]
    fn commit_posts_the_verdict_for_the_main_loop() {
        let sh = servo();
        let k = mg90_a();
        store_all(&sh, &k);
        assert!(!sh.data_job_pending(), "a store posts nothing");
        sh.table
            .with_mut(|t| t.control.pos_lut.pos_lut_cmd = cmd::COMMIT);
        sh.pos_lut_after_commit(POS_LUT_CMD, 1);
        assert!(sh.data_job_pending());
        assert_eq!(pos_lut_state(&sh), state::LOADING, "identity until judged");
        assert_eq!(flags(&sh), STAMP_MISMATCH, "refused until verified");
        assert!(sh.data_job_service());
        assert!(!sh.data_job_pending());
        assert_eq!(pos_lut_state(&sh), state::LIVE);
        assert_eq!(flags(&sh), STAMP_MISMATCH, "the hashed points changed");
        assert!(!sh.data_job_service(), "nothing left");

        // a STORE behind a posted COMMIT cancels it: loading again, and
        // only the next COMMIT judges the array
        sh.table
            .with_mut(|t| t.control.pos_lut.pos_lut_cmd = cmd::COMMIT);
        sh.pos_lut_after_commit(POS_LUT_CMD, 1);
        let mut w = [0; PAGE_POINTS];
        w.copy_from_slice(&k[..PAGE_POINTS]);
        sh.table.with_mut(|t| {
            t.control.pos_lut.pos_lut_page = 0;
            t.control.pos_lut.pos_lut_cmd = cmd::STORE;
            t.control.pos_lut.pos_lut_points = w;
        });
        sh.pos_lut_after_commit(POS_LUT_PAGE, 2 + 2 * PAGE_POINTS as u16);
        assert!(!sh.data_job_pending());
        assert!(!sh.data_job_service());
        assert_eq!(pos_lut_state(&sh), state::LOADING);
        assert_eq!(commit(&sh), state::LIVE);
    }

    /// A write landing between the run and its publish moves the
    /// generation: the run is discarded, the marks stand, the job stays
    /// posted and the next run judges the new set.
    #[test]
    fn a_write_mid_job_discards_the_run() {
        let sh = servo();
        let k = mg90_a();
        store_all(&sh, &k);
        sh.table
            .with_mut(|t| t.control.pos_lut.pos_lut_cmd = cmd::COMMIT);
        sh.pos_lut_after_commit(POS_LUT_CMD, 1);
        let run = sh.data_job_run().expect("posted");
        // torque comes on under the run: what HIGH would have refused
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        sh.data_state_after_commit(crate::regions::control::addr::lifecycle::TORQUE_ENABLE, 1);
        assert!(!sh.data_job_publish(run));
        assert_eq!(pos_lut_state(&sh), state::LOADING);
        assert_eq!(flags(&sh), STAMP_MISMATCH);
        assert!(sh.data_job_pending());
        assert!(sh.data_job_service());
        assert_eq!(pos_lut_state(&sh), state::REJECT_TORQUE);
        assert_eq!(flags(&sh), 0, "the identity is what the stamp covers");

        // the same race on a stamp write, against a covered write
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        sh.data_state_after_commit(crate::regions::control::addr::lifecycle::TORQUE_ENABLE, 1);
        assert!(!sh.data_job_pending(), "a torque write posts nothing");
        sh.data_state_after_commit(crate::regions::calib::addr::stamp::PLANT_STAMP, 2);
        let run = sh.data_job_run().expect("posted");
        sh.table.with_mut(|t| t.calib.pot.raw_max = 3000);
        sh.data_state_after_commit(crate::regions::calib::addr::pot::RAW_MAX, 2);
        assert!(!sh.data_job_publish(run));
        assert_eq!(flags(&sh), STAMP_MISMATCH);
        assert!(sh.data_job_service());
        assert_eq!(flags(&sh), STAMP_MISMATCH, "the stop moved under the stamp");
        sh.table
            .with_mut(|t| t.calib.stamp.plant_stamp = stamp::compute(t, None));
        sh.data_state_after_commit(crate::regions::calib::addr::stamp::PLANT_STAMP, 2);
        assert!(sh.data_job_service());
        assert_eq!(flags(&sh), 0);
    }
}

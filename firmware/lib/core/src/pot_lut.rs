//! Pot linearization on a fixed grid over the 12-bit ADC domain: 256
//! intervals of 16 raw counts, 257 i16 corrections against the identity
//! ramp (knot k at raw k * 16, the last fixed 0). All-zero is the identity.
//! The output is linearized counts in Q4 (`raw << 4` at identity, `raw +
//! c[k]` at a knot), so counts stay counts downstream. `notebooks/oscnb/
//! potlut.py` mirrors `index`, `interp_q4` and `validate` bit for bit; the
//! mg90-a test below pins the two against each other.

pub const ADC_BITS: u32 = 12;
pub const GRID_SHIFT: u32 = 4;
pub const GRID: usize = 1 << GRID_SHIFT;
pub const INTERVALS: usize = 1 << (ADC_BITS - GRID_SHIFT);
pub const KNOTS: usize = INTERVALS + 1;
const ADC_MASK: u16 = (1 << ADC_BITS) - 1;
const FRAC_MASK: u16 = GRID as u16 - 1;
const GAIN_MAX: i32 = 16;

/// Why a table cannot be applied.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reject {
    /// A nonzero knot at or beyond a stop: the stops must map to themselves.
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
/// so both knot loads are provably in range and no bounds check remains.
#[inline(always)]
pub fn interp_q4(raw: u16, knots: &[i16; KNOTS]) -> u16 {
    let raw = raw & ADC_MASK;
    let i = (raw >> GRID_SHIFT) as usize;
    let c0 = knots[i] as i32;
    let c1 = knots[i + 1] as i32;
    let f = (raw & FRAC_MASK) as i32;
    (((raw as i32 + c0) << GRID_SHIFT) + (c1 - c0) * f) as u16
}

/// Physics sanity only, never quality: zero at and beyond the stops (every
/// knot `k <= (raw_min + 15) >> 4` and `k >= raw_max >> 4`, so both stops
/// map to themselves whether or not they sit on a knot; stops unset admits
/// only the identity), then every interval's Q4 gain `16 + c[k+1] - c[k]`
/// in `1..16 * 16`. Ends are judged before shape, as the host mirror does.
pub fn validate(knots: &[i16; KNOTS], raw_min: u16, raw_max: u16) -> Result<(), Reject> {
    let lo = (raw_min as usize + GRID - 1) >> GRID_SHIFT;
    let hi = raw_max as usize >> GRID_SHIFT;
    if knots
        .iter()
        .enumerate()
        .any(|(k, &c)| (k <= lo || k >= hi) && c != 0)
    {
        return Err(Reject::Ends);
    }
    let shape = knots.iter().zip(&knots[1..]).all(|(&c0, &c1)| {
        let d = GRID as i32 + c1 as i32 - c0 as i32;
        (1..GAIN_MAX * GRID as i32).contains(&d)
    });
    if shape { Ok(()) } else { Err(Reject::Shape) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::crc::osc_crc_continue;

    const ZERO: [i16; KNOTS] = [0; KNOTS];

    // mg90-a on the 2S session, stops 209/3849, covered 542..3520 (bringup
    // captures/mg90/pot-lut-mg90-a-grid.json; the fixed last knot appended).
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
    // osc-CRC-16 over the 4096 Q4 words LE, from potlut.GridLut.q4 in Python.
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

    fn mg90_a() -> [i16; KNOTS] {
        let mut k = ZERO;
        k[..INTERVALS].copy_from_slice(&MG90_A);
        k
    }

    fn table(edits: &[(usize, i16)]) -> [i16; KNOTS] {
        let mut k = ZERO;
        for &(i, c) in edits {
            k[i] = c;
        }
        k
    }

    /// A step of `rise` at knot 101 tapering back to zero one Q4 count under
    /// nominal per interval, so only the step's interval is off nominal.
    fn step(rise: i16) -> [i16; KNOTS] {
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
    fn knots_land_exactly() {
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
        // stops off a knot: 209 -> knots 0..=14 are the low inset, 3849 -> 240..
        for k in [13, 14, 240, 255, 256] {
            assert_eq!(validate(&table(&[(k, 1)]), 209, 3849), Err(Reject::Ends));
        }
        for k in [15, 239] {
            assert_eq!(validate(&table(&[(k, 1)]), 209, 3849), Ok(()));
        }
        // stops on a knot: the knot itself is inset on both sides
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
    fn interp_never_indexes_past_the_last_knot() {
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
}

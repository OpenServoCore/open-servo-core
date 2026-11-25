//! The ADC's wide codes from a sweep's raw pot rows: the converter sticks
//! at a code just under a multiple of 256 while its input rises through the
//! codes above, so those codes are never read and the code under them
//! reads several times its neighbors' rows (`osc_servo_core::adc`). Stage
//! 1's finder (oscnb `widecodes.find`) with one change: a carry's rows are
//! held against the codes around it, not the sweep's one median, since an
//! open-loop sweep's speed, and so its rows a code, varies 2-3x along the
//! travel.
//!
//! Carry k is code `(k << 8) | 255`, its slot 8 x the codes it skips (half
//! the skipped span, Q4), as the firmware's map holds it.

use osc_servo_core::adc::{ADC_MASK, ADC_WIDE_MASK, ADC_WIDE_SHIFT, ADC_WIDE_SLOTS, wide_q4};

use super::median;

/// Q4 a slot holds per skipped code: half a count.
pub const Q4_PER_SKIPPED: u32 = 8;
/// A run shorter than this moves the reading by a count or less at its
/// middle, inside the converter's own ripple from code to code.
pub const MIN_SKIPPED: usize = 3;
/// A stuck code stands for itself and the codes it skips, so it reads
/// (1 + skipped) x its neighbors' rows; under this share of that it is a
/// thin stretch, not a stuck code.
pub const WIDE_MIN_SHARE: f64 = 0.5;
/// Counts at each end of the swept range left out: the codes next to a
/// turnaround are crossed slowly or not at all.
pub const EDGE_MARGIN_COUNTS: usize = 15;
/// Codes either side of a carry its reference is the median rows over.
pub const REF_SIDE_CODES: usize = 64;
/// The run above a carry the reach makes room for.
pub const REACH_RUN_CODES: usize = 16;
/// A run is followed this far at most.
pub const RUN_MAX_CODES: usize = 40;
/// Reference rows a code under which a code never read may be chance.
pub const THIN_ROWS: f64 = 100.0;
/// The same for each half of the repeat check.
pub const HALF_THIN_ROWS: f64 = 50.0;
/// Codes a stray run's reference takes at least.
pub const STRAY_REF_MIN_CODES: usize = 24;
/// Codes in the converter's span.
pub const CODES: usize = ADC_MASK as usize + 1;

/// What one carry's rows say.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum State {
    Wide,
    NotWide,
    /// Under [`THIN_ROWS`] reference rows a code: slot 0, named.
    Thin,
    /// Not inside the swept range with its reference: slot 0, named.
    Unreached,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Carry {
    pub slot: usize,
    pub code: u16,
    pub state: State,
    /// Codes never read right above it.
    pub skipped: usize,
    /// Its rows over the reference.
    pub ratio: f64,
    /// The reference: median rows a code around it.
    pub rows_ref: f64,
}

/// A run of [`MIN_SKIPPED`] or more codes never read under a code that is
/// not a carry, which reads wide: the map cannot hold it.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Stray {
    pub code: u16,
    pub skipped: usize,
    pub ratio: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Finding {
    pub slots: [u8; ADC_WIDE_SLOTS],
    /// Every carry, slot order.
    pub carries: Vec<Carry>,
    pub strays: Vec<Stray>,
    pub lo: u16,
    pub hi: u16,
}

/// Why the slots are not written.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Refusal {
    /// A wide carry skips more codes than a slot byte holds.
    SlotOver {
        code: u16,
        skipped: usize,
    },
    /// A carry both halves read differs by more than one unit.
    Halves {
        slot: usize,
        even: u8,
        odd: u8,
    },
    Stray(Stray),
    /// Fewer than half the carries between the stops are reached.
    FewReached {
        reached: usize,
        between: usize,
    },
}

impl Refusal {
    pub fn said(&self) -> String {
        match self {
            Refusal::SlotOver { code, skipped } => format!(
                "code {code} skips {skipped} codes, a slot of {} Q4: over a byte",
                Q4_PER_SKIPPED as usize * skipped
            ),
            Refusal::Halves { slot, even, odd } => format!(
                "slot {slot} reads {even} on the even passes and {odd} on the odd, over one unit ({Q4_PER_SKIPPED}) apart"
            ),
            Refusal::Stray(s) => format!(
                "{} codes never read over code {}, which reads {:.1}x its neighbors and is no carry: the map cannot hold it",
                s.skipped, s.code, s.ratio
            ),
            Refusal::FewReached { reached, between } => {
                format!(
                    "{reached} of the {between} carries between the stops are reached, under half"
                )
            }
        }
    }
}

/// Rows a code, codes past the converter's span left out.
pub fn histogram(codes: &[u16]) -> Vec<u32> {
    let mut h = vec![0u32; CODES];
    for &c in codes {
        if let Some(n) = h.get_mut(c as usize) {
            *n += 1;
        }
    }
    h
}

/// The wide codes of raw pot rows `codes` swept from `lo` to `hi`.
pub fn find(codes: &[u16], lo: u16, hi: u16) -> Result<Finding, Refusal> {
    find_in(&histogram(codes), lo, hi, THIN_ROWS)
}

/// [`find`] on a histogram `h` (rows a code, [`CODES`] long), a carry with
/// under `floor` reference rows a code thin.
pub fn find_in(h: &[u32], lo: u16, hi: u16, floor: f64) -> Result<Finding, Refusal> {
    let rows = |c: usize| h.get(c).copied().unwrap_or(0) as f64;
    let a = lo as usize + EDGE_MARGIN_COUNTS;
    let b = (hi as usize).saturating_sub(EDGE_MARGIN_COUNTS);
    let mut slots = [0u8; ADC_WIDE_SLOTS];
    let mut carries = Vec::with_capacity(ADC_WIDE_SLOTS);
    for (k, slot) in slots.iter_mut().enumerate() {
        let c = (k << ADC_WIDE_SHIFT) | ADC_WIDE_MASK as usize;
        let mut carry = Carry {
            slot: k,
            code: c as u16,
            state: State::Unreached,
            skipped: 0,
            ratio: 0.0,
            rows_ref: 0.0,
        };
        if a + REF_SIDE_CODES <= c && c + REACH_RUN_CODES + REF_SIDE_CODES <= b {
            let mut s = 0;
            while s < RUN_MAX_CODES && rows(c + 1 + s) == 0.0 {
                s += 1;
            }
            let reference: Vec<f64> = (c - REF_SIDE_CODES..c)
                .chain(c + 1 + s..c + 1 + s + REF_SIDE_CODES)
                .map(rows)
                .collect();
            let n = median(&reference).unwrap_or(0.0);
            carry.skipped = s;
            carry.rows_ref = n;
            carry.ratio = if n > 0.0 { rows(c) / n } else { 0.0 };
            carry.state = if n < floor {
                State::Thin
            } else if s >= MIN_SKIPPED && carry.ratio >= WIDE_MIN_SHARE * (1 + s) as f64 {
                State::Wide
            } else {
                State::NotWide
            };
            if carry.state == State::Wide {
                let q4 = Q4_PER_SKIPPED as usize * s;
                *slot = u8::try_from(q4).map_err(|_| Refusal::SlotOver {
                    code: c as u16,
                    skipped: s,
                })?;
            }
        }
        carries.push(carry);
    }
    Ok(Finding {
        slots,
        carries,
        strays: strays(h, a, b, floor),
        lo,
        hi,
    })
}

/// Runs of codes never read wholly inside `a..=b` under a code that is no
/// carry and reads wide against the codes around the run.
fn strays(h: &[u32], a: usize, b: usize, floor: f64) -> Vec<Stray> {
    let rows = |c: usize| h.get(c).copied().unwrap_or(0) as f64;
    let mut out = Vec::new();
    let mut c = a + 1;
    while c <= b {
        if rows(c) != 0.0 {
            c += 1;
            continue;
        }
        let start = c;
        while c <= b && rows(c) == 0.0 {
            c += 1;
        }
        if c > b {
            break;
        }
        let (under, s) = (start - 1, c - start);
        if under & ADC_WIDE_MASK as usize == ADC_WIDE_MASK as usize || s < MIN_SKIPPED {
            continue;
        }
        let reference: Vec<f64> = (under.saturating_sub(REF_SIDE_CODES).max(a)..under)
            .chain(c..(c + REF_SIDE_CODES).min(b + 1))
            .map(rows)
            .collect();
        if reference.len() < STRAY_REF_MIN_CODES {
            continue;
        }
        let n = median(&reference).unwrap_or(0.0);
        if n < floor {
            continue;
        }
        let ratio = rows(under) / n;
        if ratio >= WIDE_MIN_SHARE * (1 + s) as f64 {
            out.push(Stray {
                code: under as u16,
                skipped: s,
                ratio,
            });
        }
    }
    out
}

/// The step's refusals on a finding over all rows and its two halves
/// (odd and even passes apart, found with [`HALF_THIN_ROWS`]), the stops
/// `raw_min`/`raw_max`: a stray wide run, a carry both halves read more
/// than one unit apart, under half the carries between the stops reached.
pub fn check(
    all: &Finding,
    even: &Finding,
    odd: &Finding,
    raw_min: u16,
    raw_max: u16,
) -> Result<(), Refusal> {
    if let Some(s) = all.strays.first() {
        return Err(Refusal::Stray(*s));
    }
    let read = |c: &Carry| matches!(c.state, State::Wide | State::NotWide);
    for ((c, e), o) in all.carries.iter().zip(&even.carries).zip(&odd.carries) {
        if read(c) && read(e) && read(o) {
            let (se, so) = (even.slots[c.slot], odd.slots[c.slot]);
            if se.abs_diff(so) as u32 > Q4_PER_SKIPPED {
                return Err(Refusal::Halves {
                    slot: c.slot,
                    even: se,
                    odd: so,
                });
            }
        }
    }
    let between: Vec<&Carry> = all
        .carries
        .iter()
        .filter(|c| raw_min < c.code && c.code < raw_max)
        .collect();
    let reached = between
        .iter()
        .filter(|c| c.state != State::Unreached)
        .count();
    if 2 * reached < between.len() {
        return Err(Refusal::FewReached {
            reached,
            between: between.len(),
        });
    }
    Ok(())
}

/// A raw code read as the middle of the band it stands for, counts: its
/// slot / 16 added on a carry.
pub fn compensated(code: u16, slots: &[u8; ADC_WIDE_SLOTS]) -> f64 {
    code as f64 + wide_q4(code, slots) as f64 / 16.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 200 rows on every code from 400 to 3600, except `skip` codes over
    /// carry 7 (2047) that read none while 2047 holds their rows.
    fn sweep(skip: usize) -> Vec<u32> {
        let mut h = vec![0u32; CODES];
        for n in h.iter_mut().take(3601).skip(400) {
            *n = 200;
        }
        h[2048..2048 + skip].fill(0);
        h[2047] = 200 * (1 + skip as u32);
        h
    }

    #[test]
    fn a_stuck_carry_reads_its_slot_and_the_rest_read_zero() {
        let f = find_in(&sweep(11), 400, 3600, THIN_ROWS).expect("found");
        assert_eq!(f.slots[7], 88);
        assert_eq!(f.slots.iter().filter(|&&s| s != 0).count(), 1);
        let c7 = f.carries[7];
        assert_eq!(
            (c7.state, c7.skipped, c7.rows_ref),
            (State::Wide, 11, 200.0)
        );
        assert!((c7.ratio - 12.0).abs() < 1e-12);
        assert_eq!(f.carries[3].state, State::NotWide);
        // 255 lacks its 64 codes below inside 415, 3583 its 80 above
        // inside 3585
        for k in [0, 13, 14, 15] {
            assert_eq!(f.carries[k].state, State::Unreached, "{k}");
        }
        assert_eq!(f.carries[12].state, State::NotWide);
        assert!(f.strays.is_empty());
    }

    #[test]
    fn a_short_run_or_a_thin_stretch_is_not_wide() {
        let f = find_in(&sweep(2), 400, 3600, THIN_ROWS).expect("found");
        assert_eq!((f.carries[7].state, f.slots[7]), (State::NotWide, 0));
        let mut h = sweep(6);
        h[2047] = 200;
        let f = find_in(&h, 400, 3600, THIN_ROWS).expect("found");
        assert_eq!((f.carries[7].state, f.slots[7]), (State::NotWide, 0));
        let thin: Vec<u32> = sweep(11).iter().map(|n| n / 4).collect();
        let f = find_in(&thin, 400, 3600, THIN_ROWS).expect("found");
        assert_eq!((f.carries[7].state, f.slots[7]), (State::Thin, 0));
        assert_eq!(find_in(&thin, 400, 3600, 40.0).expect("found").slots[7], 88);
    }

    #[test]
    fn the_reference_is_local_where_the_speed_varies() {
        // three times the rows a code but over 1800..2300: the sweep's
        // median would read carry 7 at 4x, under the 6x it needs
        let mut h = sweep(11);
        for (c, n) in h.iter_mut().enumerate() {
            if !(1800..=2300).contains(&c) {
                *n *= 3;
            }
        }
        let f = find_in(&h, 400, 3600, THIN_ROWS).expect("found");
        assert_eq!((f.carries[7].state, f.slots[7]), (State::Wide, 88));
        assert_eq!(f.carries[7].rows_ref, 200.0);
    }

    #[test]
    fn a_slot_over_a_byte_and_a_stray_run_refuse() {
        let h = sweep(33);
        assert_eq!(
            find_in(&h, 400, 3600, THIN_ROWS),
            Err(Refusal::SlotOver {
                code: 2047,
                skipped: 33
            })
        );
        let mut h = sweep(11);
        h[1001..1005].fill(0);
        h[1000] = 1000;
        let f = find_in(&h, 400, 3600, THIN_ROWS).expect("found");
        assert_eq!(f.strays.len(), 1);
        assert_eq!((f.strays[0].code, f.strays[0].skipped), (1000, 4));
        assert!(matches!(
            check(&f, &f, &f, 100, 3700),
            Err(Refusal::Stray(_))
        ));
    }

    #[test]
    fn the_halves_and_the_reach_are_checked() {
        let all = find_in(&sweep(11), 400, 3600, THIN_ROWS).expect("found");
        let off = find_in(&sweep(9), 400, 3600, HALF_THIN_ROWS).expect("found");
        let near = find_in(&sweep(10), 400, 3600, HALF_THIN_ROWS).expect("found");
        assert_eq!(check(&all, &all, &near, 100, 3700), Ok(()));
        assert_eq!(
            check(&all, &all, &off, 100, 3700),
            Err(Refusal::Halves {
                slot: 7,
                even: 88,
                odd: 72
            })
        );
        let mut h = vec![0u32; CODES];
        for n in h.iter_mut().take(1700).skip(1000) {
            *n = 200;
        }
        let short = find_in(&h, 1000, 1699, THIN_ROWS).expect("found");
        assert_eq!(
            check(&short, &short, &short, 100, 3700),
            Err(Refusal::FewReached {
                reached: 2,
                between: 14
            })
        );
    }

    #[test]
    fn a_carry_reads_as_the_middle_of_its_band() {
        let mut slots = [0u8; ADC_WIDE_SLOTS];
        slots[7] = 88;
        assert_eq!(compensated(2047, &slots), 2052.5);
        assert_eq!(compensated(2046, &slots), 2046.0);
        assert_eq!(compensated(1023, &slots), 1023.0);
    }
}

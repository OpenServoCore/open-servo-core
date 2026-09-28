//! Plant stamp: the identified and calibrated values and the effective pot
//! LUT as one transaction. The host computes the stamp over the set it
//! INTENDED to write and stores it in `plant_stamp`; firmware recomputes
//! over what actually landed at every checkpoint (boot, a torque-off stamp
//! write, a LUT COMMIT, SAVE), so a write that never landed, a torn save,
//! a hand edit or a table rebuilt under old constants all read
//! `STAMP_MISMATCH`.
//!
//! Covered: the fields a fit or a cal produces, in table order, named once
//! in [`COVERED_NAMES`] and exported through the descriptor so hosts keep
//! no second list. Not covered: identity and comms, the user-owned safety
//! limits (current, thermal, undervolt, `duty_max_q15`), the raw sensor
//! screen, the winding anchor, and the RO board facts install re-seeds.
//!
//! The LUT hashes as its 256 knots LE, zeros while no table is live: a LUT
//! that falls back to identity changes the stamp by itself.
//!
//! Software CRC, not the SPI engine: SPI1 is the transport's frame-CRC
//! coprocessor and a dispatch-time share would race RX spans. ~600 B
//! bitwise at 2 wait states is ~0.6 ms, torque-off only.

use osc_protocol::crc::osc_crc_continue;

use crate::pot_lut::INTERVALS;
use crate::regions::ControlTable;

pub const TAG: &str = "osc-plant-1";
/// A stamp value never produced by [`compute`]: the servo was never stamped.
pub const UNSTAMPED: u16 = 0;

/// The covered fields, in table order.
pub const COVERED_NAMES: [&str; 35] = [
    "pos_min_phys_counts",
    "pos_max_phys_counts",
    "pos_min_soft_counts",
    "pos_max_soft_counts",
    "i_kp_q88",
    "i_ki_q412",
    "i_kaw_q412",
    "v_kp_q88",
    "v_ki_q412",
    "v_kaw_q412",
    "j_ff_q88",
    "p_kp_q88",
    "pos_deadband_counts",
    "velocity_limit_cps",
    "accel_limit_q88",
    "drive_polarity",
    "stall_omega_max_cps",
    "rtherm_omega_max_cps",
    "l1_q016",
    "l2_q88",
    "l3_q88",
    "pos_error_counts",
    "raw_min",
    "raw_max",
    "ke_uvs_per_rad",
    "r_q12",
    "recip_ke_q",
    "b_i_q313",
    "fric_fc_counts",
    "fric_fv_q016",
    "fric_breakaway_counts",
    "ke_vpc_q",
    "angle_min_cdeg",
    "angle_max_cdeg",
    "gear_ratio_centi",
];

/// One covered byte span of the table.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Span {
    pub addr: u16,
    pub width: u16,
}

impl Span {
    fn overlaps(&self, addr: u16, end: u16) -> bool {
        self.addr < end && self.addr + self.width > addr
    }
}

const fn str_eq(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

/// The named field's span, from the table's own descriptors: a name that
/// is not in the table fails the build.
const fn span_of(name: &str) -> Span {
    let fields = &ControlTable::FIELDS;
    let mut i = 0;
    while i < fields.len() {
        if str_eq(fields[i].name, name) {
            return Span {
                addr: fields[i].addr,
                width: fields[i].width,
            };
        }
        i += 1;
    }
    panic!("covered field is not in the table");
}

/// [`COVERED_NAMES`] resolved to spans at build time; only these 4 B
/// entries reach the servo image.
pub const COVERED: [Span; COVERED_NAMES.len()] = {
    let mut out = [Span { addr: 0, width: 0 }; COVERED_NAMES.len()];
    let mut i = 0;
    while i < out.len() {
        out[i] = span_of(COVERED_NAMES[i]);
        i += 1;
    }
    out
};

/// The table as the flat byte map it is (repr(C), padding-free by the
/// derive's assert, every byte initialized).
fn bytes(t: &ControlTable) -> &[u8] {
    // SAFETY: see fn doc; the borrow of `t` bounds the slice.
    unsafe {
        core::slice::from_raw_parts(
            (t as *const ControlTable).cast::<u8>(),
            core::mem::size_of::<ControlTable>(),
        )
    }
}

/// `max(1, crc16_arc(TAG ++ covered bytes ++ knots LE))`. `knots` is the
/// table the kernel applies, `None` while it applies none (identity). Cold
/// path with three callers: one copy, not one per checkpoint.
#[inline(never)]
pub fn compute(t: &ControlTable, knots: Option<&[i16; INTERVALS]>) -> u16 {
    let bytes = bytes(t);
    let mut crc = osc_crc_continue(0, TAG.as_bytes());
    for s in &COVERED {
        if let Some(b) = bytes.get(s.addr as usize..(s.addr + s.width) as usize) {
            crc = osc_crc_continue(crc, b);
        }
    }
    match knots {
        Some(k) => {
            for c in k {
                crc = osc_crc_continue(crc, &c.to_le_bytes());
            }
        }
        None => {
            for _ in 0..INTERVALS {
                crc = osc_crc_continue(crc, &[0, 0]);
            }
        }
    }
    if crc == UNSTAMPED { 1 } else { crc }
}

/// Whether a committed write `[addr, addr + len)` touched a covered field.
pub fn covers(addr: u16, len: u16) -> bool {
    let end = addr.saturating_add(len);
    COVERED.iter().any(|s| s.overlaps(addr, end))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::calib::addr::stamp::PLANT_STAMP;
    use crate::regions::config::addr::limits::CURRENT_LIMIT_COUNTS;
    use crate::regions::{CALIB_BASE_ADDR, CALIB_REGION_SIZE};

    #[test]
    fn covered_list_matches_the_named_fields() {
        for (name, span) in COVERED_NAMES.iter().zip(&COVERED) {
            let f = ControlTable::FIELDS
                .iter()
                .find(|f| f.name == *name)
                .unwrap();
            assert_eq!((f.addr, f.width), (span.addr, span.width), "{name}");
            assert!(f.writable, "{name} is host-written");
        }
        for w in COVERED.windows(2) {
            assert!(w[0].addr + w[0].width <= w[1].addr, "table order");
        }
        let hashed: u16 = COVERED.iter().map(|s| s.width).sum();
        assert_eq!(hashed, 77);
        let last = COVERED[COVERED.len() - 1];
        assert!(last.addr + last.width <= CALIB_BASE_ADDR + CALIB_REGION_SIZE);
        assert!(!covers(PLANT_STAMP, 2), "the stamp does not cover itself");
        assert!(!covers(CURRENT_LIMIT_COUNTS, 2), "user safety limit");
    }

    #[test]
    fn zero_lut_hashes_like_identity() {
        let t = ControlTable::new();
        assert_eq!(compute(&t, None), compute(&t, Some(&[0; INTERVALS])));
        let mut k = [0i16; INTERVALS];
        k[100] = 1;
        assert_ne!(compute(&t, None), compute(&t, Some(&k)));
    }

    #[test]
    fn stamp_is_never_zero_and_follows_the_covered_set() {
        let mut t = ControlTable::new();
        let empty = compute(&t, None);
        assert_ne!(empty, UNSTAMPED);
        t.calib.stamp.plant_stamp = empty;
        assert_eq!(compute(&t, None), empty, "the stamp is not covered");
        t.config.limits.current_limit_counts = 1200;
        assert_eq!(compute(&t, None), empty, "user limits are not covered");
        t.calib.motor.recip_ke_q = 3700;
        let ke = compute(&t, None);
        assert_ne!(ke, empty);
        t.config.pos_limits.pos_max_phys_counts = 4095;
        assert_ne!(compute(&t, None), ke);
    }

    #[test]
    fn covers_reports_every_intersecting_span() {
        assert!(covers(COVERED[0].addr, 1));
        assert!(covers(COVERED[0].addr + 3, 1));
        assert!(covers(COVERED[0].addr.wrapping_sub(1), 2));
        assert!(!covers(COVERED[0].addr.wrapping_sub(1), 1));
        assert!(covers(0, 1024), "a whole-table write");
        assert!(!covers(0x200, 0x100), "telemetry");
    }
}

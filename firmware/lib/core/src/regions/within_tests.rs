//! Each fused `within` field against the rules it was declared with before
//! the fusion: the table's rules and the old ones give the same verdict and
//! the same error for every value swept, under every bound configuration.

use control_table::map::View;
use control_table::rules::{self, CmpRule, OP_GE, OP_GT, OP_IN, OP_LE, OP_LT, SPEC_RHS_REG};
use control_table::{Error, RegisterMap};

use super::config::addr::common::ID;
use super::config::addr::pos_phys::{POS_MAX_PHYS_COUNTS, POS_MIN_PHYS_COUNTS};
use super::config::addr::pos_soft::{POS_MAX_SOFT_COUNTS, POS_MIN_SOFT_COUNTS};
use super::control::addr::lifecycle::GOAL_POSITION;
use crate::kernel::phases::loops::trajectory::GOAL_LIM_COUNTS;

type C = crate::ControlTableCell;
const SIZE: usize = <C as RegisterMap>::SIZE;

const fn imm(addr: u16, width: u8, signed: bool, op: u8, v: i32) -> CmpRule {
    CmpRule {
        addr,
        spec: rules::spec(width, signed, false, op),
        val: v as u32,
    }
}

const fn reg(addr: u16, op: u8, rhs: u16) -> CmpRule {
    CmpRule {
        addr,
        spec: rules::spec(4, true, false, op) | SPEC_RHS_REG,
        val: rhs as u32,
    }
}

/// The i32 fields, each with its rules as declared before the fusion.
const OLD_I32: [(u16, &[CmpRule]); 3] = [
    (
        GOAL_POSITION,
        &[
            reg(GOAL_POSITION, OP_GE, POS_MIN_PHYS_COUNTS),
            reg(GOAL_POSITION, OP_LE, POS_MAX_PHYS_COUNTS),
        ],
    ),
    (
        POS_MIN_SOFT_COUNTS,
        &[
            reg(POS_MIN_SOFT_COUNTS, OP_GE, POS_MIN_PHYS_COUNTS),
            reg(POS_MIN_SOFT_COUNTS, OP_LE, POS_MAX_PHYS_COUNTS),
            reg(POS_MIN_SOFT_COUNTS, OP_LT, POS_MAX_SOFT_COUNTS),
            imm(POS_MIN_SOFT_COUNTS, 4, true, OP_GE, -GOAL_LIM_COUNTS),
            imm(POS_MIN_SOFT_COUNTS, 4, true, OP_LE, GOAL_LIM_COUNTS),
        ],
    ),
    (
        POS_MAX_SOFT_COUNTS,
        &[
            reg(POS_MAX_SOFT_COUNTS, OP_GE, POS_MIN_PHYS_COUNTS),
            reg(POS_MAX_SOFT_COUNTS, OP_LE, POS_MAX_PHYS_COUNTS),
            reg(POS_MAX_SOFT_COUNTS, OP_GT, POS_MIN_SOFT_COUNTS),
            imm(POS_MAX_SOFT_COUNTS, 4, true, OP_GE, -GOAL_LIM_COUNTS),
            imm(POS_MAX_SOFT_COUNTS, 4, true, OP_LE, GOAL_LIM_COUNTS),
        ],
    ),
];

/// `[min_phys, max_phys, min_soft, max_soft]` as stored: the bench rails,
/// degenerate, inverted, and the i32 extremes.
const CONFIGS: [[i32; 4]; 7] = [
    [0, 4095, 0, 4095],
    [-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS, -100, 100],
    [100, 100, 100, 100],
    [4095, 0, 4095, 0],
    [i32::MIN, i32::MAX, i32::MIN, i32::MAX],
    [i32::MAX, i32::MIN, 0, 0],
    [-1, 1, 1, -1],
];

/// Values swept on each side of every bound: a compare's verdict flips only
/// at its bound, so agreement across each bound's window is agreement over
/// the whole range; the stride walk checks that between the windows.
const EDGE: i32 = 256;
const STRIDE: usize = 65_521;

/// The table's rules on `field`, address-ordered and contiguous.
fn fused(field: u16) -> &'static [CmpRule] {
    let all = <C as RegisterMap>::CMP_RULES;
    let start = all
        .iter()
        .position(|r| r.addr == field)
        .unwrap_or(all.len());
    let n = all[start..].iter().take_while(|r| r.addr == field).count();
    &all[start..start + n]
}

fn verdict(set: &[CmpRule], storage: &[u8], field: u16, value: &[u8]) -> Result<(), Error> {
    let view = View::new(storage.as_ptr(), storage.len(), field, value);
    set.iter()
        .try_for_each(|r| rules::check_cmp(&view, r.addr, r.spec, r.val))
}

fn storage_for(cfg: &[i32; 4]) -> [u8; SIZE] {
    let mut s = [0u8; SIZE];
    let addrs = [
        POS_MIN_PHYS_COUNTS,
        POS_MAX_PHYS_COUNTS,
        POS_MIN_SOFT_COUNTS,
        POS_MAX_SOFT_COUNTS,
    ];
    for (a, v) in addrs.iter().zip(cfg) {
        s[*a as usize..*a as usize + 4].copy_from_slice(&v.to_le_bytes());
    }
    s
}

fn assert_same(field: u16, old: &[CmpRule], storage: &[u8], v: i32) {
    let b = v.to_le_bytes();
    assert_eq!(
        verdict(fused(field), storage, field, &b),
        verdict(old, storage, field, &b),
        "field {field:#x} value {v}"
    );
}

#[test]
fn fused_fields_hold_within_rules() {
    let shape = |field: u16| -> heapless::Vec<u8, 4> {
        fused(field).iter().map(|r| (r.spec >> 8) as u8).collect()
    };
    assert_eq!(shape(ID), [OP_IN]);
    assert_eq!(shape(GOAL_POSITION), [OP_IN]);
    assert_eq!(shape(POS_MIN_SOFT_COUNTS), [OP_IN, OP_LT, OP_IN]);
    assert_eq!(shape(POS_MAX_SOFT_COUNTS), [OP_IN, OP_GT, OP_IN]);
}

#[test]
fn id_within_matches_its_pair_over_every_value() {
    let old = [imm(ID, 1, false, OP_GE, 1), imm(ID, 1, false, OP_LE, 249)];
    let storage = [0u8; SIZE];
    for v in 0..=u8::MAX {
        assert_eq!(
            verdict(fused(ID), &storage, ID, &[v]),
            verdict(&old, &storage, ID, &[v]),
            "id {v}"
        );
    }
}

#[test]
fn i32_withins_match_their_pairs_across_every_bound() {
    for cfg in &CONFIGS {
        let storage = storage_for(cfg);
        let edges =
            cfg.iter()
                .copied()
                .chain([-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS, 0, i32::MIN, i32::MAX]);
        for (field, old) in OLD_I32 {
            for e in edges.clone() {
                for v in e.saturating_sub(EDGE)..=e.saturating_add(EDGE) {
                    assert_same(field, old, &storage, v);
                }
            }
            for v in (0..=u32::MAX).step_by(STRIDE) {
                assert_same(field, old, &storage, v as i32);
            }
        }
    }
}

/// Every i32 value at the bench rails. Minutes in release:
/// `cargo test --release -p osc-servo-core within_full_range -- --ignored`.
#[test]
#[ignore]
fn within_full_range() {
    let storage = storage_for(&CONFIGS[0]);
    for (field, old) in OLD_I32 {
        for v in i32::MIN..=i32::MAX {
            assert_same(field, old, &storage, v);
        }
    }
}

#![feature(sync_unsafe_cell)]

use control_table::rules::{CmpOp, Rhs, RuleKind};
use control_table::{Enum, Error, FlatBlock, FlatTable, RegisterFile, RegisterMap, ValidationKind};
use core::mem::{offset_of, size_of};

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug, Enum)]
pub enum Mode {
    Idle = 0,
    Run = 1,
}

// --- Blocks --------------------------------------------------------------

#[repr(C)]
#[derive(FlatBlock)]
pub struct Ident {
    #[ct_field(access = ro)]
    pub model: u16,
    pub id: u8,
    pub lock: u8,
}

#[repr(C)]
#[derive(FlatBlock)]
pub struct Limits {
    #[ct_field(le = 100u8)]
    pub max_ratio: u8,
    pub mode: Mode,
    pub enabled: bool,
    pub _rsvd: u8,
}

#[repr(C)]
#[derive(FlatBlock)]
#[ct_block(hooks = GoalHooks)]
pub struct Goal {
    // Cross-section compare: goal stays <= the config ceiling byte.
    #[ct_field(le = &config::addr::limits::MAX_RATIO)]
    pub target: u8,
    #[ct_field(hook = on_target)]
    pub commit: u8,
}

pub trait GoalHooks {
    fn on_target(&mut self, v: u8);
}

// --- Sections ------------------------------------------------------------

mod config {
    use super::{Ident, Limits};
    use control_table::Section;

    #[repr(C)]
    #[derive(Section)]
    #[ct_section(base = 0x0000, size = 12)]
    pub struct Config {
        pub ident: Ident,
        pub limits: Limits,
        #[ct_section(skip)]
        pub _rsvd: [u8; 4],
    }
}

mod control {
    use super::Goal;
    use control_table::Section;

    #[repr(C)]
    #[derive(Section)]
    #[ct_section(
        base = 0x000C,
        size = 4,
        write_locked_by = super::config::addr::ident::LOCK,
        hooks = super::GoalHooks,
    )]
    pub struct Control {
        pub goal: Goal,
        #[ct_section(skip)]
        pub _rsvd: [u8; 2],
    }
}

use config::Config;
use control::Control;

#[repr(C)]
#[derive(FlatTable)]
#[ct_table(size = 24, hooks = GoalHooks)]
struct Table {
    pub config: Config,
    pub control: Control,
    #[ct_table(skip)]
    pub _rsvd: [u8; 8],
}

// -------------------------------------------------------------------------

#[test]
fn addr_consts_are_base_plus_offset() {
    assert_eq!(
        addr::config::ident::MODEL,
        Config::BASE + offset_of!(Config, ident) as u16 + offset_of!(Ident, model) as u16
    );
    assert_eq!(
        addr::config::ident::LOCK,
        Config::BASE + offset_of!(Config, ident) as u16 + offset_of!(Ident, lock) as u16
    );
    assert_eq!(
        addr::config::limits::MAX_RATIO,
        Config::BASE + offset_of!(Config, limits) as u16 + offset_of!(Limits, max_ratio) as u16
    );
    assert_eq!(
        addr::control::goal::TARGET,
        Control::BASE + offset_of!(Control, goal) as u16 + offset_of!(Goal, target) as u16
    );
}

#[test]
fn section_base_and_size_consts() {
    assert_eq!(Config::BASE, 0x0000);
    assert_eq!(Config::SECTION_SIZE, 12);
    assert_eq!(Control::BASE, 0x000C);
    assert_eq!(Control::SECTION_SIZE, 4);
}

#[test]
fn config_rules_rebased_to_absolute() {
    // Field order: ident (no rules) then limits (compare, enum, bool).
    let r = Config::CT_RULES_ABS;
    assert_eq!(r.len(), 3);

    assert_eq!(r[0].offset, addr::config::limits::MAX_RATIO);
    match r[0].kind {
        RuleKind::Cmp {
            op: CmpOp::Le,
            rhs: Rhs::Imm(100),
            ..
        } => {}
        _ => panic!("wrong max_ratio rule"),
    }

    assert_eq!(r[1].offset, addr::config::limits::MODE);
    match r[1].kind {
        RuleKind::Enum { allowed } => assert_eq!(allowed, Mode::ALLOWED),
        _ => panic!("wrong mode rule"),
    }

    assert_eq!(r[2].offset, addr::config::limits::ENABLED);
    assert!(matches!(r[2].kind, RuleKind::Enum { .. }));
}

#[test]
fn control_rule_is_cross_section_reg_compare() {
    let c = Control::CT_RULES_ABS;
    assert_eq!(c.len(), 1);
    assert_eq!(c[0].offset, addr::control::goal::TARGET);
    match c[0].kind {
        RuleKind::Cmp {
            op: CmpOp::Le,
            rhs: Rhs::Reg(a),
            ..
        } => assert_eq!(a, addr::config::limits::MAX_RATIO),
        _ => panic!("wrong control rule"),
    }
}

#[test]
fn writable_words_track_ro_rw_and_reserved() {
    let words = Table::WRITABLE_WORDS;
    assert_eq!(words.len(), 24usize.div_ceil(32));

    let bit = |a: u16| (words[a as usize / 32] >> (a as usize % 32)) & 1 == 1;

    // ro model bytes clear.
    assert!(!bit(addr::config::ident::MODEL));
    assert!(!bit(addr::config::ident::MODEL + 1));
    // rw bytes set.
    assert!(bit(addr::config::ident::ID));
    assert!(bit(addr::config::ident::LOCK));
    assert!(bit(addr::config::limits::MAX_RATIO));
    assert!(bit(addr::control::goal::TARGET));
    // reserved / skip tails clear.
    assert!(!bit(8)); // config._rsvd tail
    assert!(!bit(14)); // control._rsvd tail
    assert!(!bit(20)); // table-level _rsvd
}

#[test]
fn sections_metadata_exposed_through_register_map() {
    let secs = <TableCell as RegisterMap>::SECTIONS;
    assert_eq!(secs.len(), 2);

    assert_eq!(secs[0].base, Config::BASE);
    assert_eq!(secs[0].size, Config::SECTION_SIZE);
    assert_eq!(secs[0].rules.len(), Config::CT_RULES_ABS.len());
    assert!(secs[0].write_lock.is_none());

    assert_eq!(secs[1].base, Control::BASE);
    assert_eq!(secs[1].size, Control::SECTION_SIZE);
    assert_eq!(secs[1].rules.len(), Control::CT_RULES_ABS.len());
    assert_eq!(secs[1].write_lock, Some(addr::config::ident::LOCK));

    assert_eq!(<TableCell as RegisterMap>::SIZE, 24);
}

fn fresh() -> TableCell {
    TableCell::new()
}

#[test]
fn read_round_trips_written_bytes() {
    let t = fresh();
    t.write(addr::config::ident::ID, &[0x42]).unwrap();
    assert_eq!(t.read(addr::config::ident::ID, 1).unwrap(), &[0x42]);
}

#[test]
fn write_to_ro_byte_rejected_by_mask() {
    let t = fresh();
    let err = t.write(addr::config::ident::MODEL, &[1, 2]).unwrap_err();
    assert_eq!(err, Error::AccessError);
}

#[test]
fn write_rejected_by_enum_rule() {
    let t = fresh();
    let err = t.write(addr::config::limits::MODE, &[9]).unwrap_err();
    assert_eq!(err, Error::ValidationError(ValidationKind::Enum));
    t.write(addr::config::limits::MODE, &[Mode::Run as u8])
        .unwrap();
}

#[test]
fn write_rejected_by_compare_rule() {
    let t = fresh();
    let err = t
        .write(addr::config::limits::MAX_RATIO, &[200])
        .unwrap_err();
    assert_eq!(err, Error::ValidationError(ValidationKind::Compare));
    t.write(addr::config::limits::MAX_RATIO, &[100]).unwrap();
}

#[test]
fn cross_section_compare_reads_config_ceiling() {
    let t = fresh();
    t.write(addr::config::limits::MAX_RATIO, &[50]).unwrap();
    assert_eq!(
        t.write(addr::control::goal::TARGET, &[80]).unwrap_err(),
        Error::ValidationError(ValidationKind::Compare)
    );
    t.write(addr::control::goal::TARGET, &[40]).unwrap();
    assert_eq!(t.read(addr::control::goal::TARGET, 1).unwrap(), &[40]);
}

#[test]
fn write_lock_blocks_locked_section_only() {
    let t = fresh();
    // Raise the ceiling so target=1 clears the compare rule; the lock (not the
    // rule) must be what rejects it. Rules are evaluated before the lock check.
    t.write(addr::config::limits::MAX_RATIO, &[100]).unwrap();
    t.write(addr::config::ident::LOCK, &[1]).unwrap();
    assert_eq!(
        t.write(addr::control::goal::TARGET, &[1]).unwrap_err(),
        Error::ValidationError(ValidationKind::Locked)
    );
    // config itself is not gated by the lock.
    t.write(addr::config::ident::ID, &[7]).unwrap();

    t.write(addr::config::ident::LOCK, &[0]).unwrap();
    t.write(addr::control::goal::TARGET, &[1]).unwrap();
    assert_eq!(t.read(addr::control::goal::TARGET, 1).unwrap(), &[1]);
}

struct Rec {
    hit: Option<u8>,
}

impl GoalHooks for Rec {
    fn on_target(&mut self, v: u8) {
        self.hit = Some(v);
    }
}

#[test]
fn hook_dispatches_through_table_for_hooked_field() {
    let mut tbl = Table::new();
    tbl.control.goal.commit = 0xAB;

    let mut rec = Rec { hit: None };
    tbl.dispatch_events(addr::control::goal::COMMIT, 1, &mut rec);
    assert_eq!(rec.hit, Some(0xAB));

    // A window entirely inside config must not fire the control hook.
    let mut miss = Rec { hit: None };
    tbl.dispatch_events(addr::config::ident::ID, 1, &mut miss);
    assert_eq!(miss.hit, None);
}

#[test]
fn new_zero_initializes_including_skips() {
    let t = Table::new();
    assert_eq!(t.config.ident.model, 0);
    assert_eq!(t.config.limits.max_ratio, 0);
    assert_eq!(t.config.limits.mode, Mode::Idle);
    assert!(!t.config.limits.enabled);
    assert_eq!(t.control.goal.target, 0);
    assert_eq!(t._rsvd, [0u8; 8]);
    assert_eq!(size_of::<Table>(), 24);
}

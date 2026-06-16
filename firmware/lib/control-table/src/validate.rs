use crate::desc::{
    BlockDesc, CompareOp, Error, FieldValidator, RegionValidator, Rhs, ValidationKind,
};
use crate::route::Router;
use crate::stage::{Snapshot, StagedView, StagedWrites};

impl CompareOp {
    pub fn apply<T: PartialOrd>(self, a: &T, b: &T) -> bool {
        match self {
            CompareOp::Lt => a < b,
            CompareOp::Le => a <= b,
            CompareOp::Gt => a > b,
            CompareOp::Ge => a >= b,
            CompareOp::Eq => a == b,
            CompareOp::Ne => a != b,
        }
    }
}

impl FieldValidator {
    pub fn run(&self, view: &StagedView, addr: u16, size: u16) -> Result<(), Error> {
        match self {
            FieldValidator::EnumU8 { allowed } => {
                let mut b = [0u8; 1];
                view.read_bytes(addr, &mut b)?;
                if allowed.contains(&b[0]) {
                    Ok(())
                } else {
                    Err(Error::ValidationError(ValidationKind::Enum))
                }
            }
            FieldValidator::CompareU8 { op, abs, rhs } => {
                run_compare_i32(view, addr, *op, *abs, widen(rhs), 1, false, u8::MAX as i32)
            }
            FieldValidator::CompareU16 { op, abs, rhs } => {
                run_compare_i32(view, addr, *op, *abs, widen(rhs), 2, false, u16::MAX as i32)
            }
            FieldValidator::CompareI8 { op, abs, rhs } => {
                run_compare_i32(view, addr, *op, *abs, widen(rhs), 1, true, i8::MAX as i32)
            }
            FieldValidator::CompareI16 { op, abs, rhs } => {
                run_compare_i32(view, addr, *op, *abs, widen(rhs), 2, true, i16::MAX as i32)
            }
            FieldValidator::CompareI32 { op, abs, rhs } => {
                run_compare_i32(view, addr, *op, *abs, widen(rhs), 4, true, i32::MAX)
            }
            FieldValidator::Custom(f) => f(view, addr, size),
        }
    }
}

enum RhsI32 {
    Value(i32),
    Addr(u16),
}

trait WidenI32: Copy {
    fn widen(self) -> i32;
}
impl WidenI32 for u8 {
    fn widen(self) -> i32 {
        self as i32
    }
}
impl WidenI32 for u16 {
    fn widen(self) -> i32 {
        self as i32
    }
}
impl WidenI32 for i8 {
    fn widen(self) -> i32 {
        self as i32
    }
}
impl WidenI32 for i16 {
    fn widen(self) -> i32 {
        self as i32
    }
}
impl WidenI32 for i32 {
    fn widen(self) -> i32 {
        self
    }
}

fn widen<T: WidenI32>(rhs: &Rhs<T>) -> RhsI32 {
    match rhs {
        Rhs::Value(v) => RhsI32::Value(v.widen()),
        Rhs::Addr(a) => RhsI32::Addr(*a),
    }
}

#[inline(never)]
fn read_widened(view: &StagedView, addr: u16, width: u8, signed: bool) -> Result<i32, Error> {
    let mut b = [0u8; 4];
    view.read_bytes(addr, &mut b[..width as usize])?;
    Ok(match (width, signed) {
        (1, false) => b[0] as i32,
        (1, true) => b[0] as i8 as i32,
        (2, false) => u16::from_le_bytes([b[0], b[1]]) as i32,
        (2, true) => i16::from_le_bytes([b[0], b[1]]) as i32,
        _ => i32::from_le_bytes(b),
    })
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn run_compare_i32(
    view: &StagedView,
    self_addr: u16,
    op: CompareOp,
    abs: bool,
    rhs: RhsI32,
    width: u8,
    signed: bool,
    sat_max: i32,
) -> Result<(), Error> {
    let mut a = read_widened(view, self_addr, width, signed)?;
    let mut b = match rhs {
        RhsI32::Value(v) => v,
        RhsI32::Addr(other) => read_widened(view, other, width, signed)?,
    };
    if abs && signed {
        a = a.saturating_abs().min(sat_max);
        b = b.saturating_abs().min(sat_max);
    }
    if op.apply(&a, &b) {
        Ok(())
    } else {
        Err(Error::ValidationError(ValidationKind::Compare))
    }
}

pub(crate) fn run_field_validators(
    router: &dyn Router,
    staged: &StagedWrites,
    snap: Snapshot,
    abs_start: u16,
    len: usize,
    blocks: &[BlockDesc],
) -> Result<(), Error> {
    let view = StagedView::new(router, staged, snap);
    let req_lo = abs_start as usize;
    let req_hi = req_lo + len;
    for block in blocks {
        let b_lo = block.addr as usize;
        let b_hi = b_lo + block.size as usize;
        if b_hi <= req_lo {
            continue;
        }
        if b_lo >= req_hi {
            break;
        }
        for field in block.fields {
            let f_lo = field.addr as usize;
            let f_hi = f_lo + field.size as usize;
            if f_hi <= req_lo {
                continue;
            }
            if f_lo >= req_hi {
                break;
            }
            if field.validators.is_empty() {
                continue;
            }
            for v in field.validators {
                v.run(&view, field.addr, field.size)?;
            }
        }
    }
    Ok(())
}

pub(crate) fn run_block_validators(
    router: &dyn Router,
    staged: &StagedWrites,
    snap: Snapshot,
    abs_start: u16,
    len: usize,
    blocks: &[BlockDesc],
) -> Result<(), Error> {
    let req_lo = abs_start as usize;
    let req_hi = req_lo + len;
    let mut view: Option<StagedView> = None;
    for block in blocks {
        let b_lo = block.addr as usize;
        let b_hi = b_lo + block.size as usize;
        if b_hi <= req_lo {
            continue;
        }
        if b_lo >= req_hi {
            break;
        }
        if block.validators.is_empty() {
            continue;
        }
        let v = view.get_or_insert_with(|| StagedView::new(router, staged, snap));
        for f in block.validators {
            f(v, block.addr, block.size)?;
        }
    }
    Ok(())
}

pub(crate) fn run_region_validators(
    router: &dyn Router,
    staged: &StagedWrites,
    snap: Snapshot,
    validators: &[RegionValidator],
) -> Result<(), Error> {
    if validators.is_empty() {
        return Ok(());
    }
    let view = StagedView::new(router, staged, snap);
    for v in validators {
        v(&view)?;
    }
    Ok(())
}

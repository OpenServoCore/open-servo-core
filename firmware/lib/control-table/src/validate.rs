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
                let b = read_le(view, addr, |b: [u8; 1]| b[0])?;
                if allowed.contains(&b) {
                    Ok(())
                } else {
                    Err(Error::ValidationError(ValidationKind::Enum))
                }
            }
            FieldValidator::CompareU8 { op, abs, rhs } => {
                run_compare(view, addr, *op, *abs, *rhs, |b: [u8; 1]| b[0], |v| v)
            }
            FieldValidator::CompareU16 { op, abs, rhs } => {
                run_compare(view, addr, *op, *abs, *rhs, u16::from_le_bytes, |v| v)
            }
            FieldValidator::CompareI8 { op, abs, rhs } => run_compare(
                view,
                addr,
                *op,
                *abs,
                *rhs,
                |b: [u8; 1]| b[0] as i8,
                i8::saturating_abs,
            ),
            FieldValidator::CompareI16 { op, abs, rhs } => run_compare(
                view,
                addr,
                *op,
                *abs,
                *rhs,
                i16::from_le_bytes,
                i16::saturating_abs,
            ),
            FieldValidator::CompareI32 { op, abs, rhs } => run_compare(
                view,
                addr,
                *op,
                *abs,
                *rhs,
                i32::from_le_bytes,
                i32::saturating_abs,
            ),
            FieldValidator::Custom(f) => f(view, addr, size),
        }
    }
}

fn read_le<T, const N: usize>(
    view: &StagedView,
    addr: u16,
    decode: fn([u8; N]) -> T,
) -> Result<T, Error> {
    let mut b = [0u8; N];
    view.read_bytes(addr, &mut b)?;
    Ok(decode(b))
}

fn run_compare<T: PartialOrd + Copy, const N: usize>(
    view: &StagedView,
    self_addr: u16,
    op: CompareOp,
    abs: bool,
    rhs: Rhs<T>,
    decode: fn([u8; N]) -> T,
    saturating_abs: fn(T) -> T,
) -> Result<(), Error> {
    let mut a = read_le(view, self_addr, decode)?;
    let mut b = match rhs {
        Rhs::Value(v) => v,
        Rhs::Addr(other) => read_le(view, other, decode)?,
    };
    if abs {
        a = saturating_abs(a);
        b = saturating_abs(b);
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

use crate::desc::{Error, ValidationKind};
use crate::map::View;

pub struct Rule {
    pub offset: u16,
    pub width: u8,
    pub kind: RuleKind,
}

pub enum RuleKind {
    Enum {
        allowed: &'static [u8],
    },
    Cmp {
        op: CmpOp,
        rhs: Rhs,
        signed: bool,
        abs: bool,
    },
}

pub enum Rhs {
    Imm(i32),
    Reg(u16),
}

#[derive(Copy, Clone)]
pub enum CmpOp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

impl CmpOp {
    pub fn apply(self, a: i32, b: i32) -> bool {
        match self {
            CmpOp::Lt => a < b,
            CmpOp::Le => a <= b,
            CmpOp::Gt => a > b,
            CmpOp::Ge => a >= b,
            CmpOp::Eq => a == b,
            CmpOp::Ne => a != b,
        }
    }
}

impl Rule {
    pub fn eval(&self, view: &View) -> Result<(), Error> {
        match &self.kind {
            RuleKind::Enum { allowed } => {
                let mut b = [0u8; 1];
                view.read_into(self.offset, &mut b)?;
                if allowed.contains(&b[0]) {
                    Ok(())
                } else {
                    Err(Error::ValidationError(ValidationKind::Enum))
                }
            }
            RuleKind::Cmp {
                op,
                rhs,
                signed,
                abs,
            } => {
                let mut a = read_widened(view, self.offset, self.width, *signed)?;
                let mut b = match rhs {
                    Rhs::Imm(v) => *v,
                    Rhs::Reg(other) => read_widened(view, *other, self.width, *signed)?,
                };
                if *abs && *signed {
                    // Match validate.rs: clamp the abs result back to the field
                    // width's signed max (saturating_abs of MIN overflows the width).
                    let sat_max = match self.width {
                        1 => i8::MAX as i32,
                        2 => i16::MAX as i32,
                        _ => i32::MAX,
                    };
                    a = a.saturating_abs().min(sat_max);
                    b = b.saturating_abs().min(sat_max);
                }
                if op.apply(a, b) {
                    Ok(())
                } else {
                    Err(Error::ValidationError(ValidationKind::Compare))
                }
            }
        }
    }
}

fn read_widened(view: &View, addr: u16, width: u8, signed: bool) -> Result<i32, Error> {
    let mut b = [0u8; 4];
    view.read_into(addr, &mut b[..width as usize])?;
    Ok(match (width, signed) {
        (1, false) => b[0] as i32,
        (1, true) => b[0] as i8 as i32,
        (2, false) => u16::from_le_bytes([b[0], b[1]]) as i32,
        (2, true) => i16::from_le_bytes([b[0], b[1]]) as i32,
        _ => i32::from_le_bytes(b),
    })
}

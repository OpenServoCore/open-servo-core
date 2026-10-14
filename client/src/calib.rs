//! Calibration fields a host reads by meaning rather than by number.

use osc_protocol::wire::Id;

use crate::client::Client;
use crate::descriptor::Descriptor;
use crate::error::{Error, LinkError};
use crate::pipe::Pipe;

/// `gear_class`: the gear train's material, written by `osc cal`. A byte
/// outside the known codes reads `Unknown`, as a never-calibrated servo's 0
/// does.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GearClass {
    Unknown,
    Plastic,
    Metal,
}

impl GearClass {
    pub const fn from_byte(b: u8) -> Self {
        match b {
            1 => GearClass::Plastic,
            2 => GearClass::Metal,
            _ => GearClass::Unknown,
        }
    }

    pub const fn byte(self) -> u8 {
        match self {
            GearClass::Unknown => 0,
            GearClass::Plastic => 1,
            GearClass::Metal => 2,
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            GearClass::Unknown => "unknown",
            GearClass::Plastic => "plastic",
            GearClass::Metal => "metal",
        }
    }
}

pub async fn gear_class<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
) -> Result<GearClass, Error> {
    let f = d
        .field("gear_class")
        .ok_or_else(|| Error::Descriptor(format!("no gear_class in {}", d.model)))?;
    let b = c.read_span(id, f.addr, f.end()).await?;
    let byte = b
        .first()
        .copied()
        .ok_or_else(|| Error::Link(LinkError::Desync("short gear_class".into())))?;
    Ok(GearClass::from_byte(byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gear_class_decodes_its_codes_and_reads_anything_else_unknown() {
        for g in [GearClass::Unknown, GearClass::Plastic, GearClass::Metal] {
            assert_eq!(GearClass::from_byte(g.byte()), g);
        }
        assert_eq!(GearClass::from_byte(0), GearClass::Unknown);
        assert_eq!(GearClass::from_byte(1), GearClass::Plastic);
        assert_eq!(GearClass::from_byte(2), GearClass::Metal);
        assert_eq!(GearClass::from_byte(3), GearClass::Unknown);
        assert_eq!(GearClass::from_byte(0xFF), GearClass::Unknown);
        assert_eq!(GearClass::Metal.name(), "metal");
    }
}

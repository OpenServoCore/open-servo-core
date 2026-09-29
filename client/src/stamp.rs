//! Plant stamp, the host half of the firmware's `stamp` module: the
//! identified and calibrated values and the effective position table as one
//! transaction. A host writes the set, computes the stamp over it and
//! stores it in `plant_stamp`; the firmware recomputes at every torque-off
//! checkpoint and reads `STAMP_MISMATCH` (`data_state`) when the two
//! differ.
//!
//! Bit-identical to the firmware by construction: the recipe comes from
//! the descriptor's `stamp` block (tag, covered names in table order,
//! point count), the bytes are the fields' own table bytes at descriptor
//! width, and the CRC is the protocol's CRC-16/ARC. The points are the
//! table the kernel applies (`pos_lut::effective`): the array while
//! `pos_lut_state` is LIVE, zeros otherwise.

use std::fmt;
use std::time::Duration;

use osc_protocol::crc::osc_crc_continue;
use osc_protocol::wire::Id;

use crate::client::Client;
use crate::data_state::STAMP_MISMATCH;
use crate::descriptor::{Descriptor, Field};
use crate::error::{Error, LinkError};
use crate::pipe::Pipe;
use crate::pos_lut;

/// A value [`Stamp::compute`] never produces: the servo was never stamped.
pub const UNSTAMPED: u16 = 0;

/// The descriptor's recipe resolved to fields.
pub struct Stamp<'a> {
    tag: &'a str,
    covered: Vec<&'a Field>,
    pos_lut_points: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StampError {
    /// `bytes` does not reach the named field.
    Short(String),
    LutPoints {
        want: usize,
        got: usize,
    },
}

impl fmt::Display for StampError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StampError::Short(name) => write!(f, "table bytes stop before {name}"),
            StampError::LutPoints { want, got } => {
                write!(f, "{want} lut points expected, got {got}")
            }
        }
    }
}

impl std::error::Error for StampError {}

impl Descriptor {
    pub fn stamp(&self) -> Result<Stamp<'_>, Error> {
        let spec = self
            .stamp
            .as_ref()
            .ok_or_else(|| Error::Descriptor(format!("{} carries no stamp recipe", self.model)))?;
        let covered = spec
            .covered
            .iter()
            .map(|n| {
                self.field(n).ok_or_else(|| {
                    Error::Descriptor(format!("covered field {n} is not in {}", self.model))
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Stamp {
            tag: &spec.tag,
            covered,
            pos_lut_points: spec.pos_lut_points,
        })
    }
}

impl<'a> Stamp<'a> {
    pub fn covered(&self) -> &[&'a Field] {
        &self.covered
    }

    /// Whether a write `[addr, addr + len)` touches a covered field: the
    /// firmware marks `STAMP_MISMATCH` on it.
    pub fn covers(&self, addr: u16, len: u16) -> bool {
        let end = addr.saturating_add(len);
        self.covered.iter().any(|f| f.addr < end && f.end() > addr)
    }

    /// `[lo, hi)` around every covered field: the one span a host reads to
    /// compute over the live set.
    pub fn span(&self) -> (u16, u16) {
        let lo = self.covered.iter().map(|f| f.addr).min().unwrap_or(0);
        let hi = self.covered.iter().map(|f| f.end()).max().unwrap_or(0);
        (lo, hi)
    }

    /// `max(1, crc16_arc(tag ++ covered bytes in table order ++ lut_points
    /// LE))` over `bytes`, the table from address `base`. `None` points are
    /// the identity table (all zero).
    pub fn compute(
        &self,
        base: u16,
        bytes: &[u8],
        lut_points: Option<&[i16]>,
    ) -> Result<u16, StampError> {
        if let Some(k) = lut_points
            && k.len() != self.pos_lut_points
        {
            return Err(StampError::LutPoints {
                want: self.pos_lut_points,
                got: k.len(),
            });
        }
        let mut crc = osc_crc_continue(0, self.tag.as_bytes());
        for f in &self.covered {
            let lo = (f.addr as usize).checked_sub(base as usize);
            let b = lo
                .and_then(|lo| bytes.get(lo..lo + f.width as usize))
                .ok_or_else(|| StampError::Short(f.name.clone()))?;
            crc = osc_crc_continue(crc, b);
        }
        match lut_points {
            Some(k) => {
                for c in k {
                    crc = osc_crc_continue(crc, &c.to_le_bytes());
                }
            }
            None => {
                for _ in 0..self.pos_lut_points {
                    crc = osc_crc_continue(crc, &[0, 0]);
                }
            }
        }
        Ok(if crc == UNSTAMPED { 1 } else { crc })
    }
}

/// The stamp the servo holds beside the one its live set computes to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Verdict {
    pub stored: u16,
    pub computed: u16,
}

impl Verdict {
    pub fn matches(&self) -> bool {
        self.stored == self.computed
    }
}

/// The stamp of the set the servo holds now, over the points it applies.
pub async fn compute<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<u16, Error> {
    let stamp = d.stamp()?;
    let (lo, hi) = stamp.span();
    let bytes = c.read_span(id, lo, hi).await?;
    let lut_points = pos_lut::effective(c, id, d).await?;
    stamp
        .compute(lo, &bytes, lut_points.as_ref().map(|k| &k[..]))
        .map_err(|e| Error::Link(LinkError::Desync(e.to_string())))
}

pub async fn verdict<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<Verdict, Error> {
    let f = plant_stamp(d)?;
    let b = c.read(id, f.addr, f.width).await?;
    if b.len() < 2 {
        return Err(Error::Link(LinkError::Desync("short plant_stamp".into())));
    }
    Ok(Verdict {
        stored: u16::from_le_bytes([b[0], b[1]]),
        computed: compute(c, id, d).await?,
    })
}

/// Stamp the set the servo holds now. Verified by the firmware only with
/// torque off: under torque it lands unverified and `STAMP_MISMATCH` waits
/// for the next torque-off checkpoint. The checkpoint runs in the servo's
/// main loop after the reply, so this waits for the verdict: it returns
/// once `STAMP_MISMATCH` clears, or after [`VERDICT_WAIT`] with the flag
/// standing for the caller to read.
pub async fn restamp<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<u16, Error> {
    let stamp = compute(c, id, d).await?;
    c.write(id, plant_stamp(d)?.addr, &stamp.to_le_bytes())
        .await?;
    let flags = d
        .field("data_flags")
        .ok_or_else(|| Error::Descriptor(format!("no data_flags in {}", d.model)))?;
    for _ in 0..VERDICT_TRIES {
        let b = c.read(id, flags.addr, flags.width).await?;
        if b.first().is_some_and(|f| f & STAMP_MISMATCH == 0) {
            break;
        }
        c.pause(VERDICT_TICK).await;
    }
    Ok(stamp)
}

/// How long a host waits for a verdict the servo's main loop lands after
/// its reply (a stamp checkpoint, a LUT COMMIT): ~0.6 ms of CRC on the
/// servo, polled once per [`VERDICT_TICK`].
pub const VERDICT_WAIT: Duration = Duration::from_millis(20);
pub const VERDICT_TICK: Duration = Duration::from_millis(1);
pub(crate) const VERDICT_TRIES: u32 = (VERDICT_WAIT.as_millis() / VERDICT_TICK.as_millis()) as u32;

fn plant_stamp(d: &Descriptor) -> Result<&Field, Error> {
    d.field("plant_stamp")
        .ok_or_else(|| Error::Descriptor(format!("no plant_stamp in {}", d.model)))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPEC: &str = r#"{
        "format": 2, "model": "osc-servo", "class": "servo", "model_number": 257,
        "firmware_major": 0, "firmware_minor": 1, "table_size": 1024,
        "generator": "test",
        "stamp": {"tag": "osc-plant-1", "covered": ["a", "b", "c"], "pos_lut_points": 4},
        "fields": [
            {"name": "a", "addr": 32, "width": 4, "access": "rw", "kind": "int"},
            {"name": "x", "addr": 36, "width": 2, "access": "rw", "kind": "uint"},
            {"name": "b", "addr": 38, "width": 2, "access": "rw", "kind": "uint"},
            {"name": "c", "addr": 128, "width": 1, "access": "rw", "kind": "bool"},
            {"name": "plant_stamp", "addr": 130, "width": 2, "access": "rw", "kind": "uint"}
        ]
    }"#;

    fn spec() -> Descriptor {
        Descriptor::parse(SPEC).unwrap()
    }

    #[test]
    fn recipe_resolves_in_table_order_and_spans_the_set() {
        let d = spec();
        let s = d.stamp().unwrap();
        let names: Vec<&str> = s.covered().iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["a", "b", "c"]);
        assert_eq!(s.span(), (32, 129));
        assert!(s.covers(32, 1));
        assert!(s.covers(35, 1));
        assert!(!s.covers(36, 2), "x is not covered");
        assert!(s.covers(37, 2), "a write straddling into b");
        assert!(!s.covers(130, 2), "the stamp does not cover itself");
    }

    #[test]
    fn compute_hashes_tag_bytes_and_points_from_the_span_base() {
        let d = spec();
        let s = d.stamp().unwrap();
        let mut table = vec![0u8; 256];
        table[32..36].copy_from_slice(&(-7i32).to_le_bytes());
        table[38..40].copy_from_slice(&500u16.to_le_bytes());
        table[128] = 1;
        let want = {
            let mut crc = osc_crc_continue(0, b"osc-plant-1");
            crc = osc_crc_continue(crc, &table[32..36]);
            crc = osc_crc_continue(crc, &table[38..40]);
            crc = osc_crc_continue(crc, &table[128..129]);
            osc_crc_continue(crc, &[0u8; 8])
        };
        assert_eq!(s.compute(0, &table, None).unwrap(), want);
        assert_eq!(
            s.compute(32, &table[32..129], None).unwrap(),
            want,
            "the same set from the span base"
        );
        assert_eq!(s.compute(0, &table, Some(&[0; 4])).unwrap(), want);
        assert_ne!(s.compute(0, &table, Some(&[0, 1, 0, 0])).unwrap(), want);
        table[36] = 9;
        assert_eq!(
            s.compute(0, &table, None).unwrap(),
            want,
            "an uncovered byte is not hashed"
        );
        table[39] = 9;
        assert_ne!(s.compute(0, &table, None).unwrap(), want);
    }

    #[test]
    fn compute_refuses_short_input_and_wrong_point_counts() {
        let d = spec();
        let s = d.stamp().unwrap();
        let table = vec![0u8; 100];
        assert_eq!(
            s.compute(0, &table, None),
            Err(StampError::Short("c".into()))
        );
        assert_eq!(
            s.compute(0, &[0u8; 256], Some(&[0; 3])),
            Err(StampError::LutPoints { want: 4, got: 3 })
        );
        assert_eq!(
            s.compute(40, &[0u8; 256], None),
            Err(StampError::Short("a".into())),
            "a base past the field"
        );
    }

    #[test]
    fn recipe_errors_name_the_gap() {
        let json = SPEC.replacen("\"c\"]", "\"nope\"]", 1);
        let d = Descriptor::parse(&json).unwrap();
        assert!(matches!(d.stamp(), Err(Error::Descriptor(m)) if m.contains("nope")));
        let json = SPEC.replacen(
            "\"stamp\": {\"tag\": \"osc-plant-1\", \"covered\": [\"a\", \"b\", \"c\"], \"pos_lut_points\": 4},",
            "",
            1,
        );
        let d = Descriptor::parse(&json).unwrap();
        assert!(matches!(d.stamp(), Err(Error::Descriptor(_))));
    }
}

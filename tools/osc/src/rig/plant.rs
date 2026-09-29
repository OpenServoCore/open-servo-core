//! The plant data the servo holds beside its recordings: the position table as
//! the kernel applies it (read for the experiments as osc-ident's `Pot`
//! and for the records that say which counts a run was fitted in), the
//! plant stamp against the live set, and the data state. A capture is only
//! as interpretable as this record: the same raw pot streams as different
//! `pos_lin` under different tables.

use anyhow::Result;
use osc_client::blocking::Client;
use osc_client::data_state::DataState;
use osc_client::descriptor::Descriptor;
use osc_client::nusb::NusbPipe;
use osc_client::pos_lut::{INTERVALS, state};
use osc_client::stamp::{UNSTAMPED, Verdict};
use osc_client::{Error, Id};
use osc_ident::lut::{GRID, GridLut, POINTS};
use osc_ident::pot::Pot;
use osc_protocol::crc::osc_crc;
use serde_json::{Value, json};

use crate::descriptor;

/// The effective table: the array while LIVE, the identity otherwise.
pub(crate) struct Lut {
    pub(crate) state: u8,
    pub(crate) points: [i16; INTERVALS],
}

impl Lut {
    pub(crate) fn read(c: &mut Client<NusbPipe>, id: Id, d: &Descriptor) -> Result<Self> {
        let s = c.pos_lut_state(id, d)?;
        let points = if s == state::LIVE {
            c.pos_lut(id, d)?.points
        } else {
            [0; INTERVALS]
        };
        Ok(Self { state: s, points })
    }

    pub(crate) fn live(&self) -> bool {
        self.state == state::LIVE
    }

    pub(crate) fn state_name(&self) -> String {
        match state::name(self.state) {
            Some(n) => n.to_string(),
            None => format!("state {}", self.state),
        }
    }

    pub(crate) fn grid(&self) -> GridLut {
        let mut points = [0i16; POINTS];
        points[..INTERVALS].copy_from_slice(&self.points);
        GridLut { points }
    }

    /// What the experiments fit through.
    pub(crate) fn pot(&self) -> Pot {
        if self.live() {
            Pot::live(self.grid())
        } else {
            Pot::RAW
        }
    }

    /// CRC-16/ARC over the effective points LE, the bytes as the stamp
    /// hashes them; the identity has its own value.
    pub(crate) fn crc(&self) -> u16 {
        let bytes: Vec<u8> = self.points.iter().flat_map(|k| k.to_le_bytes()).collect();
        osc_crc(&bytes)
    }

    pub(crate) fn nonzero(&self) -> usize {
        self.points.iter().filter(|&&k| k != 0).count()
    }

    /// Raw counts of the first and last nonzero point.
    pub(crate) fn band(&self) -> Option<(u16, u16)> {
        let first = self.points.iter().position(|&k| k != 0)?;
        let last = self.points.iter().rposition(|&k| k != 0)?;
        Some((first as u16 * GRID, last as u16 * GRID))
    }

    pub(crate) fn describe(&self) -> String {
        if !self.live() {
            return format!("{} (the kernel applies the identity)", self.state_name());
        }
        match self.band() {
            Some((lo, hi)) => format!(
                "LIVE (crc {:#06x}, {} nonzero calibration points at raw {lo}..{hi})",
                self.crc(),
                self.nonzero()
            ),
            None => format!("LIVE (crc {:#06x}, every calibration point 0)", self.crc()),
        }
    }

    /// The compact record a recording's meta carries.
    pub(crate) fn json(&self) -> Value {
        json!({
            "lut_state": self.state_name(),
            "lut_crc": format!("{:#06x}", self.crc()),
            "lut_nonzero_points": self.nonzero(),
            "lut_band": self.band().map(|(lo, hi)| [lo, hi]),
        })
    }

    /// The whole table as the image `osc lut write` and `osc lut grade`
    /// take, tagged with the state and crc the recordings name it by.
    pub(crate) fn image(&self, stops: (u16, u16), dataset: &str, source: &str) -> Value {
        let covered = self.band().unwrap_or((0, 0));
        let img = self
            .grid()
            .image(stops.0, stops.1, dataset, 0, covered, source);
        let mut v = serde_json::to_value(img).expect("image serializes");
        v["lut_state"] = json!(self.state_name());
        v["lut_crc"] = json!(format!("{:#06x}", self.crc()));
        v
    }
}

/// The pot stops the table is validated against.
pub(crate) fn stops(c: &mut Client<NusbPipe>, id: Id, d: &Descriptor) -> Result<(u16, u16)> {
    let mut read = |name: &str| -> Result<u16> {
        let f = descriptor::field(d, name)?;
        let b = c.read(id, f.addr, f.width)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    };
    Ok((read("raw_min")?, read("raw_max")?))
}

/// The table, the stamp verdict and the data state as one reading.
pub(crate) struct Snapshot {
    pub(crate) lut: Lut,
    /// None when the descriptor carries no stamp recipe.
    pub(crate) stamp: Option<Verdict>,
    pub(crate) data: DataState,
}

impl Snapshot {
    pub(crate) fn read(c: &mut Client<NusbPipe>, id: Id, d: &Descriptor) -> Result<Self> {
        let lut = Lut::read(c, id, d)?;
        let stamp = match c.stamp_verdict(id, d) {
            Ok(v) => Some(v),
            Err(Error::Descriptor(_)) => None,
            Err(e) => return Err(e.into()),
        };
        let data = c.data_state(id, d)?;
        Ok(Self { lut, stamp, data })
    }

    pub(crate) fn stamp_verdict(&self) -> &'static str {
        match self.stamp {
            None => "none",
            Some(v) if v.stored == UNSTAMPED => "unstamped",
            Some(v) if v.matches() => "match",
            Some(_) => "mismatch",
        }
    }

    pub(crate) fn data_names(&self) -> Vec<&'static str> {
        self.data.reasons().iter().map(|r| r.name()).collect()
    }

    /// The `plant` block of a recording's meta.
    pub(crate) fn json(&self) -> Value {
        let mut v = self.lut.json();
        v["plant_stamp"] = json!(self.stamp.map(|s| format!("{:#06x}", s.stored)));
        v["stamp_computed"] = json!(self.stamp.map(|s| format!("{:#06x}", s.computed)));
        v["stamp_verdict"] = json!(self.stamp_verdict());
        v["data_flags"] = json!(self.data_names());
        v
    }

    /// One log line.
    pub(crate) fn line(&self) -> String {
        let data = match self.data.flags {
            0 => "clean".to_string(),
            _ => self.data_names().join(" | "),
        };
        format!(
            "lut {}, stamp {}, data {data}",
            self.lut.describe(),
            self.stamp_verdict()
        )
    }

    /// Whether a later reading describes the same plant: the same table,
    /// the same stamp verdict, the same data reasons.
    pub(crate) fn same(&self, later: &Self) -> bool {
        self.lut.state == later.lut.state
            && self.lut.points == later.lut.points
            && self.stamp_verdict() == later.stamp_verdict()
            && self.data.flags == later.data.flags
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_client::data_state::STAMP_MISMATCH;

    fn bent() -> Lut {
        let mut points = [0i16; INTERVALS];
        for (k, c) in points.iter_mut().enumerate().take(220).skip(35) {
            *c = (k as i16 - 35) % 7 + 1;
        }
        Lut {
            state: state::LIVE,
            points,
        }
    }

    #[test]
    fn identity_and_a_table_describe_themselves() {
        let id = Lut {
            state: state::IDENTITY,
            points: [0; INTERVALS],
        };
        assert_eq!(id.describe(), "IDENTITY (the kernel applies the identity)");
        assert_eq!(id.pot(), Pot::RAW);
        assert_eq!(id.band(), None);
        let rejected = Lut {
            state: state::REJECT_ENDS,
            ..bent()
        };
        assert_eq!(rejected.pot(), Pot::RAW);
        assert!(rejected.describe().starts_with("REJECT_ENDS"));

        let b = bent();
        assert_eq!(b.nonzero(), 185);
        assert_eq!(b.band(), Some((560, 3504)));
        assert!(b.pot().is_live());
        assert_eq!(b.pot().counts(560), 561.0);
        let crc = b.crc();
        assert_ne!(crc, id.crc());
        assert_eq!(
            b.describe(),
            format!("LIVE (crc {crc:#06x}, 185 nonzero calibration points at raw 560..3504)")
        );
        let zero = Lut {
            state: state::LIVE,
            points: [0; INTERVALS],
        };
        assert_eq!(
            zero.crc(),
            id.crc(),
            "an all-zero LIVE table hashes like the identity"
        );
        assert!(zero.pot().is_live());
    }

    #[test]
    fn snapshot_records_table_stamp_and_data_and_notices_a_change() {
        let a = Snapshot {
            lut: bent(),
            stamp: Some(Verdict {
                stored: 0x1234,
                computed: 0x1234,
            }),
            data: DataState {
                flags: 0,
                fault_code: 0,
            },
        };
        let j = a.json();
        assert_eq!(j["lut_state"], "LIVE");
        assert_eq!(j["lut_nonzero_points"], 185);
        assert_eq!(j["lut_band"], json!([560, 3504]));
        assert_eq!(j["lut_crc"], format!("{:#06x}", bent().crc()));
        assert_eq!(j["plant_stamp"], "0x1234");
        assert_eq!(j["stamp_verdict"], "match");
        assert_eq!(j["data_flags"], json!([]));
        assert!(
            a.line().ends_with("stamp match, data clean"),
            "{}",
            a.line()
        );

        let b = Snapshot {
            lut: bent(),
            stamp: Some(Verdict {
                stored: 0x1234,
                computed: 0x5678,
            }),
            data: DataState {
                flags: STAMP_MISMATCH,
                fault_code: 0,
            },
        };
        assert_eq!(b.json()["stamp_verdict"], "mismatch");
        assert_eq!(b.json()["data_flags"], json!(["STAMP_MISMATCH"]));
        assert!(b.line().ends_with("stamp mismatch, data STAMP_MISMATCH"));
        assert!(a.same(&a));
        assert!(!a.same(&b));

        let rebooted = Snapshot {
            lut: Lut {
                state: state::IDENTITY,
                points: [0; INTERVALS],
            },
            stamp: Some(Verdict {
                stored: 0x1234,
                computed: 0x1234,
            }),
            data: a.data,
        };
        assert!(!a.same(&rebooted), "a table lost to a reboot is a change");
        let j = rebooted.json();
        assert_eq!(j["lut_state"], "IDENTITY");
        assert_eq!(j["lut_band"], Value::Null);
        let unstamped = Snapshot {
            stamp: Some(Verdict {
                stored: UNSTAMPED,
                computed: 7,
            }),
            ..rebooted
        };
        assert_eq!(unstamped.stamp_verdict(), "unstamped");
        let no_recipe = Snapshot {
            stamp: None,
            ..unstamped
        };
        assert_eq!(no_recipe.json()["plant_stamp"], Value::Null);
        assert_eq!(no_recipe.stamp_verdict(), "none");
    }

    #[test]
    fn image_is_the_lut_write_format_tagged_with_state_and_crc() {
        let b = bent();
        let v = b.image((209, 3849), "mg90-a__2s", "session start");
        assert_eq!(v["raw_min"], 209);
        assert_eq!(v["raw_max"], 3849);
        assert_eq!(v["grid_shift"], 4);
        assert_eq!(v["points"].as_array().unwrap().len(), INTERVALS);
        assert_eq!(v["covered"], json!([560, 3504]));
        assert_eq!(v["dataset"], "mg90-a__2s");
        assert_eq!(v["lut_state"], "LIVE");
        assert_eq!(v["lut_crc"], format!("{:#06x}", b.crc()));
        let img: osc_ident::lut::Image = serde_json::from_value(v).unwrap();
        assert_eq!(img.lut(), Some(b.grid()));
    }
}

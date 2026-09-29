//! The `osc lut` verbs that touch a servo: `write`, `show`, `grade
//! --servo`, `clear`. Each runs the data-state warning first like every
//! other servo verb.

use std::path::PathBuf;

use anyhow::{Result, bail};
use osc_client::blocking::Client;
use osc_client::data_state::{CONFIG_CORRUPT, DataState, STAMP_MISMATCH};
use osc_client::descriptor::Descriptor;
use osc_client::nusb::NusbPipe;
use osc_client::pos_lut::{self, PosLut};
use osc_client::{Error, Id, ResultCode};
use osc_ident::lut::{GAIN_MAX, GRID, GridLut, POINTS, Reject};
use serde::Serialize;

use super::grade::Report;
use crate::descriptor;

#[derive(clap::Args, Debug)]
pub(crate) struct WriteArgs {
    /// An image JSON (from `osc lut build` or the notebook).
    image: PathBuf,
    /// Persist with MGMT SAVE after the table is LIVE. STAMP_MISMATCH
    /// stands until `osc ident`, so this saves a servo that refuses closed
    /// loop on purpose.
    #[arg(long)]
    save: bool,
}

#[derive(clap::Args, Debug)]
pub(crate) struct ShowArgs {
    /// The table and its report as JSON.
    #[arg(long)]
    json: bool,
}

struct Servo {
    c: Client<NusbPipe>,
    id: Id,
    d: Descriptor,
}

fn open(baud: String, id: u8) -> Result<Servo> {
    let mut c = crate::rig::connect(&baud)?;
    let id = Id::new(id);
    let d = crate::state::descriptor(&mut c, id)?;
    crate::state::warn(&mut c, id, &d)?;
    Ok(Servo { c, id, d })
}

impl Servo {
    fn stops(&mut self) -> Result<(u16, u16)> {
        let mut read = |name: &str| -> Result<u16> {
            let f = descriptor::field(&self.d, name)?;
            let b = self.c.read(self.id, f.addr, f.width)?;
            Ok(u16::from_le_bytes([b[0], b[1]]))
        };
        Ok((read("raw_min")?, read("raw_max")?))
    }

    fn lut(&mut self) -> Result<PosLut> {
        Ok(self.c.pos_lut(self.id, &self.d)?)
    }

    /// The state line after a write: what the data state says about the
    /// table the kernel now applies.
    fn after(&mut self) -> Result<DataState> {
        let s = self.c.data_state(self.id, &self.d)?;
        if s.flags & STAMP_MISMATCH != 0 {
            println!(
                "data state {}: closed loop refused until osc ident (the table re-defines the domain the identified set was fitted in; osc lut write never stamps)",
                s.names()
            );
        } else if let Some(msg) = s.message() {
            println!("data state {}: {msg}", s.names());
        } else {
            println!("data state clean: closed loop allowed");
        }
        Ok(s)
    }

    /// SAVE with the servo's own gate and the one osc adds: a CONFIG_CORRUPT
    /// servo runs board defaults, which only `osc recover` may bless.
    fn save(&mut self, s: &DataState) -> Result<()> {
        if s.flags & CONFIG_CORRUPT != 0 {
            bail!(
                "id {}: saved settings are unreadable and board defaults are running; run osc recover",
                self.id.as_byte()
            );
        }
        match self.c.save(self.id) {
            Ok(()) => println!("id {}: saved", self.id.as_byte()),
            Err(Error::Servo(ResultCode::Access)) => {
                bail!("SAVE needs torque disabled (protocol sec 9.4)")
            }
            Err(e) => return Err(e.into()),
        }
        Ok(())
    }
}

fn grid(lut: &PosLut) -> GridLut {
    let mut points = [0i16; POINTS];
    points[..pos_lut::INTERVALS].copy_from_slice(&lut.points);
    GridLut { points }
}

fn state_name(state: u8) -> String {
    match pos_lut::state::name(state) {
        Some(n) => n.to_string(),
        None => format!("state {state}"),
    }
}

/// Why the firmware would refuse `lut` against `stops`, in its own terms.
fn explain(lut: &GridLut, stops: (u16, u16)) -> Option<String> {
    let (raw_min, raw_max) = stops;
    match lut.validate(raw_min, raw_max) {
        Ok(()) => None,
        Err(Reject::Ends) => {
            let lo = (raw_min as usize + GRID as usize - 1) >> 4;
            let hi = raw_max as usize >> 4;
            let k = (0..POINTS)
                .find(|&k| (k <= lo || k >= hi) && lut.points[k] != 0)
                .unwrap_or(0);
            Some(format!(
                "REJECT_ENDS: knot {k} (raw {}) is {} inside the identity inset of stops {raw_min}..{raw_max} (knots <= {lo} and >= {hi} must be 0); rebuild against these stops (osc lut build --raw-min {raw_min} --raw-max {raw_max}) or re-run osc cal",
                k * GRID as usize,
                lut.points[k]
            ))
        }
        Err(Reject::Shape) => {
            let k = lut
                .points
                .windows(2)
                .position(|w| {
                    let d = GRID as i32 + w[1] as i32 - w[0] as i32;
                    !(1..GAIN_MAX * GRID as i32).contains(&d)
                })
                .unwrap_or(0);
            let d = GRID as i32 + lut.points[k + 1] as i32 - lut.points[k] as i32;
            Some(format!(
                "REJECT_SHAPE: interval at raw {}..{} has gain {:.2}x nominal (must be within 1/{GRID}..{GAIN_MAX}x); not a pot, rebuild from a fresh capture",
                k * GRID as usize,
                (k + 1) * GRID as usize,
                d as f64 / GRID as f64
            ))
        }
    }
}

pub(crate) fn write(a: &WriteArgs, baud: String, id: u8) -> Result<()> {
    let img = super::load(&a.image)?;
    let lut = super::on_grid(&img, &a.image)?;
    println!("image: {}", super::describe(&img, &a.image));
    let mut s = open(baud, id)?;
    let stops = s.stops()?;
    println!("servo stops {}..{}", stops.0, stops.1);
    if stops != (img.raw_min, img.raw_max) {
        println!(
            "warning: the image was built against stops {}..{}, the servo holds {}..{}; validating against the servo's",
            img.raw_min, img.raw_max, stops.0, stops.1
        );
    }
    if let Some(why) = explain(&lut, stops) {
        bail!("{why}");
    }
    println!("validate against servo stops: ok");
    let points: [i16; pos_lut::INTERVALS] = lut.points[..pos_lut::INTERVALS].try_into()?;
    match s.c.write_pos_lut(s.id, &s.d, &points) {
        Ok(()) => {}
        Err(Error::Lut(e)) => bail!("id {}: {e}", s.id.as_byte()),
        Err(e) => return Err(e.into()),
    }
    let r = Report::new(&lut, stops);
    println!(
        "id {}: lut LIVE, read back knot for knot\n{}",
        s.id.as_byte(),
        r.summary()
    );
    let after = s.after()?;
    if a.save {
        s.save(&after)?;
    } else {
        println!(
            "not saved: SAVE persists the table while LIVE (osc lut write --save, osc ident --save, osc save)"
        );
    }
    Ok(())
}

#[derive(Serialize)]
struct Shown {
    state: u8,
    state_name: Option<&'static str>,
    live: bool,
    #[serde(flatten)]
    report: Report,
    points: Vec<i16>,
}

pub(crate) fn show(a: &ShowArgs, baud: String, id: u8) -> Result<()> {
    let mut s = open(baud, id)?;
    let stops = s.stops()?;
    let lut = s.lut()?;
    let report = Report::new(&grid(&lut), stops);
    if a.json {
        let shown = Shown {
            state: lut.state,
            state_name: lut.state_name(),
            live: lut.live(),
            report,
            points: lut.points.to_vec(),
        };
        println!("{}", serde_json::to_string_pretty(&shown)?);
        return Ok(());
    }
    println!(
        "id {}: lut {}{}",
        s.id.as_byte(),
        state_name(lut.state),
        if lut.live() {
            ""
        } else {
            " (the kernel applies the identity)"
        }
    );
    print!("{}", report.summary());
    if let Some(why) = explain(&grid(&lut), stops) {
        println!("against the servo's stops: {why}");
    }
    Ok(())
}

pub(crate) fn grade(baud: String, id: u8, json: bool) -> Result<()> {
    let mut s = open(baud, id)?;
    let stops = s.stops()?;
    let lut = s.lut()?;
    let report = Report::new(&grid(&lut), stops);
    if json {
        println!("{}", serde_json::to_string_pretty(&report)?);
        return Ok(());
    }
    println!("id {}: lut {}", s.id.as_byte(), state_name(lut.state));
    print!("{}{}", report.chart(), report.summary());
    Ok(())
}

pub(crate) fn clear(baud: String, id: u8) -> Result<()> {
    let mut s = open(baud, id)?;
    match s.c.clear_pos_lut(s.id, &s.d) {
        Ok(()) => {}
        Err(Error::Lut(e)) => bail!("id {}: {e}", s.id.as_byte()),
        Err(e) => return Err(e.into()),
    }
    println!(
        "id {}: lut LIVE with every knot 0 (the identity; hashes like no table, so a stamped set stays stamped)",
        s.id.as_byte()
    );
    s.after()?;
    println!("not saved: SAVE persists it (osc save)");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn explain_names_the_offending_point_or_interval() {
        let mut lut = GridLut::IDENTITY;
        assert_eq!(explain(&lut, (209, 3849)), None);
        lut.points[14] = 1;
        let why = explain(&lut, (209, 3849)).unwrap();
        assert!(
            why.starts_with("REJECT_ENDS: knot 14 (raw 224) is 1 inside"),
            "{why}"
        );
        assert!(why.contains("knots <= 14 and >= 240"), "{why}");
        assert!(why.contains("--raw-min 209 --raw-max 3849"), "{why}");
        lut.points[14] = 0;
        lut.points[100] = 16;
        let why = explain(&lut, (209, 3849)).unwrap();
        assert!(
            why.starts_with("REJECT_SHAPE: interval at raw 1600..1616 has gain 0.00x"),
            "{why}"
        );
        lut.points[100] = 15;
        assert_eq!(explain(&lut, (209, 3849)), None);
    }
}

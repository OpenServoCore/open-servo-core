//! TEL rows: the four layouts identification streams, a row decoded off
//! its chunks (protocol sec 5.6), and the CSV a capture lands as, the
//! notebooks' own (osc-lab's `sweep.csv`), so `oscnb` reads it unchanged.

use std::fmt::{self, Write as _};

use osc_protocol::tel::{self, Chunk, Chunks};
use osc_servo_core::regions::telemetry::tick_flag;

/// A table field a row may carry: its address and width.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Reg {
    pub name: &'static str,
    pub addr: u16,
    pub width: u8,
}

const fn reg(name: &'static str, addr: u16, width: u8) -> Reg {
    Reg { name, addr, width }
}

/// The TELEMETRY fields a row decodes, mirrored from the servo's table;
/// `the_fields_are_the_descriptors` holds them to the exported layout.
pub mod telemetry {
    use super::{Reg, reg};

    pub const POS: Reg = reg("pos", 0x238, 2);
    /// The chip's 1.2 V reference against VDD.
    pub const VREF_RAW: Reg = reg("vref_raw", 0x23a, 2);
    pub const VBUS_RAW: Reg = reg("vbus_raw", 0x23c, 2);
    pub const NTC_RAW: Reg = reg("ntc_raw", 0x23e, 2);
    pub const CURRENT_RAW: Reg = reg("current_raw", 0x240, 2);
    pub const VMOTOR_A: Reg = reg("vmotor_a", 0x242, 2);
    pub const VMOTOR_B: Reg = reg("vmotor_b", 0x244, 2);
    /// Signed PWM compare counts; full scale is CALIB `pwm_period_ticks`.
    pub const DUTY_TICKS: Reg = reg("duty_ticks", 0x246, 2);
    pub const CURRENT_TROUGH: Reg = reg("current_trough", 0x248, 2);
    pub const I_TICK_COUNTS: Reg = reg("i_tick_counts", 0x24a, 2);
    pub const VDIFF_COUNTS: Reg = reg("vdiff_counts", 0x24c, 2);
    pub const TICK_FLAGS: Reg = reg("tick_flags", 0x252, 2);
    pub const THETA_HAT_Q16: Reg = reg("theta_hat_q16", 0x258, 4);
    pub const OMEGA_HAT_CPS: Reg = reg("omega_hat_cps", 0x25c, 4);
    /// Published at the medium step: a row carries it up to a tick old.
    pub const VBUS_COUNTS: Reg = reg("vbus_counts", 0x266, 2);

    pub const ALL: [Reg; 15] = [
        POS,
        VREF_RAW,
        VBUS_RAW,
        NTC_RAW,
        CURRENT_RAW,
        VMOTOR_A,
        VMOTOR_B,
        DUTY_TICKS,
        CURRENT_TROUGH,
        I_TICK_COUNTS,
        VDIFF_COUNTS,
        TICK_FLAGS,
        THETA_HAT_Q16,
        OMEGA_HAT_CPS,
        VBUS_COUNTS,
    ];
}

use telemetry as t;

/// The chunk covering `first..=last`, table order.
const fn span(first: Reg, last: Reg) -> Chunk {
    Chunk::new(first.addr, last.addr + last.width as u16 - first.addr)
}

/// The rows identification streams, 12 B each, two chunks at most.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Layout {
    /// pos, current_raw, vmotor_a, vmotor_b, duty_ticks, current_trough:
    /// the open-loop steps, the notebooks' row.
    Raw,
    /// pos, vref_raw, vbus_raw, ntc_raw, current_raw, vmotor_a: the rest.
    Rest,
    /// pos, duty_ticks, current_trough, i_tick_counts, vdiff_counts: the
    /// holds.
    Reading,
    /// theta_hat_q16, omega_hat_cps, i_tick_counts, vdiff_counts: the
    /// closed-loop steps.
    Loop,
}

impl Layout {
    pub const fn chunks(self) -> Chunks {
        match self {
            Layout::Raw => Chunks::two(
                span(t::POS, t::POS),
                span(t::CURRENT_RAW, t::CURRENT_TROUGH),
            ),
            Layout::Rest => Chunks::one(t::POS.addr, 12),
            Layout::Reading => {
                Chunks::two(span(t::POS, t::POS), span(t::DUTY_TICKS, t::VDIFF_COUNTS))
            }
            Layout::Loop => Chunks::two(
                span(t::THETA_HAT_Q16, t::OMEGA_HAT_CPS),
                span(t::I_TICK_COUNTS, t::VDIFF_COUNTS),
            ),
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            Layout::Raw => "raw",
            Layout::Rest => "rest",
            Layout::Reading => "reading",
            Layout::Loop => "loop",
        }
    }

    /// The fields a row carries past the CSV's fixed columns.
    const fn extra(self) -> &'static [Reg] {
        match self {
            Layout::Loop => &[t::THETA_HAT_Q16, t::OMEGA_HAT_CPS],
            _ => &[],
        }
    }
}

/// A row's chunks as `addr+len` joined by `,`: `0x238+2,0x246+8`.
pub fn chunks_text(chunks: &Chunks) -> String {
    let parts: Vec<String> = chunks
        .0
        .iter()
        .filter(|c| c.len != 0)
        .map(|c| format!("{:#x}+{}", c.addr, c.len))
        .collect();
    parts.join(",")
}

/// A row's chunks as a capture's meta records them, `[addr, len]` pairs.
pub fn chunks_pairs(chunks: &Chunks) -> Vec<[u16; 2]> {
    chunks
        .0
        .iter()
        .filter(|c| c.len != 0)
        .map(|c| [c.addr, c.len])
        .collect()
}

/// The shunt read the drive window `duty_ticks` drove: the firmware's own
/// rule against CALIB `i_window_min_ticks` (protocol sec 5.6).
pub fn window_valid(duty_ticks: i16, i_window_min_ticks: u16) -> bool {
    duty_ticks != 0 && duty_ticks.unsigned_abs() >= i_window_min_ticks
}

/// One row's bytes, at most a row's worth.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct RowBytes {
    len: u8,
    bytes: [u8; tel::ROW_BYTES_MAX],
}

impl RowBytes {
    pub fn new(row: &[u8]) -> Self {
        let mut r = Self::default();
        let n = row.len().min(tel::ROW_BYTES_MAX);
        r.bytes[..n].copy_from_slice(&row[..n]);
        r.len = n as u8;
        r
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }
}

/// One decoded row; a field the row does not carry is None. `tick` is the
/// fast-tick index within its burst, from the frames' stamps with the
/// first row at 0, so a lost frame or a dropped row shows as a jump.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelFrame {
    pub tick: u64,
    /// The row's `tick_flags` I_VALID, else [`window_valid`] on its duty;
    /// false with neither.
    pub window_valid: bool,
    pub pos: Option<u16>,
    /// `i_tick_counts`.
    pub current: Option<i16>,
    pub current_trough: Option<u16>,
    /// `duty_ticks`.
    pub duty_counts: Option<i16>,
    /// `vdiff_counts`.
    pub vdiff: Option<i16>,
    /// `vbus_counts`.
    pub vbus: Option<u16>,
    pub current_raw: Option<u16>,
    pub vmotor_a: Option<u16>,
    pub vmotor_b: Option<u16>,
    pub vbus_raw: Option<u16>,
    pub ntc_raw: Option<u16>,
    pub vref_raw: Option<u16>,
    pub theta_hat_q16: Option<i32>,
    pub omega_hat_cps: Option<i32>,
    pub row: RowBytes,
}

impl TelFrame {
    /// A channel by its table name, widened.
    pub fn channel(&self, name: &str) -> Option<f64> {
        let u = |v: Option<u16>| v.map(f64::from);
        let i = |v: Option<i16>| v.map(f64::from);
        match name {
            "pos" => u(self.pos),
            "vref_raw" => u(self.vref_raw),
            "vbus_raw" => u(self.vbus_raw),
            "ntc_raw" => u(self.ntc_raw),
            "current_raw" => u(self.current_raw),
            "vmotor_a" => u(self.vmotor_a),
            "vmotor_b" => u(self.vmotor_b),
            "duty_ticks" => i(self.duty_counts),
            "current_trough" => u(self.current_trough),
            "i_tick_counts" => i(self.current),
            "vdiff_counts" => i(self.vdiff),
            "vbus_counts" => u(self.vbus),
            _ => None,
        }
    }
}

/// The fields a row of `chunks` carries, table order.
pub fn carried(chunks: &Chunks) -> Vec<Reg> {
    t::ALL
        .into_iter()
        .filter(|r| chunks.offset(r.addr, r.width as u16).is_some())
        .collect()
}

/// One row of `chunks` at fast tick `tick`; `i_window_min_ticks` judges a
/// row without `tick_flags`. A row shorter than its chunks reads as fields
/// absent.
pub fn decode_row(chunks: &Chunks, i_window_min_ticks: u16, row: &[u8], tick: u64) -> TelFrame {
    let take = |r: Reg| {
        let at = chunks.offset(r.addr, r.width as u16)?;
        let b = row.get(at..at + 2)?;
        Some(u16::from_le_bytes([b[0], b[1]]))
    };
    let word = |r: Reg| {
        let at = chunks.offset(r.addr, r.width as u16)?;
        let b = row.get(at..at + 4)?;
        Some(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    };
    let duty_counts = take(t::DUTY_TICKS).map(|v| v as i16);
    let valid = match (take(t::TICK_FLAGS), duty_counts) {
        (Some(flags), _) => flags & tick_flag::I_VALID != 0,
        (None, Some(d)) => window_valid(d, i_window_min_ticks),
        (None, None) => false,
    };
    TelFrame {
        tick,
        window_valid: valid,
        pos: take(t::POS),
        current: take(t::I_TICK_COUNTS).map(|v| v as i16),
        current_trough: take(t::CURRENT_TROUGH),
        duty_counts,
        vdiff: take(t::VDIFF_COUNTS).map(|v| v as i16),
        vbus: take(t::VBUS_COUNTS),
        current_raw: take(t::CURRENT_RAW),
        vmotor_a: take(t::VMOTOR_A),
        vmotor_b: take(t::VMOTOR_B),
        vbus_raw: take(t::VBUS_RAW),
        ntc_raw: take(t::NTC_RAW),
        vref_raw: take(t::VREF_RAW),
        theta_hat_q16: word(t::THETA_HAT_Q16),
        omega_hat_cps: word(t::OMEGA_HAT_CPS),
        row: RowBytes::new(row),
    }
}

/// osc-lab's `sweep.csv` header: `pos_lin` stays an empty column, as every
/// recording keeps one header.
pub const CSV_HEADER: &str = "seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,duty_counts,vdiff,vbus,current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw,vref_raw,pos_lin";

/// The header of a capture of `layout`: [`CSV_HEADER`], then the fields it
/// carries past those columns.
pub fn csv_header(layout: Layout) -> String {
    let mut h = CSV_HEADER.to_string();
    for r in layout.extra() {
        h.push(',');
        h.push_str(r.name);
    }
    h
}

/// A row's fields as the CSV writes them, `tick` to `vref_raw` in
/// [`CSV_HEADER`]'s order; a field the row does not carry is empty.
pub struct RowFields<'a>(pub &'a TelFrame);

impl fmt::Display for RowFields<'_> {
    fn fmt(&self, w: &mut fmt::Formatter<'_>) -> fmt::Result {
        let f = self.0;
        write!(w, "{},{}", f.tick, f.window_valid as u8)?;
        let u = |v: Option<u16>| v.map(i32::from);
        let i = |v: Option<i16>| v.map(i32::from);
        for v in [
            u(f.pos),
            i(f.current),
            u(f.current_trough),
            i(f.duty_counts),
            i(f.vdiff),
            u(f.vbus),
            u(f.current_raw),
            u(f.vmotor_a),
            u(f.vmotor_b),
            u(f.vbus_raw),
            u(f.ntc_raw),
            u(f.vref_raw),
        ] {
            match v {
                Some(v) => write!(w, ",{v}")?,
                None => w.write_str(",")?,
            }
        }
        Ok(())
    }
}

/// One segment's rows as CSV lines, no header: `seg`, the commanded duty
/// and its direction ahead of every row, as osc-lab writes them.
pub fn csv_rows(out: &mut String, layout: Layout, seg: u32, cmd_duty_q15: i16, rows: &[TelFrame]) {
    let dir = cmd_duty_q15.signum();
    for f in rows {
        let _ = write!(out, "{seg},{cmd_duty_q15},{dir},{},", RowFields(f));
        for r in layout.extra() {
            out.push(',');
            let v = match r.addr {
                a if a == t::THETA_HAT_Q16.addr => f.theta_hat_q16,
                _ => f.omega_hat_cps,
            };
            if let Some(v) = v {
                let _ = write!(out, "{v}");
            }
        }
        out.push('\n');
    }
}

/// A whole capture of one segment, header first.
pub fn csv(layout: Layout, seg: u32, cmd_duty_q15: i16, rows: &[TelFrame]) -> String {
    let mut s = csv_header(layout);
    s.push('\n');
    csv_rows(&mut s, layout, seg, cmd_duty_q15, rows);
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descriptor() -> serde_json::Value {
        let p = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../descriptors/osc-servo/0.1.json"
        );
        serde_json::from_str(&std::fs::read_to_string(p).expect("descriptor")).expect("json")
    }

    #[test]
    fn the_fields_are_the_descriptors() {
        let d = descriptor();
        for r in t::ALL {
            let f = d["fields"]
                .as_array()
                .and_then(|a| a.iter().find(|f| f["name"] == r.name))
                .unwrap_or_else(|| panic!("{} missing", r.name));
            assert_eq!(
                (f["addr"].as_u64(), f["width"].as_u64()),
                (Some(r.addr as u64), Some(r.width as u64)),
                "{}",
                r.name
            );
        }
    }

    /// The four rows of identification, each one the servo arms.
    #[test]
    fn the_four_rows_are_the_plans() {
        let text: Vec<String> = [Layout::Raw, Layout::Rest, Layout::Reading, Layout::Loop]
            .iter()
            .map(|l| {
                assert!(l.chunks().valid(), "{l:?}");
                let want = if *l == Layout::Reading { 10 } else { 12 };
                assert_eq!(l.chunks().row_bytes(), want, "{l:?}");
                chunks_text(&l.chunks())
            })
            .collect();
        assert_eq!(
            text,
            [
                "0x238+2,0x240+10",
                "0x238+12",
                "0x238+2,0x246+8",
                "0x258+8,0x24a+4"
            ]
        );
        let names = |l: Layout| {
            carried(&l.chunks())
                .iter()
                .map(|r| r.name)
                .collect::<Vec<_>>()
        };
        assert_eq!(
            names(Layout::Rest),
            [
                "pos",
                "vref_raw",
                "vbus_raw",
                "ntc_raw",
                "current_raw",
                "vmotor_a"
            ]
        );
        assert_eq!(
            names(Layout::Raw),
            [
                "pos",
                "current_raw",
                "vmotor_a",
                "vmotor_b",
                "duty_ticks",
                "current_trough"
            ]
        );
        assert_eq!(
            names(Layout::Reading),
            [
                "pos",
                "duty_ticks",
                "current_trough",
                "i_tick_counts",
                "vdiff_counts"
            ]
        );
        assert_eq!(
            names(Layout::Loop),
            [
                "i_tick_counts",
                "vdiff_counts",
                "theta_hat_q16",
                "omega_hat_cps"
            ]
        );
    }

    #[test]
    fn window_valid_is_the_drive_against_the_floor() {
        assert!(window_valid(160, 160));
        assert!(window_valid(-160, 160));
        assert!(!window_valid(159, 160));
        assert!(!window_valid(0, 0));
        assert!(window_valid(1, 0));
    }

    /// A loop row writes the observer's two words after the fixed columns.
    #[test]
    fn a_loop_row_carries_its_estimates_past_the_fixed_columns() {
        let chunks = Layout::Loop.chunks();
        let mut row = Vec::new();
        row.extend((-(5i32 << 16)).to_le_bytes());
        row.extend(742i32.to_le_bytes());
        row.extend((-3i16).to_le_bytes());
        row.extend(40i16.to_le_bytes());
        let f = decode_row(&chunks, 64, &row, 9);
        assert_eq!(
            (f.theta_hat_q16, f.omega_hat_cps, f.current, f.vdiff),
            (Some(-(5 << 16)), Some(742), Some(-3), Some(40))
        );
        assert_eq!(
            csv(Layout::Loop, 2, 0, &[f]),
            format!(
                "{CSV_HEADER},theta_hat_q16,omega_hat_cps\n2,0,0,9,0,,-3,,,40,,,,,,,,,-327680,742\n"
            )
        );
    }
}

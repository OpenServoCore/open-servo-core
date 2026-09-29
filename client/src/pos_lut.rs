//! Position table, the host half of the firmware's `pos_lut` module: the
//! 256-point table behind the CONTROL window (`pos_lut_page`, `pos_lut_cmd`,
//! `pos_lut_points`, `pos_lut_state`), STOREd a page at a time, COMMITted as
//! one, FETCHed back. The window fields come from the descriptor; the grid,
//! the page size and the state and command values mirror the servo core
//! (pinned against it by the fake-adapter test).

use std::fmt;

use osc_protocol::wire::Id;

use crate::client::Client;
use crate::descriptor::{Descriptor, Field};
use crate::error::{Error, LinkError};
use crate::pipe::Pipe;
use crate::stamp::{VERDICT_TICK, VERDICT_TRIES};

/// Host-written points; the fixed last point has no page.
pub const INTERVALS: usize = 256;
/// Raw counts per interval.
pub const GRID: u16 = 16;
pub const PAGE_POINTS: usize = 32;
pub const PAGES: usize = INTERVALS / PAGE_POINTS;

/// `pos_lut_state` values.
pub mod state {
    pub const IDENTITY: u8 = 0;
    /// Pages landed since the last COMMIT; the kernel applies the identity.
    pub const LOADING: u8 = 1;
    pub const LIVE: u8 = 2;
    pub const REJECT_TORQUE: u8 = 3;
    pub const REJECT_ENDS: u8 = 4;
    pub const REJECT_SHAPE: u8 = 5;

    pub fn name(state: u8) -> Option<&'static str> {
        Some(match state {
            IDENTITY => "IDENTITY",
            LOADING => "LOADING",
            LIVE => "LIVE",
            REJECT_TORQUE => "REJECT_TORQUE",
            REJECT_ENDS => "REJECT_ENDS",
            REJECT_SHAPE => "REJECT_SHAPE",
            _ => return None,
        })
    }
}

/// `pos_lut_cmd` values; a committed write carrying one runs it.
pub mod cmd {
    pub const NONE: u8 = 0;
    pub const STORE: u8 = 1;
    pub const FETCH: u8 = 2;
    pub const COMMIT: u8 = 3;
}

/// Why a table did not go live.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LutError {
    /// `torque_enable` is on: the servo refuses STORE and COMMIT, so the
    /// host never sends them.
    TorqueOn,
    /// The servo refused STORE or COMMIT under torque (it came on mid-write).
    RejectTorque,
    /// A nonzero point at or beyond a stop: the stops must map to themselves.
    RejectEnds,
    /// An interval with local gain outside `[1/16, 16)` of nominal.
    RejectShape,
    /// Still LOADING after [`crate::stamp::VERDICT_WAIT`]: the servo's
    /// main loop has not judged the COMMIT.
    Pending,
    /// `pos_lut_state` after COMMIT is none of the above.
    State(u8),
    /// The FETCHed page differs from what was STOREd.
    Readback { page: u8 },
}

impl fmt::Display for LutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LutError::TorqueOn => write!(f, "torque is on; the lut is written with torque off"),
            LutError::RejectTorque => write!(f, "servo refused the lut: torque came on"),
            LutError::RejectEnds => write!(
                f,
                "servo refused the lut: a nonzero point at or beyond a stop (REJECT_ENDS)"
            ),
            LutError::RejectShape => write!(
                f,
                "servo refused the lut: an interval is not monotone or gains over 16x (REJECT_SHAPE)"
            ),
            LutError::Pending => write!(f, "servo has not judged the lut (LOADING after COMMIT)"),
            LutError::State(s) => write!(f, "pos_lut_state {s} after COMMIT"),
            LutError::Readback { page } => write!(f, "lut page {page} read back differently"),
        }
    }
}

impl std::error::Error for LutError {}

/// The table as the servo holds it and what the kernel does with it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PosLut {
    pub state: u8,
    pub points: [i16; INTERVALS],
}

impl PosLut {
    pub fn live(&self) -> bool {
        self.state == state::LIVE
    }

    pub fn state_name(&self) -> Option<&'static str> {
        state::name(self.state)
    }

    /// The points the kernel applies: the array when LIVE, else identity.
    pub fn effective(&self) -> Option<&[i16; INTERVALS]> {
        self.live().then_some(&self.points)
    }
}

/// The window's fields, contiguous so one WRITE carries page, command and
/// points and one READ returns points and state.
struct Window<'a> {
    page: &'a Field,
    points: &'a Field,
    state: &'a Field,
}

impl<'a> Window<'a> {
    fn resolve(d: &'a Descriptor) -> Result<Self, Error> {
        let field = |name: &str| {
            d.field(name)
                .ok_or_else(|| Error::Descriptor(format!("no {name} in {}", d.model)))
        };
        let w = Window {
            page: field("pos_lut_page")?,
            points: field("pos_lut_points")?,
            state: field("pos_lut_state")?,
        };
        let cmd = field("pos_lut_cmd")?;
        if cmd.addr != w.page.end()
            || w.points.addr != cmd.end()
            || w.state.addr != w.points.end()
            || w.points.width as usize != 2 * PAGE_POINTS
        {
            return Err(Error::Descriptor(format!(
                "{}: lut window is not page, cmd, {PAGE_POINTS} points, state in a row",
                d.model
            )));
        }
        Ok(w)
    }
}

/// One committed window write: page and command, plus the page's points
/// for STORE.
async fn command<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    w: &Window<'_>,
    page: u8,
    cmd: u8,
    points: Option<&[i16]>,
) -> Result<(), Error> {
    let mut b = vec![page, cmd];
    if let Some(k) = points {
        for c in k {
            b.extend_from_slice(&c.to_le_bytes());
        }
    }
    c.write(id, w.page.addr, &b).await
}

/// FETCH one page and read it back with the state behind it.
async fn fetch<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    w: &Window<'_>,
    page: u8,
) -> Result<([i16; PAGE_POINTS], u8), Error> {
    command(c, id, w, page, cmd::FETCH, None).await?;
    let b = c
        .read(id, w.points.addr, w.points.width + w.state.width)
        .await?;
    if b.len() < 2 * PAGE_POINTS + 1 {
        return Err(Error::Link(LinkError::Desync("short lut page".into())));
    }
    let mut points = [0i16; PAGE_POINTS];
    for (k, pair) in points.iter_mut().zip(b.as_chunks::<2>().0) {
        *k = i16::from_le_bytes(*pair);
    }
    Ok((points, b[2 * PAGE_POINTS]))
}

pub async fn state<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<u8, Error> {
    let w = Window::resolve(d)?;
    read_state(c, id, &w).await
}

async fn read_state<P: Pipe>(c: &mut Client<P>, id: Id, w: &Window<'_>) -> Result<u8, Error> {
    let b = c.read(id, w.state.addr, w.state.width).await?;
    b.first()
        .copied()
        .ok_or_else(|| Error::Link(LinkError::Desync("short pos_lut_state".into())))
}

/// COMMIT, then the verdict: the servo judges the array in its main loop
/// after the reply, so `pos_lut_state` reads LOADING until it lands. Returns
/// the first state past LOADING, or LOADING itself after the wait.
async fn commit<P: Pipe>(c: &mut Client<P>, id: Id, w: &Window<'_>) -> Result<u8, Error> {
    command(c, id, w, 0, cmd::COMMIT, None).await?;
    let mut s = state::LOADING;
    for _ in 0..VERDICT_TRIES {
        s = read_state(c, id, w).await?;
        if s != state::LOADING {
            break;
        }
        c.pause(VERDICT_TICK).await;
    }
    Ok(s)
}

/// FETCH every page: the array as the servo holds it, whatever its state.
pub async fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<PosLut, Error> {
    let w = Window::resolve(d)?;
    let mut lut = PosLut {
        state: state::IDENTITY,
        points: [0; INTERVALS],
    };
    for page in 0..PAGES {
        let (points, state) = fetch(c, id, &w, page as u8).await?;
        lut.points[page * PAGE_POINTS..][..PAGE_POINTS].copy_from_slice(&points);
        lut.state = state;
    }
    Ok(lut)
}

/// The points the kernel applies now: `Some` only while LIVE, read with one
/// state read first so a servo at identity costs no page fetches.
pub async fn effective<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
) -> Result<Option<[i16; INTERVALS]>, Error> {
    if state(c, id, d).await? != state::LIVE {
        return Ok(None);
    }
    let lut = read(c, id, d).await?;
    Ok(lut.effective().copied())
}

/// STORE and COMMIT are refused under torque, and from LIVE the refusal
/// leaves the state standing, so a readback would say nothing: check first.
async fn torque_off<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<(), Error> {
    let torque = d
        .field("torque_enable")
        .ok_or_else(|| Error::Descriptor(format!("no torque_enable in {}", d.model)))?;
    let b = c.read(id, torque.addr, torque.width).await?;
    if b.first().is_some_and(|&on| on != 0) {
        return Err(Error::Lut(LutError::TorqueOn));
    }
    Ok(())
}

/// STORE every page, COMMIT, then FETCH every page back: the table is LIVE
/// and point for point what was sent, or the error says why not. Torque must
/// be off, checked here before anything is sent. Never stamps: a new
/// table re-defines the domain the identified set was fitted in, so the
/// COMMIT checkpoint leaves STAMP_MISMATCH for `osc ident` to clear.
pub async fn write<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    points: &[i16; INTERVALS],
) -> Result<(), Error> {
    let w = Window::resolve(d)?;
    torque_off(c, id, d).await?;
    for page in 0..PAGES {
        let k = &points[page * PAGE_POINTS..][..PAGE_POINTS];
        command(c, id, &w, page as u8, cmd::STORE, Some(k)).await?;
    }
    match commit(c, id, &w).await? {
        state::LIVE => {}
        state::LOADING => return Err(Error::Lut(LutError::Pending)),
        state::REJECT_TORQUE => return Err(Error::Lut(LutError::RejectTorque)),
        state::REJECT_ENDS => return Err(Error::Lut(LutError::RejectEnds)),
        state::REJECT_SHAPE => return Err(Error::Lut(LutError::RejectShape)),
        other => return Err(Error::Lut(LutError::State(other))),
    }
    for page in 0..PAGES {
        let (back, _) = fetch(c, id, &w, page as u8).await?;
        if back != points[page * PAGE_POINTS..][..PAGE_POINTS] {
            return Err(Error::Lut(LutError::Readback { page: page as u8 }));
        }
    }
    Ok(())
}

/// The identity as a LIVE table: all-zero points hash like no table, so a
/// stamped servo stays stamped.
pub async fn clear<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<(), Error> {
    write(c, id, d, &[0; INTERVALS]).await
}

/// COMMIT the array the servo already holds and return `pos_lut_state` after
/// it: LIVE, or the REJECT that names why the kernel now runs the
/// identity. The servo validates a table only at COMMIT and at boot, so
/// after the stops move under a LIVE table (cal) this is what judges it
/// against the new stops now instead of the next boot. Torque must be off.
pub async fn recommit<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<u8, Error> {
    let w = Window::resolve(d)?;
    torque_off(c, id, d).await?;
    commit(c, id, &w).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn states_name_themselves() {
        for s in 0..6 {
            assert!(state::name(s).is_some(), "{s}");
        }
        assert_eq!(state::name(state::LIVE), Some("LIVE"));
        assert_eq!(state::name(6), None);
        let lut = PosLut {
            state: state::LOADING,
            points: [1; INTERVALS],
        };
        assert!(!lut.live());
        assert_eq!(lut.effective(), None);
        let lut = PosLut {
            state: state::LIVE,
            ..lut
        };
        assert_eq!(lut.effective(), Some(&[1; INTERVALS]));
    }

    #[test]
    fn window_must_be_contiguous() {
        const SPEC: &str = r#"{
            "format": 2, "model": "osc-servo", "class": "servo", "model_number": 257,
            "firmware_major": 0, "firmware_minor": 1, "table_size": 1024,
            "generator": "test",
            "fields": [
                {"name": "pos_lut_page", "addr": 412, "width": 1, "access": "rw", "kind": "uint"},
                {"name": "pos_lut_cmd", "addr": 413, "width": 1, "access": "rw", "kind": "uint"},
                {"name": "pos_lut_points", "addr": 414, "width": 64, "access": "rw", "kind": "bytes"},
                {"name": "pos_lut_state", "addr": 478, "width": 1, "access": "ro", "kind": "uint"}
            ]
        }"#;
        let d = Descriptor::parse(SPEC).unwrap();
        assert!(Window::resolve(&d).is_ok());
        let d = Descriptor::parse(&SPEC.replacen("\"addr\": 478", "\"addr\": 479", 1)).unwrap();
        assert!(matches!(Window::resolve(&d), Err(Error::Descriptor(_))));
        let d = Descriptor::parse(&SPEC.replacen("\"width\": 64", "\"width\": 32", 1)).unwrap();
        assert!(matches!(Window::resolve(&d), Err(Error::Descriptor(_))));
        let d = Descriptor::parse(&SPEC.replacen("pos_lut_state", "pos_lut_stat", 1)).unwrap();
        assert!(
            matches!(Window::resolve(&d), Err(Error::Descriptor(m)) if m.contains("pos_lut_state"))
        );
    }

    /// The grid, the window and the state and command values are the servo
    /// core's, not a second opinion.
    #[cfg(feature = "fake-adapter")]
    #[test]
    fn grid_window_and_values_are_the_servo_cores() {
        use osc_servo_core::pos_lut as fw;
        assert_eq!(INTERVALS, fw::INTERVALS);
        assert_eq!(GRID as usize, fw::GRID);
        assert_eq!(PAGE_POINTS, fw::PAGE_POINTS);
        assert_eq!(PAGES, fw::PAGES);
        assert_eq!(state::IDENTITY, fw::state::IDENTITY);
        assert_eq!(state::LOADING, fw::state::LOADING);
        assert_eq!(state::LIVE, fw::state::LIVE);
        assert_eq!(state::REJECT_TORQUE, fw::state::REJECT_TORQUE);
        assert_eq!(state::REJECT_ENDS, fw::state::REJECT_ENDS);
        assert_eq!(state::REJECT_SHAPE, fw::state::REJECT_SHAPE);
        assert_eq!(cmd::NONE, fw::cmd::NONE);
        assert_eq!(cmd::STORE, fw::cmd::STORE);
        assert_eq!(cmd::FETCH, fw::cmd::FETCH);
        assert_eq!(cmd::COMMIT, fw::cmd::COMMIT);
    }
}

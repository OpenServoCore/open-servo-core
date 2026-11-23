//! The one way out. Every exit of a step, done, refused, a bus error or
//! the interrupt flag, runs [`exit`] once anything was written: zero duty,
//! torque off and read back, permit off, dead-man 0, telemetry count 0,
//! then the volatile fields the run changed put back, or the servo
//! rebooted onto its saved state when one does not go back. If the bus is
//! gone the dead-man has already dropped torque, so it is left armed
//! until torque reads off.

use std::fmt;

use osc_client::descriptor::Descriptor;
use osc_client::{Client, Error, Id, pipe::Pipe};

use crate::install;
use crate::state::field;

/// How a step ended.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Way {
    Done,
    Refused,
    Error,
    Interrupted,
}

impl Way {
    pub fn name(self) -> &'static str {
        match self {
            Way::Done => "accepted",
            Way::Refused => "refused",
            Way::Error => "aborted",
            Way::Interrupted => "interrupted",
        }
    }
}

/// What a run changed that the way out undoes: whether anything went on
/// the bus, and each volatile field it wrote with the bytes it held.
#[derive(Clone, Debug, Default)]
pub struct Ledger {
    pub touched: bool,
    pub fields: Vec<(String, u16, Vec<u8>)>,
}

impl Ledger {
    /// Read `name` and keep what it holds, ahead of writing it; a field
    /// kept once keeps its first bytes.
    pub async fn keep<P: Pipe>(
        &mut self,
        c: &mut Client<P>,
        id: Id,
        d: &Descriptor,
        name: &str,
    ) -> Result<(), Error> {
        if self.fields.iter().any(|(n, _, _)| n == name) {
            return Ok(());
        }
        let f = field(d, name)?;
        let was = c.read(id, f.addr, f.width).await?;
        self.fields.push((name.into(), f.addr, was));
        Ok(())
    }
}

/// What the way out did, a line a move, and whether torque read off.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Exit {
    pub lines: Vec<String>,
    pub torque_off: bool,
    pub rebooted: bool,
}

impl fmt::Display for Exit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.lines.join("\n"))
    }
}

async fn put<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    name: &str,
    bytes: &[u8],
) -> Result<(), Error> {
    let f = field(d, name)?;
    c.write(id, f.addr, bytes).await
}

fn said(lines: &mut Vec<String>, what: &str, r: Result<(), Error>) {
    lines.push(match r {
        Ok(()) => what.to_string(),
        Err(e) => format!("{what}: not confirmed ({e})"),
    });
}

/// The way out. Never fails: each move is tried whatever the one before
/// did, and the lines say what held.
pub async fn exit<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor, ledger: &Ledger) -> Exit {
    if !ledger.touched {
        let off = match field(d, "torque_enable") {
            Ok(f) => c
                .read(id, f.addr, 1)
                .await
                .is_ok_and(|b| b.first() == Some(&0)),
            Err(_) => false,
        };
        return Exit {
            lines: vec!["nothing was written".into()],
            torque_off: off,
            rebooted: false,
        };
    }
    let mut lines = Vec::new();
    said(
        &mut lines,
        "goal_duty 0",
        put(c, id, d, "goal_duty", &[0, 0]).await,
    );
    let mut torque_off = false;
    for _ in 0..2 {
        if put(c, id, d, "torque_enable", &[0]).await.is_ok() {
            let t = field(d, "torque_enable");
            if let Ok(f) = t
                && let Ok(b) = c.read(id, f.addr, 1).await
            {
                torque_off = b.first() == Some(&0);
            }
        }
        if torque_off {
            break;
        }
    }
    lines.push(if torque_off {
        "torque_enable 0, read back 0".into()
    } else {
        "torque_enable 0: NOT read back off; the bus dead-man is left armed to drop torque".into()
    });
    said(
        &mut lines,
        "stall_permit 0",
        put(c, id, d, "stall_permit", &[0]).await,
    );
    if torque_off {
        said(
            &mut lines,
            "bus_deadman_ticks 0",
            put(c, id, d, "bus_deadman_ticks", &[0, 0]).await,
        );
    }
    said(
        &mut lines,
        "tel_count 0",
        put(c, id, d, "tel_count", &[0, 0]).await,
    );
    let mut back = true;
    for (name, addr, was) in &ledger.fields {
        let ok = match c.write(id, *addr, was).await {
            Ok(()) => c
                .read(id, *addr, was.len() as u16)
                .await
                .is_ok_and(|b| &b == was),
            Err(_) => false,
        };
        lines.push(if ok {
            format!("{name} put back, read back")
        } else {
            format!("{name} did not go back")
        });
        back &= ok;
    }
    let mut rebooted = false;
    if !back || !torque_off {
        let mut steps = Vec::new();
        match install::reboot(c, id, &mut steps).await {
            Ok(()) => {
                rebooted = true;
                lines.push("rebooted: the servo runs its saved state".into());
                if let Ok(f) = field(d, "torque_enable")
                    && let Ok(b) = c.read(id, f.addr, 1).await
                {
                    torque_off = b.first() == Some(&0);
                }
            }
            Err(e) => lines.push(format!(
                "the reboot did not answer ({e}): power the servo off and on"
            )),
        }
    }
    Exit {
        lines,
        torque_off,
        rebooted,
    }
}

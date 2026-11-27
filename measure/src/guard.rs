//! The one way out. Every exit of a step, done, refused, a bus error or
//! the interrupt flag, runs [`exit`] once anything was written: zero duty,
//! torque off and read back, permit off, dead-man 0, telemetry count 0,
//! then the volatile fields the run changed put back, or the servo
//! rebooted onto its saved state when one does not go back. If the bus is
//! gone the dead-man has already dropped torque, so it is left armed
//! until torque reads off. Its last line counts the instructions the bus
//! sent once more after no reply, when there were any.

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

/// The kept fields as the writes that put them back: each run of fields
/// kept one after the other at adjacent addresses goes back in one write,
/// so a pair the servo checks side against side, the soft limits, goes
/// back whole (protocol sec 5.8: 0 / 0 is reached in one write only).
fn runs(fields: &[(String, u16, Vec<u8>)]) -> Vec<&[(String, u16, Vec<u8>)]> {
    let mut out = Vec::new();
    let mut start = 0;
    for i in 1..=fields.len() {
        let joins = fields.get(i).is_some_and(|(_, addr, _)| {
            let (_, a, w) = &fields[i - 1];
            *a as usize + w.len() == *addr as usize
        });
        if !joins {
            out.push(&fields[start..i]);
            start = i;
        }
    }
    out
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
        let mut lines = vec!["nothing was written".into()];
        resent(c, &mut lines);
        return Exit {
            lines,
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
    for run in runs(&ledger.fields) {
        let addr = run[0].1;
        let was: Vec<u8> = run.iter().flat_map(|f| f.2.iter().copied()).collect();
        let ok = match c.write(id, addr, &was).await {
            Ok(()) => c
                .read(id, addr, was.len() as u16)
                .await
                .is_ok_and(|b| b == was),
            Err(_) => false,
        };
        for (name, _, _) in run {
            lines.push(if ok {
                format!("{name} put back, read back")
            } else {
                format!("{name} did not go back")
            });
        }
        back &= ok;
    }
    let mut rebooted = false;
    if !back || !torque_off {
        let mut steps = Vec::new();
        match install::reboot(c, id, &mut steps).await {
            Ok(()) => {
                rebooted = true;
                lines.extend(steps.iter().map(ToString::to_string));
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
    resent(c, &mut lines);
    Exit {
        lines,
        torque_off,
        rebooted,
    }
}

fn resent<P: Pipe>(c: &mut Client<P>, lines: &mut Vec<String>) {
    match c.take_resent() {
        0 => {}
        1 => lines.push("1 instruction drew no reply and was sent once more".into()),
        n => lines.push(format!(
            "{n} instructions drew no reply and were sent once more"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::runs;

    #[test]
    fn adjacent_kept_fields_go_back_in_one_write() {
        let f = |name: &str, addr: u16, width: usize| (name.to_string(), addr, vec![0; width]);
        let kept = [
            f("mode", 0x1a0, 1),
            f("current_limit_counts", 0x064, 2),
            f("pos_min_soft_counts", 0x028, 4),
            f("pos_max_soft_counts", 0x02c, 4),
            f("goal_duty", 0x186, 2),
        ];
        let names: Vec<Vec<&str>> = runs(&kept)
            .iter()
            .map(|r| r.iter().map(|(n, _, _)| n.as_str()).collect())
            .collect();
        assert_eq!(
            names,
            [
                vec!["mode"],
                vec!["current_limit_counts"],
                vec!["pos_min_soft_counts", "pos_max_soft_counts"],
                vec!["goal_duty"],
            ]
        );
        assert!(runs(&[]).is_empty());
    }
}

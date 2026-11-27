//! Data state on every servo verb: the one-line warning a command prints
//! when `data_flags` names a reason (information: the command still runs,
//! and the servo refuses a mode only on a constant it cannot run on), the
//! `osc status` report, and `osc recover`.

use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use dialoguer::Confirm;
use osc_client::blocking::Client;
use osc_client::data_state::{DataState, fault};
use osc_client::descriptor::{Access, Descriptor, Field};
use osc_client::nusb::NusbPipe;
use osc_client::pipe::Pipe;
use osc_client::{Error, Id, ResultCode};
use osc_protocol::table::CONFIG_COMMON_END;
use osc_servo_core::regions::CONTROL_BASE_ADDR;

use crate::bus::Bus;
use crate::descriptor;

/// The servo's descriptor (built-ins plus the operator's overrides, picked
/// by one identity read), owned so callers keep no registry around.
pub(crate) fn descriptor<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Descriptor> {
    let reg = descriptor::load()?;
    Ok(crate::select_descriptor(c, id, &reg)?.clone())
}

/// Read the data state; a set reason prints the one-line warning.
pub(crate) fn warn<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<DataState> {
    let s = c.data_state(id, d)?;
    if let Some(msg) = s.message() {
        eprintln!("warning: id {}: {} - {msg}", id.as_byte(), s.names());
    }
    Ok(s)
}

/// [`warn`] for the verbs that resolve no descriptor of their own.
pub(crate) fn check<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<DataState> {
    let d = descriptor(c, id)?;
    warn(c, id, &d)
}

pub(crate) fn check_each(c: &mut Client<NusbPipe>, ids: &[Id]) -> Result<()> {
    for &id in ids {
        check(c, id)?;
    }
    Ok(())
}

/// What an ALERT means: the latched fault by name, then the data warning.
pub(crate) fn alert(c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let d = descriptor(c, id)?;
    let s = c.data_state(id, &d)?;
    if s.fault_code != fault::NONE {
        eprintln!("id {}: fault {}", id.as_byte(), fault_name(s.fault_code));
    }
    warn(c, id, &d)?;
    Ok(())
}

fn fault_name(code: u8) -> String {
    match fault::name(code) {
        Some(n) => format!("{n} ({code})"),
        None => code.to_string(),
    }
}

/// The `osc status` tail: reasons, the operator line and the latched
/// fault.
pub(crate) fn report(c: &mut Client<NusbPipe>, id: Id, d: &Descriptor) -> Result<()> {
    let s = c.data_state(id, d)?;
    if s.flags == 0 {
        println!("      data  clean");
    } else {
        println!("      data  {}", s.names());
        if let Some(msg) = s.message() {
            println!("            {msg}");
        }
    }
    if s.fault_code != fault::NONE {
        println!("      fault {}", fault_name(s.fault_code));
    }
    Ok(())
}

/// The SAVE that closes a write of a set: torque off (every write before
/// it was read-back verified), then SAVE. The data state after is printed
/// and returned.
pub(crate) fn save_set<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<DataState> {
    let f = descriptor::field(d, "torque_enable")?;
    c.write(id, f.addr, &[0])?;
    mgmt_save(c, id)?;
    let s = c.data_state(id, d)?;
    match s.message() {
        None => println!("data state clean"),
        Some(msg) => println!("data state {}: {msg}", s.names()),
    }
    Ok(s)
}

/// MGMT SAVE, torque already off.
pub(crate) fn mgmt_save<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<()> {
    match c.save(id) {
        Ok(()) => println!("saved"),
        Err(Error::Servo(ResultCode::Access)) => {
            bail!("SAVE needs torque disabled (protocol sec 9.4)")
        }
        Err(e) => return Err(e.into()),
    }
    Ok(())
}

/// `osc recover`: a saved table back onto a servo whose data state names a
/// reason.
///
/// No reason needs a wipe: a servo whose images did not load (VIRGIN,
/// CORRUPT, STALE) runs board defaults, and `--from` restores a saved
/// table and SAVEs over them; the normal fresh flow is
/// `osc-ident run`, then `osc-ident save`. `--factory` wipes every saved
/// image first, and the servo comes back factory-fresh on its board id and
/// baud.
///
/// The bench MG90 after a CALIB layout change boots CALIB_STALE; with no
/// motion at all:
///   osc status
///   osc recover --from profiles/mg90-a.json
///   osc status
#[derive(clap::Args, Debug)]
pub(crate) struct RecoverArgs {
    /// Restore CONFIG + CALIB fields from a JSON table: `{"fields": {name:
    /// value}}` (an exported profile) or `{name: value}` (a flat
    /// snapshot). Identity/comms fields, read-only fields and the volatile
    /// blocks are skipped; then SAVE.
    #[arg(long)]
    from: Option<PathBuf>,
    /// FACTORY-reset first: wipe every saved image.
    #[arg(long)]
    factory: bool,
    /// Assume yes: never prompt.
    #[arg(long)]
    yes: bool,
}

pub(crate) fn recover(args: &RecoverArgs, bus: Bus, id: u8) -> Result<()> {
    let mut id = Id::new(id);
    let mut c = crate::bus::connect(&bus, &[id])?;
    let d = descriptor(&mut c, id)?;
    let s = c.data_state(id, &d)?;
    println!("id {}:", id.as_byte());
    report(&mut c, id, &d)?;

    if args.factory {
        if !args.yes
            && !Confirm::new()
                .with_prompt(
                    "FACTORY wipes every saved image (CONFIG and CALIB); the servo reboots to board defaults on its board id and baud. Proceed",
                )
                .default(false)
                .interact()?
        {
            println!("declined, nothing changed");
            return Ok(());
        }
        c.factory(id)?;
        println!(
            "id {}: slots wiped, rebooting to board defaults",
            id.as_byte()
        );
        // The reboot lands on the board id and baud; find the servo again
        // rather than assume either.
        c.pause(Duration::from_millis(200));
        id = refind(&mut c, id)?;
        let s = c.data_state(id, &d)?;
        println!("id {}: {}", id.as_byte(), s.names());
    }

    match &args.from {
        Some(path) => {
            restore(&mut c, id, &d, path)?;
            save_set(&mut c, id, &d)?;
        }
        None if args.factory => {
            println!(
                "next: osc-ident run, then osc-ident save (or osc recover --from <table.json>)"
            );
        }
        None => match s.message() {
            None => println!("nothing to recover"),
            Some(msg) => println!("next: {msg}"),
        },
    }
    Ok(())
}

/// The servo after its FACTORY reboot: at whatever baud the bus answers,
/// on the id it was asked for or, failing that, the only id enumerated.
fn refind(c: &mut Client<NusbPipe>, id: Id) -> Result<Id> {
    match c.find_bus_baud()? {
        Some(rate) => println!("bus at {} baud", rate.as_hz()),
        None => bail!("nothing answers after the FACTORY reboot"),
    }
    match c.ping(id) {
        Ok(_) => return Ok(id),
        Err(Error::Timeout { .. }) => {}
        Err(e) => return Err(e.into()),
    }
    let found = c.discover()?;
    match found.as_slice() {
        [f] => {
            println!(
                "id {} is silent; the servo answers on its board id {} (osc assign moves it back)",
                id.as_byte(),
                f.id.as_byte()
            );
            Ok(f.id)
        }
        [] => bail!("nothing enumerates after the FACTORY reboot"),
        many => {
            let ids: Vec<String> = many.iter().map(|f| f.id.as_byte().to_string()).collect();
            bail!(
                "id {} is silent and {} servos enumerate ({}); pass --id",
                id.as_byte(),
                many.len(),
                ids.join(", ")
            )
        }
    }
}

/// Write a saved table's persisted fields in table order, read-back
/// verified. Cross-field rules can refuse a field until its partner has
/// moved, so a refused field gets one more pass after the rest landed.
fn restore(c: &mut Client<NusbPipe>, id: Id, d: &Descriptor, path: &Path) -> Result<()> {
    let text = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    let json: serde_json::Value =
        serde_json::from_str(&text).with_context(|| format!("parse {}", path.display()))?;
    let map = match json.get("fields") {
        Some(f) => f,
        None => &json,
    }
    .as_object()
    .with_context(|| format!("{}: not a JSON object of fields", path.display()))?;

    let mut pending: Vec<(&Field, Vec<u8>)> = Vec::new();
    let mut skipped: Vec<&str> = Vec::new();
    for (name, value) in map {
        let Some(f) = d.field(name) else {
            skipped.push(name);
            continue;
        };
        if f.access == Access::Ro || !persisted(f) {
            continue;
        }
        let s = match value {
            serde_json::Value::String(s) => s.clone(),
            serde_json::Value::Number(n) => n.to_string(),
            serde_json::Value::Bool(b) => if *b { "on" } else { "off" }.into(),
            other => bail!("{name}: unsupported value {other}"),
        };
        pending.push((f, descriptor::encode(f, &s)?));
    }
    if !skipped.is_empty() {
        println!("skipped (not in the descriptor): {}", skipped.join(", "));
    }
    pending.sort_by_key(|(f, _)| f.addr);
    let total = pending.len();
    for pass in 0..2 {
        let mut refused = Vec::new();
        for (f, bytes) in pending {
            match c.write(id, f.addr, &bytes) {
                Ok(()) => {
                    let back = c.read(id, f.addr, f.width)?;
                    if back != bytes {
                        bail!("{}: wrote {bytes:02x?}, read back {back:02x?}", f.name);
                    }
                }
                Err(Error::Servo(_)) if pass == 0 => refused.push((f, bytes)),
                Err(Error::Servo(code)) => bail!("{}: servo answered {code:?}", f.name),
                Err(e) => return Err(e.into()),
            }
        }
        pending = refused;
        if pending.is_empty() {
            break;
        }
    }
    println!("restored {total} fields from {}", path.display());
    Ok(())
}

/// A field SAVE persists: past the protocol's identity/comms front and
/// before the first volatile block. The identity front would move the
/// servo mid-restore; CONTROL and the blocks after it are not table state.
fn persisted(f: &Field) -> bool {
    f.addr >= CONFIG_COMMON_END && f.addr < CONTROL_BASE_ADDR
}

#[cfg(test)]
mod tests {
    use super::*;

    fn builtin() -> Descriptor {
        let reg = descriptor::load_builtins().unwrap();
        reg.iter().next().unwrap().clone()
    }

    #[test]
    fn persisted_is_config_and_calib_past_the_identity_front() {
        let d = builtin();
        for name in [
            "id",
            "baud_rate_idx",
            "response_deadline_us",
            "model_number",
        ] {
            assert!(!persisted(d.field(name).unwrap()), "{name}");
        }
        for name in [
            "pos_min_phys_counts",
            "current_limit_counts",
            "raw_min",
            "recip_ke_q",
            "rail_drop_mv",
        ] {
            assert!(persisted(d.field(name).unwrap()), "{name}");
        }
        for name in ["torque_enable", "goal_position", "data_flags", "words"] {
            assert!(!persisted(d.field(name).unwrap()), "{name}");
        }
    }
}

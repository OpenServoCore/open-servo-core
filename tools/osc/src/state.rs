//! Data state on every servo verb: the one-line warning a command prints
//! when `data_flags` names a reason (the command still runs - OpenLoop,
//! Current, cal and ident work on such a servo, the closed loops are what
//! the servo refuses), the `osc status` report with the stamp verdict, the
//! torque-off restamp behind `osc set`, and `osc recover`: the only osc
//! verb that SAVEs a CONFIG_CORRUPT servo.

use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use dialoguer::Confirm;
use osc_client::blocking::Client;
use osc_client::data_state::{CONFIG_CORRUPT, DataState, SAVE_CLEARS, STAMP_MISMATCH, fault};
use osc_client::descriptor::{Access, Descriptor, Field};
use osc_client::nusb::NusbPipe;
use osc_client::pipe::Pipe;
use osc_client::stamp::UNSTAMPED;
use osc_client::{Error, Id, ResultCode};
use osc_ident::regs::control;
use osc_protocol::table::CONFIG_COMMON_END;

use crate::descriptor;

/// The servo's descriptor (built-ins plus the operator's overrides, picked
/// by one identity read), owned so callers keep no registry around.
pub(crate) fn descriptor(c: &mut Client<NusbPipe>, id: Id) -> Result<Descriptor> {
    let reg = descriptor::load()?;
    Ok(crate::select_descriptor(c, id, &reg)?.clone())
}

/// Read the data state; a set reason prints the one-line warning.
pub(crate) fn warn(c: &mut Client<NusbPipe>, id: Id, d: &Descriptor) -> Result<DataState> {
    let s = c.data_state(id, d)?;
    if let Some(msg) = s.message() {
        eprintln!("warning: id {}: {} - {msg}", id.as_byte(), s.names());
    }
    Ok(s)
}

/// [`warn`] for the verbs that resolve no descriptor of their own.
pub(crate) fn check(c: &mut Client<NusbPipe>, id: Id) -> Result<DataState> {
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
        println!("id {}: fault {}", id.as_byte(), fault_name(s.fault_code));
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

/// The `osc status` tail: reasons, the operator line, the latched fault
/// and the stamp beside what the live set computes to.
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
    match c.stamp_verdict(id, d) {
        Ok(v) if v.stored == UNSTAMPED => {
            println!("      stamp unstamped; live set {:#06x}", v.computed)
        }
        Ok(v) if v.matches() => println!("      stamp {:#06x} = live set", v.stored),
        Ok(v) => println!(
            "      stamp {:#06x} != live set {:#06x}",
            v.stored, v.computed
        ),
        Err(Error::Descriptor(_)) => {}
        Err(e) => return Err(e.into()),
    }
    Ok(())
}

/// After a hand edit of `f`: a covered write leaves STAMP_MISMATCH standing
/// (the servo marks it; the next closed-loop enable is refused, a running
/// loop is never stopped) and the note names the deliberate ways out. No
/// tool restamps as a side effect of a write: an auto-restamp would bless
/// any edit, plant constants included, as the identified set.
pub(crate) fn covered_note(d: &Descriptor, f: &Field) {
    let covered = match d.stamp() {
        Ok(s) => s.covers(f.addr, f.width),
        Err(_) => false,
    };
    if covered {
        println!(
            "note: {} is stamp-covered: the set is marked stale and closed loop is refused at the next enable; osc stamp [--save] blesses the tuned set, osc ident refits it",
            f.name
        );
    }
}

/// The commit that closes a write of the identified or calibrated set:
/// torque off, the stamp over the set as the servo holds it (every write
/// before it was read-back verified, so this is the intended set), the
/// firmware's checkpoint as the witness, then SAVE when asked. VIRGIN and
/// STALE clear only on SAVE, so a fresh servo opens closed loop after the
/// save, never before. The state after is printed and returned.
pub(crate) fn commit<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    save: bool,
) -> Result<DataState> {
    let f = descriptor::field(d, "torque_enable")?;
    c.write(id, f.addr, &[0])?;
    let stamp = c.restamp(id, d)?;
    let s = c.data_state(id, d)?;
    if s.flags & STAMP_MISMATCH != 0 {
        bail!(
            "id {}: stamp {stamp:#06x} did not verify (a write did not land); torque left off",
            id.as_byte()
        );
    }
    println!("stamped {stamp:#06x}");
    if save {
        match c.save(id) {
            Ok(()) => println!("saved"),
            Err(Error::Servo(ResultCode::Access)) => {
                bail!("SAVE needs torque disabled (protocol sec 9.4)")
            }
            Err(e) => return Err(e.into()),
        }
    } else if s.flags & SAVE_CLEARS != 0 {
        println!(
            "not saved: {} stands until SAVE (osc stamp --save, or osc save)",
            s.names()
        );
    }
    let s = c.data_state(id, d)?;
    match s.message() {
        None => println!("data state clean: closed loop allowed"),
        Some(msg) => println!("data state {}: {msg}", s.names()),
    }
    Ok(s)
}

/// `osc stamp`: bless the set the servo holds now, after deliberate manual
/// tuning (`osc set` of covered fields). The one explicit path besides the
/// tools that write the set themselves (cal, ident, recover --from).
pub(crate) fn stamp(c: &mut Client<NusbPipe>, id: Id, save: bool) -> Result<()> {
    let d = descriptor(c, id)?;
    warn(c, id, &d)?;
    commit(c, id, &d, save)?;
    Ok(())
}

/// `osc recover`: the way back from a data state that refuses closed
/// loop.
///
/// A CONFIG_CORRUPT servo runs board defaults (limits, polarity) and
/// refuses every mode; SAVE never blesses that, so FACTORY + reboot is the
/// only exit, and the servo comes back factory-fresh on its board id and
/// baud. From there either the normal fresh flow (`osc cal`, then `osc
/// ident`) or `--from` restores a saved table, restamps and SAVEs. Every
/// other reason (STALE, VIRGIN, CALIB_CORRUPT, STAMP_MISMATCH, PLANT_UNSET)
/// needs no wipe: `--from` alone brings a saved table back.
///
/// The bench MG90 after a CALIB layout change boots CALIB_STALE |
/// STAMP_MISMATCH | PLANT_UNSET; with no motion at all:
///   osc status
///   osc recover --from profiles/mg90-a.json
///   osc status
#[derive(clap::Args, Debug)]
pub(crate) struct RecoverArgs {
    /// Restore CONFIG + CALIB fields from a JSON table: `{"fields": {name:
    /// value}}` (an exported profile) or `{name: value}` (an `ident write`
    /// snapshot). Identity/comms fields, read-only fields and the volatile
    /// blocks are skipped; then restamp + SAVE.
    #[arg(long)]
    from: Option<PathBuf>,
    /// FACTORY-reset even when the config is not corrupt.
    #[arg(long)]
    factory: bool,
    /// Assume yes: never prompt.
    #[arg(long)]
    yes: bool,
}

pub(crate) fn recover(args: &RecoverArgs, baud: String, id: u8) -> Result<()> {
    let mut c = crate::rig::connect(&baud)?;
    let mut id = Id::new(id);
    let d = descriptor(&mut c, id)?;
    let s = c.data_state(id, &d)?;
    println!("id {}:", id.as_byte());
    report(&mut c, id, &d)?;

    let corrupt = s.flags & CONFIG_CORRUPT != 0;
    if corrupt || args.factory {
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
            commit(&mut c, id, &d, true)?;
        }
        None if corrupt || args.factory => {
            println!("next: osc cal, then osc ident (or osc recover --from <table.json>)");
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
    f.addr >= CONFIG_COMMON_END && f.addr < control::TORQUE_ENABLE.addr
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
            "plant_stamp",
            "rail_drop_mv",
        ] {
            assert!(persisted(d.field(name).unwrap()), "{name}");
        }
        for name in ["torque_enable", "goal_position", "data_flags", "words"] {
            assert!(!persisted(d.field(name).unwrap()), "{name}");
        }
    }
}

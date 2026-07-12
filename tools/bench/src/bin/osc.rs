//! `osc` — the operator multitool for an osc-native bus: discovery, rescue,
//! id/baud management, persistence, profiles, and one-shot reads/writes.
//! The `tool-*` binaries measure distributions; this one operates the bus.

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use bench::cli::{SETTLE_MS, open_pirate, parse_hex, parse_span, parse_u16};
use bench::discover::{EnumOutcome, enum_query, find_bus_baud, walk};
use bench::osc::{
    PROFILE_SPANS_PER_SLOT, REBOOT_SETTLE_MS, RESCUE_PULSE_US, SAVE_SETTLE_MS, StatusFrame,
    build_assign, build_cal, build_factory, build_ping, build_profile_config, build_read,
    build_read_profile, build_reboot, build_save, build_write, profile_slot_addr,
    profile_span_split,
};
use bench::pirate::Client;
use bench::run::xfer;
use bench::{RESCUE_BAUD, SUPPORTED_BAUDS, baud_index};
use clap::{Parser, Subcommand};
use osc_protocol::wire::{ResultCode, UID_LEN};

/// Control-table address of `baud_rate_idx` (osc-core
/// `regions::config::addr::comms::BAUD_RATE_IDX`; value pinned here to keep
/// the heavy core crate out of the bench build).
const BAUD_RATE_IDX_ADDR: u16 = 0x000D;

/// Control-table address of `telemetry.clock.trim_steps` (osc-core
/// `regions::telemetry`; pinned like `BAUD_RATE_IDX_ADDR`). Signed chip trim
/// steps the trim loop has applied, read-only, volatile (§9.3).
const TRIM_STEPS_ADDR: u16 = 0x0244;

#[derive(Parser, Debug)]
#[command(
    name = "osc",
    about = "Operate an osc-native servo bus: discover, rescue, configure."
)]
struct Cli {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long, global = true)]
    port: Option<String>,
    /// Wire baud, or `auto` to find the bus by ENUM probe.
    #[arg(short, long, global = true, default_value = "auto")]
    baud: String,
    /// Servo id for the unicast verbs.
    #[arg(short, long, global = true, default_value_t = 1)]
    id: u8,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Enumerate every servo (UID + id) — all supported bauds, or just --baud.
    Discover,
    /// Rescue-pulse the bus to 0.5 M and enumerate whoever answers.
    Rescue {
        /// After the rescue, write every found servo's baud register and
        /// follow to this rate.
        #[arg(long)]
        set_baud: Option<u32>,
        /// Persist each rescued servo's config with MGMT SAVE (torque must
        /// be off).
        #[arg(long)]
        save: bool,
    },
    /// One broadcast ENUM query (LSB-first bit prefix, §9.2).
    Enum {
        /// Prefix length in bits (0..=128).
        #[arg(long, default_value_t = 0)]
        prefix_len: u8,
        /// Prefix bytes as hex (`ceil(prefix_len/8)` bytes).
        #[arg(long, default_value = "")]
        prefix: String,
    },
    /// Move the servo with this UID to a new id (broadcast ASSIGN).
    Assign {
        /// Target UID: 16 hex bytes, as reported by discover/enum.
        uid: String,
        /// New id (1..=249). Volatile until SAVE.
        new_id: u8,
    },
    /// Switch the servo's baud register and follow it. Volatile until SAVE.
    Baud {
        /// Target baud.
        to: u32,
    },
    /// Persist the live config + profiles (§9.4; torque must be off).
    Save,
    /// Wipe both saved slots; the servo reboots itself to board defaults.
    Factory,
    /// Ack, then reset once the ack has drained.
    Reboot,
    /// Read or configure a §5.2 profile slot.
    Profile {
        #[command(subcommand)]
        cmd: ProfileCmd,
    },
    /// Broadcast a CAL break train (§9.3) and read trim_steps back.
    Cal {
        /// Break spacing in µs, crystal-paced by the pirate.
        #[arg(long, default_value_t = 400)]
        gap_us: u16,
        /// Measured gaps in the train (the pirate sends gaps + 1 breaks).
        #[arg(long, default_value_t = 8)]
        gaps: u8,
        /// Servos to read trim_steps from; defaults to --id.
        #[arg(long, value_delimiter = ',')]
        ids: Vec<u8>,
    },
    /// One ping: model, firmware, turnaround.
    Ping,
    /// Read a control-table span, hex-dumped.
    Read {
        /// Table address (0x-hex or decimal).
        addr: String,
        /// Byte count (single status frame: <= 252).
        #[arg(default_value_t = 4)]
        len: u16,
    },
    /// Write hex bytes at a control-table address.
    Write {
        /// Table address (0x-hex or decimal).
        addr: String,
        /// Payload as hex bytes.
        data: String,
    },
}

#[derive(Subcommand, Debug)]
enum ProfileCmd {
    /// Decode a slot's span words and read the gathered bytes.
    Get {
        #[arg(default_value_t = 0)]
        slot: u8,
    },
    /// Configure a slot's spans (`addr:count`, comma-separated); no spans
    /// clears the slot.
    Set {
        slot: u8,
        #[arg(value_delimiter = ',')]
        spans: Vec<String>,
    },
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Connect for the unicast verbs: fixed baud, or `auto` via ENUM probe.
fn connect(cli: &Cli) -> Result<Client> {
    let mut client = open_pirate(cli.port.as_deref())?;
    match cli.baud.as_str() {
        "auto" => match find_bus_baud(&mut client)? {
            Some(b) => println!("bus at {b} baud"),
            None => bail!("no servo bus found at any supported baud"),
        },
        s => {
            let baud: u32 = s.parse().context("--baud is a rate or `auto`")?;
            client.set_baud(baud)?;
            client.reset()?;
        }
    }
    Ok(client)
}

/// One exchange that must come back OK; surfaces the ALERT bit.
fn xfer_ok(client: &mut Client, wire: &[u8], settle_ms: u64) -> Result<StatusFrame> {
    let ex = xfer(client, wire, settle_ms)?;
    if ex.status.result != Some(ResultCode::Ok) {
        bail!("status {:?}", ex.status.result);
    }
    if ex.status.alert {
        println!("ALERT set: read the telemetry fault register");
    }
    Ok(ex.status)
}

fn ping(client: &mut Client, id: u8) -> Result<()> {
    let ex = xfer(client, &build_ping(id), SETTLE_MS)?;
    if ex.status.result != Some(ResultCode::Ok) {
        bail!("status {:?}", ex.status.result);
    }
    let p = &ex.status.payload;
    if p.len() != 3 {
        bail!("ping payload {} bytes, wanted model(2) + fw(1)", p.len());
    }
    let us = ex.turnaround_ticks as f64 / client.hz_per_us()? as f64;
    println!(
        "id {}  model {:#06x}  fw {}  turnaround {us:.1} us{}",
        ex.status.id,
        u16::from_le_bytes([p[0], p[1]]),
        p[2],
        if ex.status.alert { "  ALERT" } else { "" },
    );
    Ok(())
}

fn read(client: &mut Client, id: u8, addr: u16, len: u16) -> Result<()> {
    // Settle: request + reply wire time plus servo latency and USB slack.
    let settle_ms = (25 + len as u64) * 10_000 / client.current_baud() as u64 + 4;
    let status = xfer_ok(client, &build_read(id, addr, len), settle_ms)?;
    for (i, chunk) in status.payload.chunks(16).enumerate() {
        println!("{:#06x}  {}", addr as usize + i * 16, hex(chunk));
    }
    Ok(())
}

fn save(client: &mut Client, id: u8) -> Result<()> {
    let ex = xfer(client, &build_save(id), SAVE_SETTLE_MS)?;
    match ex.status.result {
        Some(ResultCode::Ok) => {
            println!("id {id}: saved");
            Ok(())
        }
        Some(ResultCode::Access) => bail!("id {id}: SAVE needs torque disabled (§9.4)"),
        r => bail!("id {id}: SAVE nacked: {r:?}"),
    }
}

/// WRITE the baud register (ack arrives at the old rate; the servo applies
/// the change once the ack drains), then follow and verify with a ping.
fn switch_baud(client: &mut Client, ids: &[u8], to: u32) -> Result<()> {
    let idx = baud_index(to)
        .ok_or_else(|| anyhow::anyhow!("unsupported baud {to} (use {SUPPORTED_BAUDS:?})"))?;
    for &id in ids {
        xfer_ok(
            client,
            &build_write(id, BAUD_RATE_IDX_ADDR, &[idx]),
            SETTLE_MS,
        )
        .with_context(|| format!("baud write to id {id}"))?;
    }
    client.set_baud(to)?;
    client.reset()?;
    for &id in ids {
        xfer_ok(client, &build_ping(id), SETTLE_MS)
            .with_context(|| format!("id {id} not answering at {to}"))?;
    }
    println!("answering at {to} baud (volatile until SAVE)");
    Ok(())
}

fn discover(cli: &Cli) -> Result<()> {
    let mut client = open_pirate(cli.port.as_deref())?;
    let bauds: Vec<u32> = match cli.baud.as_str() {
        "auto" => SUPPORTED_BAUDS.to_vec(),
        s => vec![s.parse().context("--baud is a rate or `auto`")?],
    };
    let mut total = 0;
    for baud in bauds {
        client.set_baud(baud)?;
        client.reset()?;
        let found = walk(&mut client)?;
        if found.is_empty() {
            println!("{baud:>8}: -");
        }
        for f in &found {
            println!("{baud:>8}: id {:<3} uid {}", f.id, hex(&f.uid));
        }
        total += found.len();
    }
    println!("{total} servo(s)");
    Ok(())
}

fn rescue(cli: &Cli, set_baud: Option<u32>, do_save: bool) -> Result<()> {
    let mut client = open_pirate(cli.port.as_deref())?;
    client.lowpulse(RESCUE_PULSE_US)?;
    // Servo-side confirm completes ~100 µs after the pulse ends.
    sleep(Duration::from_millis(2));
    client.set_baud(RESCUE_BAUD)?;
    client.reset()?;
    let found = walk(&mut client)?;
    if found.is_empty() {
        bail!("rescue pulse sent but nothing answers at {RESCUE_BAUD} baud");
    }
    for f in &found {
        println!("rescued: id {:<3} uid {}", f.id, hex(&f.uid));
    }
    let ids: Vec<u8> = found.iter().map(|f| f.id).collect();
    if let Some(to) = set_baud {
        switch_baud(&mut client, &ids, to)?;
    } else {
        println!("bus is at {RESCUE_BAUD} baud until reboot (volatile)");
    }
    if do_save {
        for &id in &ids {
            save(&mut client, id)?;
        }
    }
    Ok(())
}

fn read_trim(client: &mut Client, id: u8) -> Result<i8> {
    let status = xfer_ok(client, &build_read(id, TRIM_STEPS_ADDR, 1), SETTLE_MS)?;
    match status.payload.as_slice() {
        [steps] => Ok(*steps as i8),
        p => bail!("trim_steps read returned {} bytes", p.len()),
    }
}

/// Broadcast the CAL announce + break train, then read back each servo's
/// applied trim total. A pre-read failure is reported as `?` rather than
/// aborting — CAL is exactly what rescues a servo railed by a bad trim
/// (§9.3), so it may only answer afterwards.
fn cal(client: &mut Client, gap_us: u16, gaps: u8, ids: &[u8]) -> Result<()> {
    let before: Vec<Option<i8>> = ids.iter().map(|&id| read_trim(client, id).ok()).collect();
    client.cal_train(&build_cal(gap_us, gaps), gap_us as u32, gaps as u32 + 1)?;
    println!("train sent: {gaps} gaps x {gap_us} us");
    // The trim decision applies in the servo main loop between frames —
    // one settle window covers it.
    sleep(Duration::from_millis(SETTLE_MS));
    for (&id, before) in ids.iter().zip(&before) {
        let b = before.map_or("?".into(), |v| v.to_string());
        match read_trim(client, id) {
            Ok(a) => println!("id {id:<3} trim_steps {b} -> {a}"),
            Err(e) => println!("id {id:<3} trim_steps {b} -> read failed: {e}"),
        }
    }
    Ok(())
}

fn profile(client: &mut Client, id: u8, cmd: &ProfileCmd) -> Result<()> {
    match cmd {
        ProfileCmd::Get { slot } => {
            let words = xfer_ok(
                client,
                &build_read(
                    id,
                    profile_slot_addr(*slot),
                    PROFILE_SPANS_PER_SLOT as u16 * 2,
                ),
                SETTLE_MS,
            )?
            .payload;
            let mut total = 0u16;
            for pair in words.as_chunks::<2>().0 {
                let (addr, count) = profile_span_split(u16::from_le_bytes(*pair));
                if count != 0 {
                    println!("slot {slot}: {addr:#06x}:{count}");
                    total += count as u16;
                }
            }
            if total == 0 {
                println!("slot {slot}: empty");
                return Ok(());
            }
            let data = xfer_ok(client, &build_read_profile(id, *slot), SETTLE_MS)?.payload;
            println!("data ({total} B): {}", hex(&data));
            Ok(())
        }
        ProfileCmd::Set { slot, spans } => {
            let spans: Vec<(u16, u8)> =
                spans.iter().map(|s| parse_span(s)).collect::<Result<_>>()?;
            xfer_ok(client, &build_profile_config(id, *slot, &spans), SETTLE_MS)?;
            if spans.is_empty() {
                println!("slot {slot}: cleared");
            } else {
                let total: u16 = spans.iter().map(|&(_, c)| c as u16).sum();
                println!("slot {slot}: {} span(s), {total} B", spans.len());
            }
            Ok(())
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match &cli.cmd {
        Cmd::Discover => discover(&cli),
        Cmd::Rescue { set_baud, save } => rescue(&cli, *set_baud, *save),
        Cmd::Enum { prefix_len, prefix } => {
            let prefix = parse_hex(prefix)?;
            if *prefix_len > 128 || prefix.len() != (*prefix_len as usize).div_ceil(8) {
                bail!("prefix carries ceil(prefix_len/8) bytes, prefix_len <= 128");
            }
            let mut client = connect(&cli)?;
            match enum_query(&mut client, *prefix_len, &prefix)? {
                EnumOutcome::Silent => println!("silent (empty subtree)"),
                EnumOutcome::One { uid, id } => println!("id {id:<3} uid {}", hex(&uid)),
                EnumOutcome::Collision => println!("collision (descend the prefix tree)"),
            }
            Ok(())
        }
        Cmd::Assign { uid, new_id } => {
            let bytes = parse_hex(uid)?;
            let uid: [u8; UID_LEN] = bytes
                .try_into()
                .map_err(|b: Vec<u8>| anyhow::anyhow!("uid is 16 hex bytes, got {}", b.len()))?;
            let mut client = connect(&cli)?;
            let ack = xfer_ok(&mut client, &build_assign(&uid, *new_id), SETTLE_MS)?;
            println!("id {} (volatile until SAVE)", ack.id);
            Ok(())
        }
        Cmd::Baud { to } => {
            let mut client = connect(&cli)?;
            switch_baud(&mut client, &[cli.id], *to)
        }
        Cmd::Save => save(&mut connect(&cli)?, cli.id),
        Cmd::Factory => {
            let mut client = connect(&cli)?;
            xfer_ok(&mut client, &build_factory(cli.id), SAVE_SETTLE_MS)?;
            sleep(Duration::from_millis(REBOOT_SETTLE_MS));
            println!("id {}: slots wiped, rebooted to board defaults", cli.id);
            Ok(())
        }
        Cmd::Reboot => {
            let mut client = connect(&cli)?;
            xfer_ok(&mut client, &build_reboot(cli.id), SETTLE_MS)?;
            sleep(Duration::from_millis(REBOOT_SETTLE_MS));
            println!("id {}: rebooted (back at its configured baud)", cli.id);
            Ok(())
        }
        Cmd::Cal { gap_us, gaps, ids } => {
            let ids = if ids.is_empty() {
                vec![cli.id]
            } else {
                ids.clone()
            };
            cal(&mut connect(&cli)?, *gap_us, *gaps, &ids)
        }
        Cmd::Profile { cmd } => profile(&mut connect(&cli)?, cli.id, cmd),
        Cmd::Ping => ping(&mut connect(&cli)?, cli.id),
        Cmd::Read { addr, len } => read(&mut connect(&cli)?, cli.id, parse_u16(addr)?, *len),
        Cmd::Write { addr, data } => {
            let mut client = connect(&cli)?;
            let data = parse_hex(data)?;
            xfer_ok(
                &mut client,
                &build_write(cli.id, parse_u16(addr)?, &data),
                SETTLE_MS,
            )?;
            println!("wrote {} B at {addr}", data.len());
            Ok(())
        }
    }
}

//! Raw-record smoke tool for the osc-adapter pipe: encode one SUBMIT (or
//! HELLO / ENTER_BOOTLOADER), stream the reply records, pretty-print them.
//! Deliberately session-less -- seq is fixed per invocation, one command in
//! flight, blocking reads until the terminal record lands (Ctrl-C is the
//! timeout). The real client (phase 4) owns everything this doesn't.

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use futures_lite::future::block_on;
use osc_host::link::record;
use osc_protocol::wire::{Id, Inst, Opcode};

const VID: u16 = 0x1209;
const PID: u16 = 0x0001;
const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;

#[derive(Parser)]
#[command(about = "Speak raw osc-host link records to the adapter.")]
struct Args {
    #[command(subcommand)]
    cmd: Cmd,
    /// Client sequence number to tag the SUBMIT with.
    #[arg(long, default_value_t = 1)]
    seq: u16,
}

#[derive(Subcommand)]
enum Cmd {
    /// HELLO -> INFO (link version, tick rate).
    Hello,
    /// PING one servo.
    Ping { id: u8 },
    /// READ count bytes at addr.
    Read { id: u8, addr: u16, count: u16 },
    /// WRITE hex bytes at addr.
    Write {
        id: u8,
        addr: u16,
        hex: String,
        #[arg(long)]
        noreply: bool,
    },
    /// GREAD a chain: comma-separated ids.
    Gread { ids: String, addr: u16, count: u16 },
    /// Arbitrary exchange: raw inst byte + raw hex payload.
    Submit { id: u8, inst: u8, hex: String },
    /// Rescue pulse + both ends to 0.5M.
    Rescue,
    /// Host-side UART rate only (baud index 0-3).
    Baud { idx: u8 },
    /// Engine copy of the fleet RESPONSE_DEADLINE.
    Deadline { us: u16 },
    /// Drive the DUT power rails: on/off for 3V3 then 5V.
    Rails { v3v3: String, v5: String },
    /// Hand the adapter to the WCH bootloader (4348:55e0, wlink-iap).
    Bootloader,
    /// Instrument raw TX: law break + raw hex bytes, engine bypassed.
    WireSend { hex: String },
    /// Instrument burst: comma-separated hex frames, back-to-back.
    WireBurst { frames: String },
    /// Instrument dominant-low pulse of us microseconds.
    Pulse { us: u16 },
    /// Drain captured wire edges (up to max per polarity).
    Drain {
        #[arg(default_value_t = 64)]
        max: u8,
    },
    /// Drop undrained captures + the overflow flag.
    CaptureReset,
}

fn on_off(s: &str) -> Result<bool> {
    match s {
        "on" => Ok(true),
        "off" => Ok(false),
        other => bail!("expected on|off, got {other:?}"),
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    let di = nusb::list_devices()?
        .find(|d| d.vendor_id() == VID && d.product_id() == PID)
        .context("no osc-adapter (1209:0001) on the bus")?;
    let device = di.open()?;
    // Vendor-class device: no OS driver configures it, so pick config 1
    // ourselves before claiming (harmless if already configured).
    let _ = device.set_configuration(1);
    let interface = device.claim_interface(0)?;

    let (out, wait_terminal) = encode(&args)?;
    block_on(interface.bulk_out(EP_OUT, out)).into_result()?;

    let mut pending = Vec::new();
    loop {
        while let Some((body, rest)) = split_record(&pending) {
            let last = print_record(body);
            pending = rest;
            if last || !wait_terminal {
                return Ok(());
            }
        }
        let data = block_on(interface.bulk_in(EP_IN, nusb::transfer::RequestBuffer::new(512)))
            .into_result()?;
        pending.extend_from_slice(&data);
    }
}

/// Encode the command; returns (pipe bytes, whether a terminal is owed).
fn encode(args: &Args) -> Result<(Vec<u8>, bool)> {
    let seq = args.seq;
    Ok(match &args.cmd {
        Cmd::Hello => (rec(&[record::REC_HELLO]), false),
        Cmd::Ping { id } => (
            submit(seq, *id, Inst::instruction(Opcode::Ping, 0).0, &[]),
            true,
        ),
        Cmd::Read { id, addr, count } => {
            let mut p = [0u8; 8];
            let n = osc_protocol::build::read(&mut p, *addr, *count).context("bad read args")?;
            (
                submit(seq, *id, Inst::instruction(Opcode::Read, 0).0, &p[..n]),
                true,
            )
        }
        Cmd::Write {
            id,
            addr,
            hex,
            noreply,
        } => {
            let data = parse_hex(hex)?;
            let mut p = vec![0u8; data.len() + 4];
            let n = osc_protocol::build::write(&mut p, *addr, &data).context("bad write args")?;
            let flags = if *noreply { Inst::FLAG_NOREPLY } else { 0 };
            (
                submit(seq, *id, Inst::instruction(Opcode::Write, flags).0, &p[..n]),
                true,
            )
        }
        Cmd::Gread { ids, addr, count } => {
            let ids: Vec<Id> = ids
                .split(',')
                .map(|s| Ok(Id::new(s.trim().parse()?)))
                .collect::<Result<_>>()?;
            let mut p = vec![0u8; ids.len() + 8];
            let n = osc_protocol::build::gread_uniform(&mut p, *addr, *count, &ids)
                .context("bad gread args")?;
            (
                submit(
                    seq,
                    Id::BROADCAST.as_byte(),
                    Inst::instruction(Opcode::Gread, 0).0,
                    &p[..n],
                ),
                true,
            )
        }
        Cmd::Submit { id, inst, hex } => (submit(seq, *id, *inst, &parse_hex(hex)?), true),
        Cmd::Rescue => (
            rec(&[
                record::REC_SUBMIT,
                seq as u8,
                (seq >> 8) as u8,
                record::VERB_RESCUE,
            ]),
            true,
        ),
        Cmd::Baud { idx } => (
            rec(&[
                record::REC_SUBMIT,
                seq as u8,
                (seq >> 8) as u8,
                record::VERB_HOST_BAUD,
                *idx,
            ]),
            true,
        ),
        Cmd::Deadline { us } => (
            rec(&[
                record::REC_SUBMIT,
                seq as u8,
                (seq >> 8) as u8,
                record::VERB_SET_RESPONSE_DEADLINE,
                *us as u8,
                (us >> 8) as u8,
            ]),
            true,
        ),
        Cmd::Rails { v3v3, v5 } => {
            let state = on_off(v3v3)? as u8 | (on_off(v5)? as u8) << 1;
            (rec(&[record::REC_SET_RAILS, state]), false)
        }
        Cmd::Bootloader => (rec(&[record::REC_ENTER_BOOTLOADER]), false),
        Cmd::WireSend { hex } => {
            let mut body = vec![record::REC_WIRE_SEND, seq as u8, (seq >> 8) as u8];
            body.extend_from_slice(&parse_hex(hex)?);
            (rec(&body), true)
        }
        Cmd::WireBurst { frames } => {
            let mut body = vec![record::REC_WIRE_BURST, seq as u8, (seq >> 8) as u8];
            for f in frames.split(',') {
                let bytes = parse_hex(f)?;
                if bytes.is_empty() || bytes.len() > 255 {
                    bail!("burst frame must be 1..=255 bytes");
                }
                body.push(bytes.len() as u8);
                body.extend_from_slice(&bytes);
            }
            (rec(&body), true)
        }
        Cmd::Pulse { us } => (
            rec(&[
                record::REC_WIRE_PULSE,
                seq as u8,
                (seq >> 8) as u8,
                *us as u8,
                (us >> 8) as u8,
            ]),
            true,
        ),
        Cmd::Drain { max } => (rec(&[record::REC_EDGE_DRAIN, *max]), false),
        Cmd::CaptureReset => (rec(&[record::REC_CAPTURE_RESET]), false),
    })
}

fn rec(body: &[u8]) -> Vec<u8> {
    let mut v = (body.len() as u16).to_le_bytes().to_vec();
    v.extend_from_slice(body);
    v
}

fn submit(seq: u16, id: u8, inst: u8, payload: &[u8]) -> Vec<u8> {
    let mut body = vec![
        record::REC_SUBMIT,
        seq as u8,
        (seq >> 8) as u8,
        record::VERB_EXCHANGE,
        id,
        inst,
    ];
    body.extend_from_slice(payload);
    rec(&body)
}

fn parse_hex(s: &str) -> Result<Vec<u8>> {
    let clean: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !clean.len().is_multiple_of(2) {
        bail!("odd hex length");
    }
    (0..clean.len())
        .step_by(2)
        .map(|i| Ok(u8::from_str_radix(&clean[i..i + 2], 16)?))
        .collect()
}

/// One length-prefixed record off the front, if complete.
fn split_record(bytes: &[u8]) -> Option<(Vec<u8>, Vec<u8>)> {
    if bytes.len() < 2 {
        return None;
    }
    let len = u16::from_le_bytes([bytes[0], bytes[1]]) as usize;
    if bytes.len() < 2 + len {
        return None;
    }
    Some((bytes[2..2 + len].to_vec(), bytes[2 + len..].to_vec()))
}

/// Print one record; true when it ends the conversation.
fn print_record(body: Vec<u8>) -> bool {
    match body[0] {
        record::REC_INFO => {
            let ticks = u32::from_le_bytes([body[2], body[3], body[4], body[5]]);
            println!("INFO: link v{}, {} ticks/us", body[1], ticks);
            true
        }
        record::REC_STATUS => {
            let seq = u16::from_le_bytes([body[1], body[2]]);
            println!(
                "STATUS seq={} slot={} id={} inst={:#04x} payload={}",
                seq,
                body[3],
                body[4],
                body[5],
                hex(&body[6..]),
            );
            false
        }
        record::REC_TERMINAL => {
            let seq = u16::from_le_bytes([body[1], body[2]]);
            let outcome = match body[3] {
                record::OUTCOME_SENT => "SENT".into(),
                record::OUTCOME_COMPLETE => "COMPLETE".into(),
                record::OUTCOME_TIMEOUT => format!("TIMEOUT slot={}", body[4]),
                other => format!("outcome {other:#04x}"),
            };
            let tick = u32::from_le_bytes([body[5], body[6], body[7], body[8]]);
            println!(
                "TERMINAL seq={} {} tick={} statuses={} garble={} trailing={}",
                seq,
                outcome,
                tick,
                body[9],
                u16::from_le_bytes([body[10], body[11]]),
                body[12] & record::FLAG_GARBLE_AFTER_LAST_FRAME != 0,
            );
            true
        }
        record::REC_REJECTED => {
            let seq = u16::from_le_bytes([body[1], body[2]]);
            let reason = match body[3] {
                record::REASON_BAD_ID => "bad id",
                record::REASON_BAD_INST => "bad inst",
                record::REASON_BAD_PAYLOAD => "bad payload",
                record::REASON_TOO_LONG => "too long",
                record::REASON_BUSY => "busy",
                record::REASON_MALFORMED => "malformed",
                _ => "?",
            };
            println!("REJECTED seq={seq} ({reason})");
            true
        }
        record::REC_BOOTLOADER_ACK => {
            println!("BOOTLOADER ACK -- adapter resets to 4348:55e0; use wlink-iap");
            true
        }
        record::REC_RAILS_ACK => {
            println!(
                "RAILS: 3V3 {} / 5V {}",
                if body[1] & 1 != 0 { "on" } else { "off" },
                if body[1] & 2 != 0 { "on" } else { "off" },
            );
            true
        }
        record::REC_UNKNOWN => {
            println!("UNKNOWN record type {:#04x} (adapter echo)", body[1]);
            true
        }
        record::REC_WIRE_DONE => {
            let seq = u16::from_le_bytes([body[1], body[2]]);
            let tick = u32::from_le_bytes([body[3], body[4], body[5], body[6]]);
            println!("WIRE DONE seq={seq} tick={tick}");
            true
        }
        record::REC_EDGES => {
            let now = u32::from_le_bytes([body[2], body[3], body[4], body[5]]);
            let (fall_n, rise_n) = (body[6] as usize, body[7] as usize);
            let tick = |i: usize| u16::from_le_bytes([body[8 + 2 * i], body[9 + 2 * i]]);
            let list = |from: usize, n: usize| {
                (from..from + n)
                    .map(|i| tick(i).to_string())
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            println!(
                "EDGES overflow={} now={} falls[{}]: {}",
                body[1] & 1 != 0,
                now,
                fall_n,
                list(0, fall_n),
            );
            println!("  rises[{}]: {}", rise_n, list(fall_n, rise_n));
            true
        }
        record::REC_CAPTURE_ACK => {
            println!("CAPTURE RESET");
            true
        }
        other => {
            let seq_note = if body.len() >= 3 {
                format!(" seq?={}", u16::from_le_bytes([body[1], body[2]]))
            } else {
                String::new()
            };
            println!("unrecognized record {other:#04x}{seq_note}: {}", hex(&body));
            false
        }
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

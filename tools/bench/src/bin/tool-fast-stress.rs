//! Repeated fast-instruction shots + counter-probe wedge detection.
//!
//! Modes:
//!   default     One cell at `--baud` / `--position` / `--dut-len`, repeated
//!               `--shots` times. Optional `--scope-delay-s` sleep before the
//!               first shot for arming a scope.
//!   --continuous  Same cell, hammered until `--wedge-threshold` consecutive
//!               NOREPLY/other shots, `--max-shots`, or a counter-probe wedge.
//!   --matrix    Walks (`--matrix-bauds` × `--matrix-dut-lens` ×
//!               `--matrix-positions`). Per-cell counter delta + timing
//!               verdict. Stops on the first wedge.
//!
//! Position semantics (mirrors `pirate_chip_stress.py`, plus middle):
//!   only    plain Read; chip is the sole responder.
//!   first   Fast Bulk Read [(chip), (FOREIGN_ID)] — chip fires slot 0, bus
//!           IDLEs after (FOREIGN_ID never replies).
//!   middle  Fast Bulk Read [(INJ_ID), (chip), (FOREIGN_ID)] — pirate ARMs
//!           INJ predecessor, chip emits middle-slot body (no header, no
//!           CRC), FOREIGN_ID never replies.
//!   last    Fast Bulk Read [(INJ_ID), (chip)] — pirate ARMs INJ predecessor
//!           reply so chip's snoop walk + chain CRC patch fire as in
//!           production.

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use bench::{
    FOREIGN_ID, INJ_ID, IdleStamp, LinkCounters, PirateClient, RETURN_DELAY_2US_ADDR, Round,
    Session, SessionArgs, build_fast_bulk_read, build_inj_first_bytes, build_read, clear_counters,
    parse_fast_response, parse_status_reply, read_counters, read_ct_u8, round_from_stamps,
    set_chip_baud, try_read_counters,
};
use clap::{Parser, ValueEnum};
use dxl_protocol::types::{BulkReadEntry, Id, StatusError};

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Position {
    Only,
    First,
    Middle,
    Last,
}

impl Position {
    fn as_str(self) -> &'static str {
        match self {
            Self::Only => "only",
            Self::First => "first",
            Self::Middle => "middle",
            Self::Last => "last",
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Bucket {
    Clean,
    ExtraIdle,
    Crc,
    Other,
    Noreply,
}

impl Bucket {
    const ALL: [Self; 5] = [
        Self::Clean,
        Self::ExtraIdle,
        Self::Crc,
        Self::Other,
        Self::Noreply,
    ];
    fn as_str(self) -> &'static str {
        match self {
            Self::Clean => "clean",
            Self::ExtraIdle => "extra_idle",
            Self::Crc => "crc",
            Self::Other => "other",
            Self::Noreply => "noreply",
        }
    }
    fn is_wedge(self) -> bool {
        matches!(self, Self::Noreply | Self::Other)
    }
}

struct Tally([u32; 5]);
impl Tally {
    fn new() -> Self {
        Self([0; 5])
    }
    fn bump(&mut self, b: Bucket) {
        self.0[Bucket::ALL.iter().position(|x| *x == b).unwrap()] += 1;
    }
    fn get(&self, b: Bucket) -> u32 {
        self.0[Bucket::ALL.iter().position(|x| *x == b).unwrap()]
    }
}

struct ShotOutcome {
    bucket: Bucket,
    request: Vec<u8>,
    reply: Vec<u8>,
    round: Option<Round>,
}

#[derive(Parser, Debug)]
#[command(about = "Fast-instruction stress harness with counter-probe wedge detection.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Wire baud (default-mode + --continuous). --matrix walks
    /// --matrix-bauds instead.
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(long, value_enum, default_value_t = Position::First)]
    position: Position,
    /// Predecessor INJ slot data length (--position last only).
    #[arg(long, default_value_t = 4)]
    inj_len: usize,
    /// Chip reply data length (slot payload bytes).
    #[arg(long, default_value_t = 1)]
    dut_len: usize,
    /// Shots in default mode (single-cell). Ignored under --continuous/--matrix.
    #[arg(long, default_value_t = 128)]
    shots: u32,
    /// Post-MASTER capture wait for first/last shots (ms). Sets per-shot
    /// duration: master fires, we sleep this long, then capture RX + stamps.
    #[arg(long, default_value_t = 1.0)]
    shot_wait_ms: f64,
    /// Sleep between shots (continuous + matrix). 0 = back-to-back.
    #[arg(long, default_value_t = 0.0)]
    inter_shot_ms: f64,
    /// Pre-shot sleep in default mode for scope arming.
    #[arg(long, default_value_t = 0.0)]
    scope_delay_s: f64,

    /// Continuous mode: same cell until --wedge-threshold consecutive
    /// NOREPLY/other or --max-shots.
    #[arg(long)]
    continuous: bool,
    #[arg(long, default_value_t = 10_000)]
    max_shots: u32,
    #[arg(long, default_value_t = 5)]
    wedge_threshold: u32,

    /// Matrix mode: walk (baud × dut_len × position) cells.
    #[arg(long)]
    matrix: bool,
    #[arg(long, default_value = "1000000,2000000,3000000")]
    matrix_bauds: String,
    #[arg(long, default_value = "1,4,32")]
    matrix_dut_lens: String,
    #[arg(long, default_value = "only,first,last")]
    matrix_positions: String,
    #[arg(long, default_value_t = 128)]
    matrix_n: u32,
    #[arg(long, default_value_t = 1)]
    matrix_repeat: u32,
    /// Probe chip counters every N shots inside a cell (0 = end-of-cell
    /// only). Smaller values bracket a wedge tighter at the cost of throughput.
    #[arg(long, default_value_t = 0)]
    matrix_health_every: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut session = Session::start(SessionArgs {
        port: args.port.clone(),
        target_baud: args.baud,
    })?;
    let id = Id::new(session.id);
    println!(
        "pirate: {}   chip id: {}",
        session.pirate.port_path(),
        session.id
    );

    if args.matrix {
        return run_matrix(&mut session, id, &args);
    }
    if args.continuous {
        return run_continuous(&mut session, id, &args);
    }
    run_default(&mut session, id, &args)
}

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------

fn run_default(session: &mut Session, id: Id, args: &Args) -> Result<()> {
    println!(
        "baud: {}   position: {}   INJ_LEN: {}   DUT_LEN: {}",
        session.baud,
        args.position.as_str(),
        args.inj_len,
        args.dut_len
    );

    let before = read_counters(&mut session.pirate, id)?;
    clear_counters(&mut session.pirate, id)?;

    if args.scope_delay_s > 0.0 {
        println!("[arming scope in {:.1}s]", args.scope_delay_s);
        sleep(Duration::from_secs_f64(args.scope_delay_s));
    }

    let ctx = CellCtx::new(&mut session.pirate, id, session.baud, args)?;
    let stats = run_cell(
        &mut session.pirate,
        id,
        &ctx,
        args.shots,
        args.shot_wait_ms,
        args.inter_shot_ms,
        0,
    )?;
    print_cell_summary("single", &stats, &ctx);
    for bucket in Bucket::ALL {
        if let Some((shot_i, req, reply)) = stats.first_per_bucket(bucket) {
            print_capture(
                &format!("first {} (shot {})", bucket.as_str(), shot_i),
                req,
                reply,
            );
        }
    }

    let after = try_read_counters(&mut session.pirate, id);
    let _ = before;
    match after {
        None => {
            print_wedge_banner("after shots");
            std::process::exit(1);
        }
        Some(curr) => {
            let zero = LinkCounters::default();
            let delta = curr.delta_from(&zero);
            print_counter_delta("counters bumped this run", &delta);
        }
    }
    Ok(())
}

fn run_continuous(session: &mut Session, id: Id, args: &Args) -> Result<()> {
    println!(
        "baud: {}   position: {}   INJ_LEN: {}   DUT_LEN: {}",
        session.baud,
        args.position.as_str(),
        args.inj_len,
        args.dut_len
    );
    println!(
        "[continuous — up to {} shots; stop on {} consecutive NOREPLY/other]",
        args.max_shots, args.wedge_threshold
    );
    let before = read_counters(&mut session.pirate, id)?;
    clear_counters(&mut session.pirate, id)?;

    let ctx = CellCtx::new(&mut session.pirate, id, session.baud, args)?;
    let mut stats = CellStats::new();
    let mut consecutive_wedge: u32 = 0;
    let mut wedge_shot: Option<u32> = None;
    for i in 1..=args.max_shots {
        let out = run_shot(
            &mut session.pirate,
            id,
            args.position,
            &ctx,
            args.shot_wait_ms,
        )?;
        consecutive_wedge = if out.bucket.is_wedge() {
            consecutive_wedge + 1
        } else {
            0
        };
        stats.record(i, &out, &ctx);
        if consecutive_wedge >= args.wedge_threshold || i.is_multiple_of(100) {
            print_progress_line(i, &out, &stats, &ctx, consecutive_wedge);
        }
        if consecutive_wedge >= args.wedge_threshold {
            wedge_shot = Some(i);
            break;
        }
        if args.inter_shot_ms > 0.0 {
            sleep(Duration::from_secs_f64(args.inter_shot_ms / 1000.0));
        }
    }
    for bucket in Bucket::ALL {
        if let Some((shot_i, req, reply)) = stats.first_per_bucket(bucket) {
            print_capture(
                &format!("first {} (shot {})", bucket.as_str(), shot_i),
                req,
                reply,
            );
        }
    }
    let after = try_read_counters(&mut session.pirate, id);
    if let Some(shot) = wedge_shot {
        print_wedge_banner(&format!(
            "at shot {shot} ({} consecutive NOREPLY/other)",
            args.wedge_threshold
        ));
    } else if after.is_none() {
        print_wedge_banner("after run completed");
    } else {
        println!("\n[completed {} shots without wedge]", args.max_shots);
    }
    print_cell_summary("final", &stats, &ctx);
    let _ = before;
    if let Some(curr) = after {
        let delta = curr.delta_from(&LinkCounters::default());
        print_counter_delta("counter deltas this run", &delta);
    } else {
        println!("counter deltas: unavailable — chip not responding.");
    }
    if wedge_shot.is_some() || after.is_none() {
        std::process::exit(1);
    }
    Ok(())
}

fn run_matrix(session: &mut Session, id: Id, args: &Args) -> Result<()> {
    let bauds = parse_int_list(&args.matrix_bauds)?;
    let dut_lens: Vec<usize> = parse_int_list(&args.matrix_dut_lens)?
        .into_iter()
        .map(|v| v as usize)
        .collect();
    let positions = parse_position_list(&args.matrix_positions)?;
    println!(
        "[matrix — bauds={bauds:?} dut_lens={dut_lens:?} positions={:?}  n={}/cell  health_every={}]",
        positions.iter().map(|p| p.as_str()).collect::<Vec<_>>(),
        args.matrix_n,
        if args.matrix_health_every == 0 {
            "cell-end".to_string()
        } else {
            args.matrix_health_every.to_string()
        }
    );

    for iteration in 1..=args.matrix_repeat {
        if args.matrix_repeat > 1 {
            println!(
                "\n=== matrix iteration {iteration}/{} ===",
                args.matrix_repeat
            );
        }
        let wedged = walk_matrix(&mut session.pirate, id, &bauds, &dut_lens, &positions, args)?;
        if wedged {
            std::process::exit(1);
        }
    }
    Ok(())
}

fn walk_matrix(
    pirate: &mut PirateClient,
    id: Id,
    bauds: &[u32],
    dut_lens: &[usize],
    positions: &[Position],
    args: &Args,
) -> Result<bool> {
    let mut last_counters = read_counters(pirate, id).unwrap_or_default();
    for (baud_idx, &baud) in bauds.iter().enumerate() {
        if baud_idx > 0 {
            println!();
        }
        if let Err(e) = set_chip_baud(pirate, id.as_byte(), baud) {
            println!("  {}M baud: set_chip_baud failed: {e}", baud / 1_000_000);
            if try_read_counters(pirate, id).is_none() {
                print_wedge_banner(&format!("on set_chip_baud({baud})"));
                return Ok(true);
            }
            continue;
        }
        // Re-baseline after set_chip_baud's warmup absorbs the baud-switch
        // glitch — the FE/NE bumps from the switch are already in the
        // counters; snapshot here so the first cell's delta starts
        // post-switch.
        if let Some(rebaseline) = try_read_counters(pirate, id) {
            last_counters = rebaseline;
        }
        for &dut_len in dut_lens {
            for &pos in positions {
                let cell_args = Args {
                    position: pos,
                    dut_len,
                    ..clone_args(args)
                };
                let ctx = CellCtx::new(pirate, id, baud, &cell_args)?;
                let cell = format!("{}M {:<5} {:>3}B", baud / 1_000_000, pos.as_str(), dut_len);
                let stats = match run_cell(
                    pirate,
                    id,
                    &ctx,
                    args.matrix_n,
                    args.shot_wait_ms,
                    args.inter_shot_ms,
                    args.matrix_health_every,
                ) {
                    Ok(s) => s,
                    Err(e) => {
                        println!("  {cell:<22} ABORT ({e})");
                        return Ok(true);
                    }
                };
                let summary = stats.format_summary();
                let timing = format_timing(&stats, &ctx);
                if let Some(wedge_shot) = stats.wedge_shot {
                    println!("  {cell:<22} {summary}  {timing}  WEDGE@shot{wedge_shot}");
                    if let Some((shot_i, req, reply)) = stats.first_non_clean() {
                        print_capture(
                            &format!("first non-clean in {cell} (shot {shot_i})"),
                            req,
                            reply,
                        );
                    }
                    print_wedge_banner(&format!(
                        "in cell {cell} between shot {} and {wedge_shot}",
                        wedge_shot.saturating_sub(args.matrix_health_every) + 1
                    ));
                    return Ok(true);
                }
                let health = try_read_counters(pirate, id);
                let tag = match &health {
                    None => "WEDGE@cell-end".to_string(),
                    Some(h) => {
                        let delta = h.delta_from(&last_counters);
                        last_counters = *h;
                        if delta.is_empty() {
                            "ok".to_string()
                        } else {
                            format!("ok ({})", format_counter_delta(&delta))
                        }
                    }
                };
                println!("  {cell:<22} {summary}  {timing}  {tag}");
                if let Some((shot_i, req, reply)) = stats.first_non_clean() {
                    print_capture(
                        &format!("first non-clean in {cell} (shot {shot_i})"),
                        req,
                        reply,
                    );
                }
                if health.is_none() {
                    print_wedge_banner(&format!("after cell {cell}"));
                    return Ok(true);
                }
            }
        }
    }
    Ok(false)
}

// ---------------------------------------------------------------------------
// Cell + shot mechanics
// ---------------------------------------------------------------------------

struct CellCtx {
    target_ticks: f64,
    thresh_us: f64,
    ticks_per_us: f64,
    position: Position,
    dut_len: usize,
    inj_len: usize,
}

impl CellCtx {
    fn new(pirate: &mut PirateClient, id: Id, baud: u32, args: &Args) -> Result<Self> {
        let ticks_per_us = pirate.hz_per_us()? as f64;
        let byte_time_ticks = 10.0 * ticks_per_us * 1_000_000.0 / baud as f64;
        let byte_time_us = byte_time_ticks / ticks_per_us;
        let rdt_us = read_ct_u8(pirate, id, RETURN_DELAY_2US_ADDR)? as f64 * 2.0;
        let target_ticks = match args.position {
            Position::Last => {
                // 2-slot chain (INJ + chip) including CRC; span = (last - first).
                let packet_length = 1 + (2 + args.inj_len) + (2 + args.dut_len) + 2;
                let reply_bytes = 7 + packet_length;
                (reply_bytes - 1) as f64 * byte_time_ticks
            }
            Position::Middle => {
                // 3-slot chain captured up to chip's last byte: FOREIGN never
                // emits and the chain CRC patch deadline misses, so the bytes
                // we see are INJ's first-slot (10 + inj_len) + chip's middle
                // body (2 + dut_len). Span = (last - first).
                let reply_bytes = 10 + args.inj_len + 2 + args.dut_len;
                (reply_bytes - 1) as f64 * byte_time_ticks
            }
            _ => rdt_us * ticks_per_us + byte_time_ticks,
        };
        Ok(Self {
            target_ticks,
            thresh_us: byte_time_us / 2.0,
            ticks_per_us,
            position: args.position,
            dut_len: args.dut_len,
            inj_len: args.inj_len,
        })
    }
}

/// (shot_index, request_bytes, reply_bytes) of the first capture in a bucket.
type FirstCapture = (u32, Vec<u8>, Vec<u8>);

struct CellStats {
    tally: Tally,
    offsets: Vec<f64>,
    first_per_bucket: [Option<FirstCapture>; 5],
    first_fail: Option<(u32, Bucket, Vec<u8>, Vec<u8>)>,
    wedge_shot: Option<u32>,
}

impl CellStats {
    fn new() -> Self {
        Self {
            tally: Tally::new(),
            offsets: Vec::new(),
            first_per_bucket: Default::default(),
            first_fail: None,
            wedge_shot: None,
        }
    }
    fn record(&mut self, shot: u32, out: &ShotOutcome, ctx: &CellCtx) {
        self.tally.bump(out.bucket);
        let slot = Bucket::ALL.iter().position(|b| *b == out.bucket).unwrap();
        if self.first_per_bucket[slot].is_none() {
            self.first_per_bucket[slot] = Some((shot, out.request.clone(), out.reply.clone()));
        }
        if !matches!(out.bucket, Bucket::Clean | Bucket::ExtraIdle) && self.first_fail.is_none() {
            self.first_fail = Some((shot, out.bucket, out.request.clone(), out.reply.clone()));
        }
        if let (Bucket::Clean | Bucket::ExtraIdle, Some(r)) = (out.bucket, out.round) {
            let raw = match ctx.position {
                Position::Last => r.last.wrapping_sub(r.first) as f64,
                _ => r.first.wrapping_sub(r.req) as f64,
            };
            self.offsets.push(raw - ctx.target_ticks);
        }
    }
    fn first_non_clean(&self) -> Option<(u32, &[u8], &[u8])> {
        self.first_fail
            .as_ref()
            .map(|(i, _, req, reply)| (*i, req.as_slice(), reply.as_slice()))
    }
    fn first_per_bucket(&self, bucket: Bucket) -> Option<(u32, &[u8], &[u8])> {
        let slot = Bucket::ALL.iter().position(|b| *b == bucket).unwrap();
        self.first_per_bucket[slot]
            .as_ref()
            .map(|(i, req, reply)| (*i, req.as_slice(), reply.as_slice()))
    }
    fn format_summary(&self) -> String {
        Bucket::ALL
            .iter()
            .map(|b| format!("{:>10}", self.tally.get(*b)))
            .collect::<Vec<_>>()
            .join("   ")
    }
}

fn run_cell(
    pirate: &mut PirateClient,
    id: Id,
    ctx: &CellCtx,
    n: u32,
    shot_wait_ms: f64,
    inter_shot_ms: f64,
    health_every: u32,
) -> Result<CellStats> {
    let mut stats = CellStats::new();
    for shot in 1..=n {
        let out = run_shot(pirate, id, ctx.position, ctx, shot_wait_ms)?;
        stats.record(shot, &out, ctx);
        if health_every > 0
            && shot.is_multiple_of(health_every)
            && try_read_counters(pirate, id).is_none()
        {
            stats.wedge_shot = Some(shot);
            break;
        }
        if inter_shot_ms > 0.0 {
            sleep(Duration::from_secs_f64(inter_shot_ms / 1000.0));
        }
    }
    Ok(stats)
}

fn run_shot(
    pirate: &mut PirateClient,
    id: Id,
    pos: Position,
    ctx: &CellCtx,
    shot_wait_ms: f64,
) -> Result<ShotOutcome> {
    match pos {
        Position::Only => shot_only(pirate, id, ctx.dut_len),
        Position::First => shot_first(pirate, id, ctx.dut_len, shot_wait_ms),
        Position::Middle => shot_middle(pirate, id, ctx.inj_len, ctx.dut_len, shot_wait_ms),
        Position::Last => shot_last(pirate, id, ctx.inj_len, ctx.dut_len, shot_wait_ms),
    }
}

fn shot_only(pirate: &mut PirateClient, id: Id, dut_len: usize) -> Result<ShotOutcome> {
    let request = build_read(id, 0, dut_len as u16)?;
    let _ = pirate.drain_stamps()?;
    let reply = pirate.xfer(&request, 10_000)?;
    let stamps = pirate.drain_stamps()?;
    let round = round_from_stamps(&stamps);
    let request = request.to_vec();
    let Some(reply) = reply else {
        return Ok(ShotOutcome {
            bucket: Bucket::Noreply,
            request,
            reply: Vec::new(),
            round,
        });
    };
    if reply.len() < 11 {
        return Ok(ShotOutcome {
            bucket: Bucket::Noreply,
            request,
            reply,
            round,
        });
    }
    let bucket = match parse_status_reply(&reply, None) {
        Ok(decoded) if decoded.id.as_byte() == id.as_byte() => Bucket::Clean,
        Ok(_) => Bucket::Other,
        Err(_) => Bucket::Crc,
    };
    Ok(ShotOutcome {
        bucket,
        request,
        reply,
        round,
    })
}

fn shot_first(
    pirate: &mut PirateClient,
    id: Id,
    dut_len: usize,
    shot_wait_ms: f64,
) -> Result<ShotOutcome> {
    let entries = [
        BulkReadEntry {
            id,
            address: 0,
            length: dut_len as u16,
        },
        BulkReadEntry {
            id: Id::new(FOREIGN_ID),
            address: 0,
            length: 1,
        },
    ];
    let request = build_fast_bulk_read(&entries)?;
    let _ = pirate.drain_stamps()?;
    let b0 = pirate.bytes_count()?;
    pirate.master(&request)?;
    sleep(Duration::from_secs_f64(shot_wait_ms / 1000.0));
    let b1 = pirate.bytes_count()?;
    let total = b1.wrapping_sub(b0);
    let capture = total.min(256);
    let all_rx = if capture > 0 {
        pirate.rx_range(b0, capture as u16)?
    } else {
        Vec::new()
    };
    let reply = if all_rx.len() >= request.len() {
        all_rx[request.len()..].to_vec()
    } else {
        Vec::new()
    };
    let stamps = pirate.drain_stamps()?;
    let round = round_from_stamps(&stamps);
    let request = request.to_vec();

    // First-slot wire bytes: 4 sync + bcast_id + 2 len + STATUS + err +
    // slot_id + N data — total 10 + N.
    let first_slot_bytes = 10 + dut_len;
    if total > 256 {
        return Ok(ShotOutcome {
            bucket: Bucket::Other,
            request,
            reply,
            round,
        });
    }
    if total < (request.len() + first_slot_bytes / 2) as u32 || reply.len() < first_slot_bytes {
        return Ok(ShotOutcome {
            bucket: Bucket::Noreply,
            request,
            reply,
            round,
        });
    }
    let header_ok = reply[..4] == [0xFF, 0xFF, 0xFD, 0x00]
        && reply[4] == 0xFE
        && reply[7] == 0x55
        && reply[9] == id.as_byte();
    let bucket = if header_ok {
        Bucket::Clean
    } else {
        Bucket::Crc
    };
    Ok(ShotOutcome {
        bucket,
        request,
        reply,
        round,
    })
}

fn shot_last(
    pirate: &mut PirateClient,
    id: Id,
    inj_len: usize,
    dut_len: usize,
    shot_wait_ms: f64,
) -> Result<ShotOutcome> {
    let inj_data = vec![0xAAu8; inj_len];
    let packet_length = (1 + (2 + inj_len) + (2 + dut_len) + 2) as u16;
    let inj_bytes =
        build_inj_first_bytes(Id::new(INJ_ID), StatusError::OK, &inj_data, packet_length)?;
    let entries = [
        BulkReadEntry {
            id: Id::new(INJ_ID),
            address: 0,
            length: inj_len as u16,
        },
        BulkReadEntry {
            id,
            address: 0,
            length: dut_len as u16,
        },
    ];
    let request = build_fast_bulk_read(&entries)?;
    let _ = pirate.drain_stamps()?;
    let b0 = pirate.bytes_count()?;
    pirate.arm(&inj_bytes, 250 * 18)?;
    pirate.master(&request)?;
    sleep(Duration::from_secs_f64(shot_wait_ms / 1000.0));
    let b1 = pirate.bytes_count()?;
    let total = b1.wrapping_sub(b0);
    let capture = total.min(256);
    let all_rx = if capture > 0 {
        pirate.rx_range(b0, capture as u16)?
    } else {
        Vec::new()
    };
    let reply = if all_rx.len() >= request.len() {
        all_rx[request.len()..].to_vec()
    } else {
        Vec::new()
    };
    let stamps = pirate.drain_stamps()?;
    let round = round_from_stamps(&stamps);
    let extra_idle = stamps
        .iter()
        .filter(|s| matches!(s, IdleStamp::Round(_)))
        .count()
        != 1
        || stamps.iter().any(|s| matches!(s, IdleStamp::Plain(_)));
    let request = request.to_vec();

    let expected_min = 11 + inj_len + 2 + dut_len;
    if total < (request.len() + expected_min / 2) as u32 {
        return Ok(ShotOutcome {
            bucket: Bucket::Noreply,
            request,
            reply,
            round,
        });
    }
    if total > 256 {
        return Ok(ShotOutcome {
            bucket: Bucket::Other,
            request,
            reply,
            round,
        });
    }
    if reply.len() < 11 {
        return Ok(ShotOutcome {
            bucket: Bucket::Noreply,
            request,
            reply,
            round,
        });
    }
    let bucket = match parse_fast_response(&reply, &[inj_len, dut_len]) {
        Err(_) => Bucket::Crc,
        Ok(slots)
            if slots.len() != 2 || slots[0].id.as_byte() != INJ_ID || slots[0].data != inj_data =>
        {
            Bucket::Crc
        }
        Ok(slots) if slots[1].id.as_byte() != id.as_byte() || slots[1].data.len() != dut_len => {
            Bucket::Other
        }
        Ok(_) if extra_idle => Bucket::ExtraIdle,
        Ok(_) => Bucket::Clean,
    };
    Ok(ShotOutcome {
        bucket,
        request,
        reply,
        round,
    })
}

fn shot_middle(
    pirate: &mut PirateClient,
    id: Id,
    inj_len: usize,
    dut_len: usize,
    shot_wait_ms: f64,
) -> Result<ShotOutcome> {
    let inj_data = vec![0xAAu8; inj_len];
    // 3-slot chain reply structure: STATUS + (err+id+inj_data) +
    // (err+id+chip_data) + (err+id+1B_foreign) + CRC.
    let packet_length = (1 + (2 + inj_len) + (2 + dut_len) + (2 + 1) + 2) as u16;
    let inj_bytes =
        build_inj_first_bytes(Id::new(INJ_ID), StatusError::OK, &inj_data, packet_length)?;
    let entries = [
        BulkReadEntry {
            id: Id::new(INJ_ID),
            address: 0,
            length: inj_len as u16,
        },
        BulkReadEntry {
            id,
            address: 0,
            length: dut_len as u16,
        },
        BulkReadEntry {
            id: Id::new(FOREIGN_ID),
            address: 0,
            length: 1,
        },
    ];
    let request = build_fast_bulk_read(&entries)?;
    let _ = pirate.drain_stamps()?;
    let b0 = pirate.bytes_count()?;
    pirate.arm(&inj_bytes, 250 * 18)?;
    pirate.master(&request)?;
    sleep(Duration::from_secs_f64(shot_wait_ms / 1000.0));
    let b1 = pirate.bytes_count()?;
    let total = b1.wrapping_sub(b0);
    let capture = total.min(256);
    let all_rx = if capture > 0 {
        pirate.rx_range(b0, capture as u16)?
    } else {
        Vec::new()
    };
    let reply = if all_rx.len() >= request.len() {
        all_rx[request.len()..].to_vec()
    } else {
        Vec::new()
    };
    let stamps = pirate.drain_stamps()?;
    let round = round_from_stamps(&stamps);
    let request = request.to_vec();

    // Captured reply ends at chip's last middle byte; FOREIGN never replies.
    let first_slot_bytes = 10 + inj_len;
    let middle_bytes = 2 + dut_len;
    let expected_min = first_slot_bytes + middle_bytes;
    if total < (request.len() + expected_min / 2) as u32 {
        return Ok(ShotOutcome {
            bucket: Bucket::Noreply,
            request,
            reply,
            round,
        });
    }
    if total > 256 {
        return Ok(ShotOutcome {
            bucket: Bucket::Other,
            request,
            reply,
            round,
        });
    }
    if reply.len() < expected_min {
        return Ok(ShotOutcome {
            bucket: Bucket::Noreply,
            request,
            reply,
            round,
        });
    }
    let header_ok = reply[..4] == [0xFF, 0xFF, 0xFD, 0x00]
        && reply[4] == 0xFE
        && reply[7] == 0x55
        && reply[9] == INJ_ID
        && reply[10..10 + inj_len] == inj_data[..];
    if !header_ok {
        return Ok(ShotOutcome {
            bucket: Bucket::Crc,
            request,
            reply,
            round,
        });
    }
    let chip_id = reply[first_slot_bytes + 1];
    let bucket = if chip_id != id.as_byte() {
        Bucket::Other
    } else {
        Bucket::Clean
    };
    Ok(ShotOutcome {
        bucket,
        request,
        reply,
        round,
    })
}

// ---------------------------------------------------------------------------
// Printing
// ---------------------------------------------------------------------------

fn print_cell_summary(label: &str, stats: &CellStats, ctx: &CellCtx) {
    let buckets = Bucket::ALL
        .iter()
        .map(|b| format!("{}={}", b.as_str(), stats.tally.get(*b)))
        .collect::<Vec<_>>()
        .join("  ");
    println!("{label}: {buckets}");
    println!("{}", format_timing(stats, ctx));
}

fn print_progress_line(
    shot: u32,
    last_out: &ShotOutcome,
    stats: &CellStats,
    ctx: &CellCtx,
    consecutive_wedge: u32,
) {
    let buckets = Bucket::ALL
        .iter()
        .map(|b| format!("{}={}", b.as_str(), stats.tally.get(*b)))
        .collect::<Vec<_>>()
        .join("  ");
    println!(
        "  shot {shot:>5}: {:<10}  {buckets}  {}  (consec_wedge={consecutive_wedge})",
        last_out.bucket.as_str(),
        format_timing(stats, ctx),
    );
}

fn format_timing(stats: &CellStats, ctx: &CellCtx) -> String {
    if stats.offsets.is_empty() {
        return format!(
            "med={:>7} σ={:>5}  thresh=±{:.3}µs  --",
            "n/a", "n/a", ctx.thresh_us
        );
    }
    let mut sorted = stats.offsets.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = sorted[sorted.len() / 2];
    let sigma = if sorted.len() > 1 {
        let mean = stats.offsets.iter().sum::<f64>() / stats.offsets.len() as f64;
        let var = stats
            .offsets
            .iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>()
            / (stats.offsets.len() - 1) as f64;
        var.sqrt()
    } else {
        0.0
    };
    let med_us = med / ctx.ticks_per_us;
    let sigma_us = sigma / ctx.ticks_per_us;
    let verdict = if med_us.abs() < ctx.thresh_us {
        "PASS"
    } else {
        "FAIL"
    };
    format!(
        "med={med_us:+8.3} σ={sigma_us:6.3} thresh=±{:.3}µs {verdict:>4}",
        ctx.thresh_us
    )
}

fn print_capture(label: &str, req: &[u8], reply: &[u8]) {
    println!(
        "\n  [{label}] request ({}B):  {}",
        req.len(),
        hex_spaced(req)
    );
    println!(
        "  [{label}] reply   ({}B):  {}",
        reply.len(),
        hex_spaced(reply)
    );
}

fn hex_spaced(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn print_counter_delta(label: &str, delta: &[(&'static str, u32)]) {
    if delta.is_empty() {
        println!("{label}: none");
        return;
    }
    println!("{label}:");
    for (name, val) in delta {
        println!("  {name:25} += {val}");
    }
}

fn format_counter_delta(delta: &[(&'static str, u32)]) -> String {
    delta
        .iter()
        .map(|(n, v)| format!("{n}+={v}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn print_wedge_banner(context: &str) {
    println!("\n*** WEDGE DETECTED {context} ***");
    println!("   Counter read failed — chip not replying to plain Read.");
}

// ---------------------------------------------------------------------------
// Parsing helpers
// ---------------------------------------------------------------------------

fn parse_int_list(s: &str) -> Result<Vec<u32>> {
    s.split(',')
        .map(|x| {
            x.trim()
                .parse::<u32>()
                .map_err(|e| anyhow!("bad number {x:?}: {e}"))
        })
        .collect()
}

fn parse_position_list(s: &str) -> Result<Vec<Position>> {
    s.split(',')
        .map(|x| match x.trim() {
            "only" => Ok(Position::Only),
            "first" => Ok(Position::First),
            "middle" => Ok(Position::Middle),
            "last" => Ok(Position::Last),
            other => bail!("unknown position {other:?}"),
        })
        .collect()
}

fn clone_args(args: &Args) -> Args {
    Args {
        port: args.port.clone(),
        baud: args.baud,
        position: args.position,
        inj_len: args.inj_len,
        dut_len: args.dut_len,
        shots: args.shots,
        shot_wait_ms: args.shot_wait_ms,
        inter_shot_ms: args.inter_shot_ms,
        scope_delay_s: args.scope_delay_s,
        continuous: args.continuous,
        max_shots: args.max_shots,
        wedge_threshold: args.wedge_threshold,
        matrix: args.matrix,
        matrix_bauds: args.matrix_bauds.clone(),
        matrix_dut_lens: args.matrix_dut_lens.clone(),
        matrix_positions: args.matrix_positions.clone(),
        matrix_n: args.matrix_n,
        matrix_repeat: args.matrix_repeat,
        matrix_health_every: args.matrix_health_every,
    }
}

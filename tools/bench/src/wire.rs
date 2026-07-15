//! The bench's wire transport: the osc-adapter as instrument. Raw
//! break-framed TX, arbitrary-rate host UART, break trains, and
//! hardware-timestamped edge capture decoded into [`BStamp`]s -- the
//! adapter's 0x6x record family through osc-client's blocking facade.
//! Engine-shaped traffic (ENUM walks, rails) rides [`Wire::client`].

use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::wire::{Level, WireEdge};

use crate::BOOT_BAUD;
use crate::edges::{BStamp, stamps_from_edges};

pub struct Wire {
    client: Client<NusbPipe>,
    baud: u32,
    ticks_per_us: u32,
    stall: StallInjector,
}

/// Capture-drain poll cadence. The client's anchor chain keeps a late
/// batch's timeline exact, but absolute placement across a quiet stretch
/// needs some drain (even empty) within a u16 tick wrap (~3.6 ms at
/// 18 MHz) of it -- the settle window is spent polling, never sleeping
/// through.
const DRAIN_POLL: Duration = Duration::from_millis(1);

impl Wire {
    /// Claim the adapter (VID/PID autodetect) and put the bus at `baud`.
    pub fn connect(baud: u32) -> Result<Self> {
        let mut client =
            Client::connect(NusbPipe::open()?).context("osc-adapter HELLO handshake")?;
        let ticks_per_us = client.info().ticks_per_us;
        client.wire_baud(BOOT_BAUD)?;
        let mut wire = Self {
            client,
            baud: BOOT_BAUD,
            ticks_per_us,
            stall: StallInjector::from_env(),
        };
        if baud != BOOT_BAUD {
            wire.set_baud(baud)?;
        }
        wire.reset()?;
        Ok(wire)
    }

    /// The sessioned client underneath, for engine-shaped traffic.
    pub fn client(&mut self) -> &mut Client<NusbPipe> {
        &mut self.client
    }

    /// Capture ticks per us (the adapter's engine tick domain).
    pub fn hz_per_us(&self) -> u32 {
        self.ticks_per_us
    }

    pub fn current_baud(&self) -> u32 {
        self.baud
    }

    /// Capture ticks per wire bit at the current baud.
    pub fn bit_ticks(&self) -> u32 {
        (self.ticks_per_us as u64 * 1_000_000 / self.baud as u64) as u32
    }

    /// Host UART rate, raw bps -- off-catalog divisors allowed (the
    /// clock-tracker's detune injector rides this). Clears capture state:
    /// stamps decoded across a rate change would lie.
    pub fn set_baud(&mut self, bps: u32) -> Result<()> {
        self.client.wire_baud(bps)?;
        self.baud = bps;
        self.reset()
    }

    /// Drop undrained captures + the sticky overflow flag.
    pub fn reset(&mut self) -> Result<()> {
        self.client.reset_capture()?;
        Ok(())
    }

    /// One law break + `bytes`, raw (malformed frames allowed).
    pub fn send_frame(&mut self, bytes: &[u8]) -> Result<()> {
        self.client.wire_send(bytes)?;
        Ok(())
    }

    /// Break-framed frames back-to-back at bus pace.
    pub fn burst(&mut self, frames: &[Vec<u8>]) -> Result<()> {
        let refs: Vec<&[u8]> = frames.iter().map(|f| f.as_slice()).collect();
        self.client.wire_burst(&refs)?;
        Ok(())
    }

    /// Hold the wire dominant-low for `us` (>= 300 us on a powered fleet
    /// IS a rescue declaration, protocol sec 9.1).
    pub fn pulse_low(&mut self, us: u32) -> Result<()> {
        self.client
            .wire_pulse_low(us.try_into().context("pulse width caps at 65535 us")?)?;
        Ok(())
    }

    /// `announce` frame + `breaks` bare law breaks `gap_us` apart on the
    /// adapter's crystal grid -- the gap deliberately decoupled from the
    /// announce payload (protocol sec 9.3 trains through the engine always
    /// pace what they announce; the lying train is the trim suite's
    /// injector).
    pub fn train(&mut self, announce: &[u8], gap_us: u32, breaks: u32) -> Result<()> {
        self.client.wire_train(
            announce,
            gap_us.try_into().context("train gap caps at 65535 us")?,
            breaks
                .try_into()
                .context("train length caps at 255 breaks")?,
        )?;
        Ok(())
    }

    /// Drain whatever capture currently holds and decode it. One-shot:
    /// callers pacing a longer window use [`Wire::collect_stamps`].
    pub fn drain_stamps(&mut self) -> Result<Vec<BStamp>> {
        let mut edges = self.drain_edges_dry()?;
        // The two polarity rings drain at slightly different instants, so
        // an edge pair straddling that window lands across drain batches
        // out of order -- time-order the accumulated view before decoding
        // (the decoder's level timeline binary-searches it).
        edges.sort_by_key(|e| e.tick);
        Ok(stamps_from_edges(&edges, self.bit_ticks()))
    }

    /// Poll-drain the capture across `settle_ms`, then decode. The window
    /// is spent draining at [`DRAIN_POLL`] cadence so quiet stretches stay
    /// spanned by drains and absolute placement holds (see [`DRAIN_POLL`]).
    pub fn collect_stamps(&mut self, settle_ms: u64) -> Result<Vec<BStamp>> {
        let deadline = Instant::now() + Duration::from_millis(settle_ms);
        let mut edges = Vec::new();
        loop {
            edges.extend(self.drain_edges_dry()?);
            if Instant::now() >= deadline {
                break;
            }
            std::thread::sleep(DRAIN_POLL);
        }
        edges.extend(self.drain_edges_dry()?);
        // Cross-batch time order (see drain_stamps).
        edges.sort_by_key(|e| e.tick);
        Ok(stamps_from_edges(&edges, self.bit_ticks()))
    }

    /// Drain until a poll answers empty. Overflow is a hard failure: the
    /// capture lost edges, so any decode would lie -- state is reset and
    /// the caller's measurement dies loudly.
    fn drain_edges_dry(&mut self) -> Result<Vec<WireEdge>> {
        let mut all = Vec::new();
        loop {
            self.stall.maybe_stall();
            let drain = self.client.drain_edges()?;
            if std::env::var_os("BENCH_DUMP_EDGES").is_some() {
                eprint!("[drain now={} n={}]", drain.now, drain.edges.len());
                for e in &drain.edges {
                    eprint!(
                        " {}{}",
                        e.tick,
                        if e.level == Level::Low { "v" } else { "^" }
                    );
                }
                eprintln!();
            }
            if drain.overflow {
                let _ = self.client.reset_capture();
                bail!("edge capture overflow: undrained ring lapped (capture reset)");
            }
            if drain.edges.is_empty() {
                return Ok(all);
            }
            all.extend(drain.edges);
        }
    }
}

/// Host-stall injector: `BENCH_STALL_EVERY=N` + `BENCH_STALL_MS=M` sleeps
/// M ms before every Nth drain. Multi-ms scheduler stalls in the drain
/// path are what age captures past the u16 tick wrap -- injecting them on
/// demand turns a once-per-thousand-cycles OS event into a deterministic
/// reproducer for the unwrap's stall immunity.
struct StallInjector {
    every: u32,
    ms: u64,
    count: u32,
}

impl StallInjector {
    fn from_env() -> Self {
        fn read<T: std::str::FromStr>(k: &str) -> Option<T> {
            std::env::var(k).ok().and_then(|v| v.parse().ok())
        }
        Self {
            every: read("BENCH_STALL_EVERY").unwrap_or(0),
            ms: read("BENCH_STALL_MS").unwrap_or(8),
            count: 0,
        }
    }

    fn maybe_stall(&mut self) {
        if self.every == 0 {
            return;
        }
        self.count += 1;
        if self.count.is_multiple_of(self.every) {
            std::thread::sleep(Duration::from_millis(self.ms));
        }
    }
}

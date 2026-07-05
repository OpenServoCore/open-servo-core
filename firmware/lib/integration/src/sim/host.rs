//! `Host` — DXL master device. Encodes instruction frames via
//! `dxl_protocol::encode::encode_instruction` and queues per-byte
//! transmissions onto its [`UartTx`]; decodes incoming Status replies via
//! [`UartRx`]. All edge buffering, byte scheduling, and RX/TX logging live one
//! layer down in the UART halves — the Host is a thin device-layer over them.

use std::any::Any;

use dxl_protocol::{
    SoftwareCrcUmts,
    encode::encode_instruction,
    types::{BulkReadEntry, Id, Instruction},
};
use osc_core::BaudRate;

/// Encode one host->servo instruction frame into an owned byte buffer via the
/// flat emitter. `params` is the instruction's ordered param runs (the
/// stuffing window straddles run boundaries).
fn build_frame(id: Id, instruction: u8, params: &[&[u8]]) -> Vec<u8> {
    // Header + fully-stuffed params (worst case < 2×raw) + CRC, with margin —
    // the over-cap resilience tests deliberately push oversized bodies.
    let raw: usize = params.iter().map(|p| p.len()).sum();
    let mut buf = vec![0u8; raw * 2 + 16];
    let n = encode_instruction::<SoftwareCrcUmts>(&mut buf, id, instruction, params)
        .expect("instruction frame encodes");
    buf.truncate(n);
    buf
}

/// Flatten Bulk Read entries into their `[id, addr_le, len_le]*` wire body.
fn flatten_bulk(entries: &[BulkReadEntry]) -> Vec<u8> {
    let mut flat = Vec::with_capacity(entries.len() * 5);
    for e in entries {
        flat.push(e.id.as_byte());
        flat.extend_from_slice(&e.address.to_le_bytes());
        flat.extend_from_slice(&e.length.to_le_bytes());
    }
    flat
}

use crate::sim::defaults::{DEFAULT_BAUD, default_host_clock};
use crate::sim::uart::{RxLogEntry, RxLogKind, TxLogEntry, UartRx, UartTx, byte_time_ns};
use crate::sim::{Clock, DeviceId, Effect, EventSource, SimTime};

/// FTDI/CH340 inter-byte LATENCY_TIMER — once a byte lands in the host
/// UART, the driver schedules the next read after this much silence.
/// Drives the dynamic-deadline arithmetic in [`Host::wait_for_reply`]:
/// each new RX byte pushes the wait deadline forward by this amount.
pub const HOST_INTER_BYTE_TIMEOUT: SimTime = SimTime::from_us(2_000);

/// FTDI/CH340 first-byte budget — how long the host waits for the first
/// byte of a reply before declaring the target silent. Tuned long enough
/// to absorb a worst-case packet's TX-out time + RDT + the slowest
/// servo's TX-back at 9600 baud, short enough that broadcast-write
/// "no-reply" cases close quickly.
pub const HOST_FIRST_BYTE_TIMEOUT: SimTime = SimTime::from_ms(50);

/// Absolute backstop on a single `wait_for_reply` window — bounds the
/// total time the host will spend waiting regardless of incoming-byte
/// activity. Hit only when a buggy peer keeps the wire busy indefinitely;
/// long enough that legitimate 12-servo broadcast chains at 9600 baud
/// (~200 ms total) settle inside it.
pub const HOST_ABSOLUTE_CAP: SimTime = SimTime::from_ms(1_000);

/// Open wait-for-reply predicate. The host's `next_event_time` reports a
/// dynamic deadline derived from `last_rx_byte_at` + `inter_byte_timeout`
/// (or `opened_at + first_byte_timeout` if nothing has arrived). Each
/// new RX byte pushes the deadline forward; settle keeps draining until
/// the wire goes silent and the deadline holds. `advance` self-clears
/// when `t >= deadline`.
#[derive(Copy, Clone, Debug)]
struct Wait {
    opened_at: SimTime,
    cursor_at_open: usize,
    first_byte_timeout: SimTime,
    inter_byte_timeout: SimTime,
    absolute_cap: SimTime,
}

pub struct Host {
    id: DeviceId,
    clock: Clock,
    baud: BaudRate,
    uart_tx: UartTx,
    uart_rx: UartRx,
    /// Tracks the sim clock so `send_*` can pace TX bytes without each caller
    /// threading `now` through. Updated on every `EventSource::advance` and
    /// on every test-facing accessor via `set_now`.
    now: SimTime,
    wait: Option<Wait>,
}

impl Host {
    pub fn new(id: DeviceId) -> Self {
        Self {
            id,
            clock: default_host_clock(),
            baud: DEFAULT_BAUD,
            uart_tx: UartTx::new(DEFAULT_BAUD),
            uart_rx: UartRx::new(DEFAULT_BAUD),
            now: SimTime::ZERO,
            wait: None,
        }
    }

    pub fn with_clock(mut self, clock: Clock) -> Self {
        self.clock = clock;
        self
    }

    pub fn with_baud(mut self, baud: BaudRate) -> Self {
        self.set_baud(baud);
        self
    }

    /// Mid-sim baud retune. Wedge tests use this to simulate a host that
    /// switches baud rates without coordinating with the servo — the
    /// fresh UART state drops any half-shifted state from the prior
    /// baud, so subsequent `send_*` writes land at the new bit time.
    pub fn set_baud(&mut self, baud: BaudRate) {
        self.baud = baud;
        self.uart_tx = UartTx::new(baud);
        self.uart_rx = UartRx::new(baud);
    }

    pub fn clock(&self) -> Clock {
        self.clock
    }

    pub fn baud(&self) -> BaudRate {
        self.baud
    }

    pub fn rx_log(&self) -> &[RxLogEntry] {
        self.uart_rx.rx_log()
    }

    pub fn tx_log(&self) -> &[TxLogEntry] {
        self.uart_tx.tx_log()
    }

    pub fn rx_bytes(&self) -> Vec<u8> {
        self.uart_rx.rx_bytes()
    }

    /// `packet_end` per DXL spec — the last queued TX byte's start
    /// timestamp plus one `byte_time` (stop-bit clear). The anchor every
    /// reply-timing assertion compares against. `None` when nothing has
    /// been queued onto `UartTx`.
    pub fn packet_end_ns(&self) -> Option<u64> {
        self.uart_tx
            .tx_log()
            .last()
            .map(|e| e.at.as_ns() + byte_time_ns(self.baud))
    }

    /// Start-bit timestamps of every received byte, in arrival order.
    /// `rx_log` stamps land at byte completion (10·bp past the start
    /// bit); the subtraction recovers the start-bit instant so callers
    /// compare apples-to-apples with [`packet_end_ns`]. Idle gaps are
    /// skipped.
    pub fn rx_byte_starts_ns(&self) -> Vec<u64> {
        let bt = byte_time_ns(self.baud);
        self.uart_rx
            .rx_log()
            .iter()
            .filter_map(|e| match e.kind {
                RxLogKind::Byte(_) => Some(e.at.as_ns() - bt),
                RxLogKind::IdleGap => None,
            })
            .collect()
    }

    pub fn clear_logs(&mut self) {
        self.uart_rx.clear_log();
        self.uart_tx.clear_log();
    }

    /// Encode a Ping for `target` and queue the frame onto [`UartTx`] at
    /// baud-stride pacing starting at the host's current sim time.
    pub fn send_ping(&mut self, target: Id) {
        let buf = build_frame(target, Instruction::Ping.as_u8(), &[]);
        self.queue_frame(&buf);
    }

    /// Encode a Read for `target` at `addr` for `length` bytes and queue the
    /// frame onto [`UartTx`] at baud-stride pacing.
    pub fn send_read(&mut self, target: Id, addr: u16, length: u16) {
        let buf = build_frame(
            target,
            Instruction::Read.as_u8(),
            &[&addr.to_le_bytes(), &length.to_le_bytes()],
        );
        self.queue_frame(&buf);
    }

    /// Encode a Write of `data` to `target` at `addr` and queue the frame.
    pub fn send_write(&mut self, target: Id, addr: u16, data: &[u8]) {
        let buf = build_frame(
            target,
            Instruction::Write.as_u8(),
            &[&addr.to_le_bytes(), data],
        );
        self.queue_frame(&buf);
    }

    /// Encode a RegWrite of `data` to `target` at `addr` and queue the frame.
    /// The receiver stages the bytes; an Action frame commits them atomically.
    pub fn send_reg_write(&mut self, target: Id, addr: u16, data: &[u8]) {
        let buf = build_frame(
            target,
            Instruction::RegWrite.as_u8(),
            &[&addr.to_le_bytes(), data],
        );
        self.queue_frame(&buf);
    }

    /// Encode an Action for `target` and queue the frame, committing any
    /// previously staged RegWrite chain on the receiver.
    pub fn send_action(&mut self, target: Id) {
        let buf = build_frame(target, Instruction::Action.as_u8(), &[]);
        self.queue_frame(&buf);
    }

    /// Encode a Sync Read broadcast (one `addr`/`length` applied to every id
    /// in `ids`) and queue the frame.
    pub fn send_sync_read(&mut self, addr: u16, length: u16, ids: &[u8]) {
        let buf = build_frame(
            Id::BROADCAST,
            Instruction::SyncRead.as_u8(),
            &[&addr.to_le_bytes(), &length.to_le_bytes(), ids],
        );
        self.queue_frame(&buf);
    }

    /// Encode a Sync Write broadcast (one `addr`/`length` applied to every
    /// `[id, data...]` chunk in `body`) and queue the frame.
    pub fn send_sync_write(&mut self, addr: u16, length: u16, body: &[u8]) {
        let buf = build_frame(
            Id::BROADCAST,
            Instruction::SyncWrite.as_u8(),
            &[&addr.to_le_bytes(), &length.to_le_bytes(), body],
        );
        self.queue_frame(&buf);
    }

    /// Encode a Bulk Read broadcast (per-tuple `(id, addr, length)`) and
    /// queue the frame.
    pub fn send_bulk_read(&mut self, entries: &[BulkReadEntry]) {
        let buf = build_frame(
            Id::BROADCAST,
            Instruction::BulkRead.as_u8(),
            &[&flatten_bulk(entries)],
        );
        self.queue_frame(&buf);
    }

    /// Encode a Bulk Write broadcast (`body` is a flat
    /// `[id, addr_le, length_le, data...]+` per servo) and queue the frame.
    pub fn send_bulk_write(&mut self, body: &[u8]) {
        let buf = build_frame(Id::BROADCAST, Instruction::BulkWrite.as_u8(), &[body]);
        self.queue_frame(&buf);
    }

    /// Encode a Fast Sync Read broadcast (shared `addr` / `length`, list of
    /// `ids` in chain order) and queue the frame.
    pub fn send_fast_sync_read(&mut self, addr: u16, length: u16, ids: &[u8]) {
        let buf = build_frame(
            Id::BROADCAST,
            Instruction::FastSyncRead.as_u8(),
            &[&addr.to_le_bytes(), &length.to_le_bytes(), ids],
        );
        self.queue_frame(&buf);
    }

    /// Encode a Fast Bulk Read broadcast (per-entry `(id, addr, length)`) and
    /// queue the frame.
    pub fn send_fast_bulk_read(&mut self, entries: &[BulkReadEntry]) {
        let buf = build_frame(
            Id::BROADCAST,
            Instruction::FastBulkRead.as_u8(),
            &[&flatten_bulk(entries)],
        );
        self.queue_frame(&buf);
    }

    /// Encode a Reboot for `target` and queue the frame.
    pub fn send_reboot(&mut self, target: Id) {
        let buf = build_frame(target, Instruction::Reboot.as_u8(), &[]);
        self.queue_frame(&buf);
    }

    /// Queue an arbitrary byte sequence onto the wire at the host's current
    /// baud. Used by the resilience tests to inject malformed, truncated,
    /// or cross-baud byte sequences that the protocol-aware `send_*`
    /// builders can't produce. No framing, no CRC fixup — bytes go on
    /// the wire verbatim.
    pub fn send_raw(&mut self, bytes: &[u8]) {
        self.queue_frame(bytes);
    }

    /// Encode an arbitrary instruction byte (with optional params) for `target`.
    /// Use for non-standard / unsupported instruction tests; the servo
    /// surfaces the byte as `Instruction::Ext(byte)`.
    pub fn send_ext(&mut self, target: Id, instr: u8, params: &[u8]) {
        let buf = build_frame(target, instr, &[params]);
        self.queue_frame(&buf);
    }

    /// Open a reply-wait predicate at `self.now` with the default
    /// FTDI-shaped timeouts. The host's `next_event_time` then reports a
    /// dynamic deadline that pushes forward with each arriving RX byte;
    /// `Sim::settle` keeps draining the wire until the deadline holds.
    /// Multiple calls overwrite any prior predicate.
    pub fn wait_for_reply(&mut self) {
        self.wait_for_reply_within(HOST_FIRST_BYTE_TIMEOUT, HOST_INTER_BYTE_TIMEOUT);
    }

    /// Open a reply-wait predicate at `self.now` with explicit timeouts.
    /// Use when a test specifically pins a non-default budget — e.g. very
    /// long RDT sweeps, or chains long enough that the default
    /// inter-byte gap is too tight.
    pub fn wait_for_reply_within(&mut self, first_byte: SimTime, inter_byte: SimTime) {
        self.wait = Some(Wait {
            opened_at: self.now,
            cursor_at_open: self.uart_rx.rx_log().len(),
            first_byte_timeout: first_byte,
            inter_byte_timeout: inter_byte,
            absolute_cap: self.now + HOST_ABSOLUTE_CAP,
        });
    }

    fn queue_frame(&mut self, buf: &[u8]) {
        for (i, byte) in buf.iter().enumerate() {
            let offset = self.uart_tx.byte_offset_ns(i as u64);
            self.uart_tx.queue_byte(*byte, self.now + offset);
        }
    }

    fn wait_deadline(&self, w: &Wait) -> SimTime {
        let dyn_deadline = match self.last_rx_byte_after(w.cursor_at_open) {
            Some(last_at) => last_at + w.inter_byte_timeout,
            None => w.opened_at + w.first_byte_timeout,
        };
        if dyn_deadline < w.absolute_cap {
            dyn_deadline
        } else {
            w.absolute_cap
        }
    }

    fn last_rx_byte_after(&self, cursor: usize) -> Option<SimTime> {
        self.uart_rx.rx_log()[cursor..]
            .iter()
            .rev()
            .find(|e| matches!(e.kind, RxLogKind::Byte(_)))
            .map(|e| e.at)
    }
}

impl EventSource for Host {
    fn next_event_time(&self) -> Option<SimTime> {
        let wait_wake = self.wait.as_ref().map(|w| self.wait_deadline(w));
        [
            self.uart_tx.next_wake(),
            self.uart_rx.next_wake(),
            wait_wake,
        ]
        .into_iter()
        .flatten()
        .min()
    }

    fn advance(&mut self, t: SimTime) -> Vec<Effect> {
        self.now = t;
        // RX log accumulates inside UartRx; the Host has no protocol layer
        // yet, so the returned RxEffects are ignored here.
        let _ = self.uart_rx.advance(t);
        let effects: Vec<_> = self
            .uart_tx
            .advance(t, &[])
            .into_iter()
            .map(|(at_ns, rising)| Effect::WireEdge {
                at: SimTime::from_ns(at_ns),
                rising,
            })
            .collect();
        if let Some(w) = self.wait
            && t >= self.wait_deadline(&w)
        {
            self.wait = None;
        }
        if !effects.is_empty() {
            log::trace!(
                "host[{:?}]: advance t={:?} emit {} wire edges",
                self.id,
                t,
                effects.len()
            );
        }
        effects
    }

    fn receive_edge(&mut self, at: SimTime, rising: bool) {
        log::trace!(
            "host[{:?}]: receive_edge at={:?} rising={}",
            self.id,
            at,
            rising
        );
        self.uart_rx.receive_edge(at, rising);
    }

    fn set_now(&mut self, t: SimTime) {
        if t > self.now {
            self.now = t;
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::Sim;
    use crate::sim::defaults::DEFAULT_BAUD;
    use crate::sim::uart::RxLogKind;

    fn expected_ping_bytes(target: Id) -> Vec<u8> {
        build_frame(target, Instruction::Ping.as_u8(), &[])
    }

    #[test]
    fn tx_log_records_encoded_frame_in_order() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        sim.with_host(host, |h| {
            h.send_ping(Id::new(0x05));
            h.wait_for_reply();
        });

        let tx_bytes: Vec<u8> = sim.host(host).tx_log().iter().map(|e| e.byte).collect();
        assert_eq!(tx_bytes, expected_ping_bytes(Id::new(0x05)));
    }

    #[test]
    fn rx_log_captures_remote_tx_with_idle_gap() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let receiver = sim.add_device(Host::new);

        sim.with_host(host, |h| {
            h.send_ping(Id::new(0x05));
            h.wait_for_reply();
        });

        let rx = sim.host(receiver);
        assert_eq!(rx.rx_bytes(), expected_ping_bytes(Id::new(0x05)));
        assert!(
            rx.rx_log()
                .iter()
                .any(|e| matches!(e.kind, RxLogKind::IdleGap)),
            "expected an IdleGap entry after frame end, log = {:?}",
            rx.rx_log()
        );
    }

    #[test]
    fn tx_byte_timestamps_align_with_baud_stride() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        sim.with_host(host, |h| {
            h.send_ping(Id::new(0x01));
            h.wait_for_reply();
        });

        // Bresenham byte stride: `i × 10 × 10⁹ / baud_hz` keeps cumulative
        // phase drift bounded by ±1 ns regardless of `i`. At 1M baud
        // (DEFAULT_BAUD) the divisor is exact so every byte lands on a
        // 10_000 ns boundary — the test result is the same as the prior
        // `10 × bit_period_ns(BAUD)` formulation.
        let baud_hz = DEFAULT_BAUD.as_hz() as u64;
        let tx = sim.host(host).tx_log().to_vec();
        for (i, e) in tx.iter().enumerate() {
            let expected = (i as u64 * 10 * 1_000_000_000) / baud_hz;
            assert_eq!(e.at.as_ns(), expected, "byte {i}");
        }
    }

    #[test]
    fn clear_logs_drops_history_without_resetting_queues() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        sim.with_host(host, |h| {
            h.send_ping(Id::new(0x01));
            h.wait_for_reply();
        });
        assert!(!sim.host(host).tx_log().is_empty());

        sim.host_mut(host).clear_logs();
        assert!(sim.host(host).tx_log().is_empty());
        assert!(sim.host(host).rx_log().is_empty());
    }

    #[test]
    fn wait_for_reply_closes_on_first_byte_timeout_when_silent() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let _peer = sim.add_device(Host::new);

        sim.with_host(host, |h| {
            // No send → no TX to drive the peer; predicate closes on
            // first-byte timeout alone.
            h.wait_for_reply();
        });

        assert!(sim.host(host).rx_bytes().is_empty());
        assert_eq!(sim.now(), HOST_FIRST_BYTE_TIMEOUT);
    }
}

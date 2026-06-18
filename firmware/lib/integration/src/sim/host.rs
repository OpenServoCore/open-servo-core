//! `Host` — DXL master device. Encodes instruction frames via
//! `dxl_protocol::InstructionEncoder` and queues per-byte transmissions onto
//! its [`UartTx`]; decodes incoming Status replies via [`UartRx`]. All edge
//! buffering, byte scheduling, and RX/TX logging live one layer down in the
//! UART halves — the Host is a thin device-layer over them.

use std::any::Any;

use dxl_protocol::{
    InstructionEncoder, SoftwareCrcUmts,
    types::{BulkReadEntry, Id},
};
use osc_core::BaudRate;

use crate::sim::defaults::{DEFAULT_BAUD, default_host_clock};
use crate::sim::uart::{RxLogEntry, TxLogEntry, UartRx, UartTx};
use crate::sim::{Clock, DeviceId, Effect, EventSource, SimTime};

pub struct Host {
    id: DeviceId,
    clock: Clock,
    baud: BaudRate,
    uart_tx: UartTx,
    uart_rx: UartRx,
    /// Tracks the sim clock so `send_*` can pace TX bytes without each caller
    /// threading `now` through. Updated on every `EventSource::advance`.
    now: SimTime,
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
        }
    }

    pub fn with_clock(mut self, clock: Clock) -> Self {
        self.clock = clock;
        self
    }

    pub fn with_baud(mut self, baud: BaudRate) -> Self {
        self.baud = baud;
        self.uart_tx = UartTx::new(baud);
        self.uart_rx = UartRx::new(baud);
        self
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

    pub fn clear_logs(&mut self) {
        self.uart_rx.clear_log();
        self.uart_tx.clear_log();
    }

    /// Encode a Ping for `target` and queue the frame onto [`UartTx`] at
    /// baud-stride pacing starting at the host's current sim time. The first
    /// byte goes out on the next [`Sim::advance`](crate::sim::Sim::advance)
    /// that reaches it.
    pub fn send_ping(&mut self, target: Id) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .ping(target)
            .expect("ping frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a Read for `target` at `addr` for `length` bytes and queue the
    /// frame onto [`UartTx`] at baud-stride pacing.
    pub fn send_read(&mut self, target: Id, addr: u16, length: u16) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .read(target, addr, length)
            .expect("read frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a Write of `data` to `target` at `addr` and queue the frame.
    pub fn send_write(&mut self, target: Id, addr: u16, data: &[u8]) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .write(target, addr, data)
            .expect("write frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a RegWrite of `data` to `target` at `addr` and queue the frame.
    /// The receiver stages the bytes; an Action frame commits them atomically.
    pub fn send_reg_write(&mut self, target: Id, addr: u16, data: &[u8]) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .reg_write(target, addr, data)
            .expect("reg_write frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode an Action for `target` and queue the frame, committing any
    /// previously staged RegWrite chain on the receiver.
    pub fn send_action(&mut self, target: Id) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .action(target)
            .expect("action frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a Sync Read broadcast (one `addr`/`length` applied to every id
    /// in `ids`) and queue the frame.
    pub fn send_sync_read(&mut self, addr: u16, length: u16, ids: &[u8]) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .sync_read(addr, length, ids)
            .expect("sync_read frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a Sync Write broadcast (one `addr`/`length` applied to every
    /// `[id, data...]` chunk in `body`) and queue the frame.
    pub fn send_sync_write(&mut self, addr: u16, length: u16, body: &[u8]) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .sync_write(addr, length, body)
            .expect("sync_write frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a Bulk Read broadcast (per-tuple `(id, addr, length)`) and
    /// queue the frame.
    pub fn send_bulk_read(&mut self, entries: &[BulkReadEntry]) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .bulk_read(entries)
            .expect("bulk_read frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a Bulk Write broadcast (`body` is a flat
    /// `[id, addr_le, length_le, data...]+` per servo) and queue the frame.
    pub fn send_bulk_write(&mut self, body: &[u8]) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .bulk_write(body)
            .expect("bulk_write frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode a Reboot for `target` and queue the frame.
    pub fn send_reboot(&mut self, target: Id) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .reboot(target)
            .expect("reboot frame encodes");
        self.queue_frame(&buf);
    }

    /// Encode an arbitrary instruction byte (with optional params) for `target`.
    /// Use for non-standard / unsupported instruction tests; the servo's
    /// streaming parser surfaces the byte as `Instruction::Ext(byte)`.
    pub fn send_ext(&mut self, target: Id, instr: u8, params: &[u8]) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .ext(target, instr, params)
            .expect("ext frame encodes");
        self.queue_frame(&buf);
    }

    fn queue_frame(&mut self, buf: &[u8]) {
        let stride = 10 * self.uart_tx.bit_period_ns();
        for (i, byte) in buf.iter().enumerate() {
            self.uart_tx.queue_byte(*byte, self.now + i as u64 * stride);
        }
    }
}

impl EventSource for Host {
    fn next_event_time(&self) -> Option<SimTime> {
        [self.uart_tx.next_wake(), self.uart_rx.next_wake()]
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
                source: self.id,
                at: SimTime::from_ns(at_ns),
                rising,
            })
            .collect();
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

    fn reset(&mut self) {
        self.uart_tx = UartTx::new(self.baud);
        self.uart_rx = UartRx::new(self.baud);
        self.now = SimTime::ZERO;
    }

    fn tick(&mut self, t: SimTime) {
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
    use dxl_protocol::InstructionEncoder;

    fn expected_ping_bytes(target: Id) -> Vec<u8> {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .ping(target)
            .unwrap();
        buf
    }

    #[test]
    fn tx_log_records_encoded_frame_in_order() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        sim.advance(SimTime::from_ms(5), |sim, _| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(Id::new(0x05));
        });

        let tx_bytes: Vec<u8> = sim
            .device::<Host>(host)
            .unwrap()
            .tx_log()
            .iter()
            .map(|e| e.byte)
            .collect();
        assert_eq!(tx_bytes, expected_ping_bytes(Id::new(0x05)));
    }

    #[test]
    fn rx_log_captures_remote_tx_with_idle_gap() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let receiver = sim.add_device(Host::new);

        sim.advance(SimTime::from_ms(5), |sim, _| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(Id::new(0x05));
        });

        let rx = sim.device::<Host>(receiver).unwrap();
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
        sim.advance(SimTime::from_ms(5), |sim, _| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(Id::new(0x01));
        });

        let stride = 10 * crate::sim::uart::bit_period_ns(DEFAULT_BAUD);
        let tx = sim.device::<Host>(host).unwrap().tx_log().to_vec();
        for (i, e) in tx.iter().enumerate() {
            assert_eq!(e.at.as_ns(), i as u64 * stride, "byte {i}");
        }
    }

    #[test]
    fn clear_logs_drops_history_without_resetting_queues() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        sim.advance(SimTime::from_ms(5), |sim, _| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(Id::new(0x01));
        });
        assert!(!sim.device::<Host>(host).unwrap().tx_log().is_empty());

        sim.device_mut::<Host>(host).unwrap().clear_logs();
        assert!(sim.device::<Host>(host).unwrap().tx_log().is_empty());
        assert!(sim.device::<Host>(host).unwrap().rx_log().is_empty());
    }

    #[test]
    fn sim_reset_zeroes_time_and_clears_host_logs() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let receiver = sim.add_device(Host::new);
        sim.advance(SimTime::from_ms(5), |sim, _| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(Id::new(0x01));
        });
        assert_ne!(sim.now(), SimTime::ZERO);
        assert!(!sim.device::<Host>(receiver).unwrap().rx_log().is_empty());

        sim.reset();
        assert_eq!(sim.now(), SimTime::ZERO);
        assert!(sim.device::<Host>(host).unwrap().tx_log().is_empty());
        assert!(sim.device::<Host>(receiver).unwrap().rx_log().is_empty());
    }
}

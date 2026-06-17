//! `Host` — DXL master device. Encodes instruction frames via
//! `dxl_protocol::InstructionEncoder` and queues per-byte transmissions onto
//! its [`UartTx`]; decodes incoming Status replies via [`UartRx`]. All edge
//! buffering, byte scheduling, and RX/TX logging live one layer down in the
//! UART halves — the Host is a thin device-layer over them.

use std::any::Any;

use dxl_protocol::{InstructionEncoder, SoftwareCrcUmts, types::Id};

use crate::sim::uart::{RxLogEntry, TxLogEntry, UartRx, UartTx};
use crate::sim::{Clock, DeviceId, Effect, EventSource, SimTime};

pub struct Host {
    id: DeviceId,
    clock: Clock,
    baud: u32,
    uart_tx: UartTx,
    uart_rx: UartRx,
}

impl Host {
    pub fn new(id: DeviceId, clock: Clock, baud: u32) -> Self {
        Self {
            id,
            clock,
            baud,
            uart_tx: UartTx::new(baud),
            uart_rx: UartRx::new(baud),
        }
    }

    pub fn clock(&self) -> Clock {
        self.clock
    }

    pub fn baud(&self) -> u32 {
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

    /// Encode a Ping for `target` and queue one TX byte per frame byte onto
    /// [`UartTx`] at `now + i * 10 * bit_period_ns`. The first byte goes out
    /// on the next [`Sim::advance`](crate::sim::Sim::advance) that reaches
    /// `now`.
    pub fn send_ping(&mut self, now: SimTime, target: Id) {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .ping(target)
            .expect("ping frame encodes");
        let stride = 10 * self.uart_tx.bit_period_ns();
        for (i, byte) in buf.iter().enumerate() {
            self.uart_tx.queue_byte(*byte, now + i as u64 * stride);
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
        // RX log accumulates inside UartRx; the Host has no protocol layer
        // yet, so the returned RxEffects are ignored here.
        let _ = self.uart_rx.advance(t);
        self.uart_tx
            .advance(t)
            .into_iter()
            .map(|(at_ns, rising)| Effect::WireEdge {
                source: self.id,
                at: SimTime::from_ns(at_ns),
                rising,
            })
            .collect()
    }

    fn receive_edge(&mut self, at: SimTime, rising: bool) {
        self.uart_rx.receive_edge(at, rising);
    }

    fn reset(&mut self) {
        self.uart_tx = UartTx::new(self.baud);
        self.uart_rx = UartRx::new(self.baud);
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
    use crate::sim::uart::RxLogKind;
    use dxl_protocol::InstructionEncoder;

    const HOST_CLOCK: Clock = Clock::new(48_000_000);
    const BAUD: u32 = 115_200;

    fn expected_ping_bytes(target: Id) -> Vec<u8> {
        let mut buf: Vec<u8> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut buf)
            .ping(target)
            .unwrap();
        buf
    }

    #[test]
    fn tx_log_records_encoded_frame_in_order() {
        let mut sim = Sim::new(BAUD);
        let host = sim.add_device(|id| Host::new(id, HOST_CLOCK, BAUD));
        sim.advance(SimTime::from_ms(5), |sim, now| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(now, Id::new(0x05));
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
        let mut sim = Sim::new(BAUD);
        let host = sim.add_device(|id| Host::new(id, HOST_CLOCK, BAUD));
        let receiver = sim.add_device(|id| Host::new(id, HOST_CLOCK, BAUD));

        sim.advance(SimTime::from_ms(5), |sim, now| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(now, Id::new(0x05));
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
        let mut sim = Sim::new(BAUD);
        let host = sim.add_device(|id| Host::new(id, HOST_CLOCK, BAUD));
        sim.advance(SimTime::from_ms(5), |sim, now| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(now, Id::new(0x01));
        });

        let stride = 10 * crate::sim::uart::bit_period_ns(BAUD);
        let tx = sim.device::<Host>(host).unwrap().tx_log().to_vec();
        for (i, e) in tx.iter().enumerate() {
            assert_eq!(e.at.as_ns(), i as u64 * stride, "byte {i}");
        }
    }

    #[test]
    fn clear_logs_drops_history_without_resetting_queues() {
        let mut sim = Sim::new(BAUD);
        let host = sim.add_device(|id| Host::new(id, HOST_CLOCK, BAUD));
        sim.advance(SimTime::from_ms(5), |sim, now| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(now, Id::new(0x01));
        });
        assert!(!sim.device::<Host>(host).unwrap().tx_log().is_empty());

        sim.device_mut::<Host>(host).unwrap().clear_logs();
        assert!(sim.device::<Host>(host).unwrap().tx_log().is_empty());
        assert!(sim.device::<Host>(host).unwrap().rx_log().is_empty());
    }

    #[test]
    fn sim_reset_zeroes_time_and_clears_host_logs() {
        let mut sim = Sim::new(BAUD);
        let host = sim.add_device(|id| Host::new(id, HOST_CLOCK, BAUD));
        let receiver = sim.add_device(|id| Host::new(id, HOST_CLOCK, BAUD));
        sim.advance(SimTime::from_ms(5), |sim, now| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(now, Id::new(0x01));
        });
        assert_ne!(sim.now(), SimTime::ZERO);
        assert!(!sim.device::<Host>(receiver).unwrap().rx_log().is_empty());

        sim.reset();
        assert_eq!(sim.now(), SimTime::ZERO);
        assert!(sim.device::<Host>(host).unwrap().tx_log().is_empty());
        assert!(sim.device::<Host>(receiver).unwrap().rx_log().is_empty());
    }
}

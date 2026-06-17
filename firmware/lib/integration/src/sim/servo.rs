//! `Servo` — DXL servo device under test. Composes `Dxl` (services-layer
//! dispatcher state) + `DxlUart<TestProviders, 64, 128, 140>` (the codec /
//! classifier / scheduler driver) + `Shared` (control table) + the 7 spy
//! state companions from `crate::mocks`. Drives inbound wire edges through
//! the codec's trait surface (byte ring + edge ring + IDLE callback),
//! invokes `Dxl::poll` to produce a Status reply, and re-encodes the reply's
//! TX bytes onto outbound wire edges via `UartTx`.

use std::any::Any;

use dxl_protocol::SoftwareCrcUmts;
use dxl_protocol::types::Id;
use osc_core::services::dxl::Dxl;
use osc_core::traits::{DxlBus, DxlDispatcher};
use osc_core::{BaudRate, Shared};
use osc_drivers::dxl::uart::DxlUart;
use osc_drivers::dxl::uart::clock::Clock as DxlClock;
use osc_drivers::dxl::uart::codec::Codec;
use osc_drivers::dxl::uart::fast_last::FastLast;
use osc_drivers::mocks::{EdgeDmaOp, FastLastSchedulerOp, ScheduleOp, TestProviders, TxBusOp};
use osc_drivers::traits::dxl::{DmaFlags, Providers};

use crate::mocks::{
    ClockTrimState, EdgeDmaState, FastLastSchedulerState, RxDmaState, TxBusState, TxSchedulerState,
    UsartBaudState, mock_clock_trim, mock_edge_dma, mock_fast_last_scheduler, mock_rx_dma,
    mock_tx_bus, mock_tx_scheduler, mock_usart_baud,
};
use crate::sim::uart::{UartRx, UartTx, bit_period_ns};
use crate::sim::{Clock, DeviceId, Effect, EventSource, SimTime};

const RX_BUF_LEN: usize = 64;
const EDGE_BUF_LEN: usize = 128;
const TX_BUF_LEN: usize = 140;

const DEFAULT_DXL_ID: Id = Id::new(1);
const DEFAULT_RDT_US: u32 = 250;

type ServoUart = DxlUart<TestProviders, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

pub struct Servo {
    id: DeviceId,
    clock: Clock,
    baud: BaudRate,
    dxl_id: Id,
    rdt_us: u32,

    dxl: Dxl,
    shared: Shared,
    uart: ServoUart,

    tx_bus_state: TxBusState,
    clock_trim_state: ClockTrimState,
    usart_baud_state: UsartBaudState,
    edge_dma_state: EdgeDmaState,
    rx_dma_state: RxDmaState,
    tx_scheduler_state: TxSchedulerState,
    fast_last_scheduler_state: FastLastSchedulerState,

    uart_tx: UartTx,
    uart_rx: UartRx,
    rx_seq: u32,
    edge_seq: u32,
}

impl Servo {
    pub fn new(id: DeviceId, clock: Clock, baud: BaudRate) -> Self {
        let mut s = Self {
            id,
            clock,
            baud,
            dxl_id: DEFAULT_DXL_ID,
            rdt_us: DEFAULT_RDT_US,

            dxl: Dxl::new(),
            shared: Shared::new(),
            uart: build_uart(baud, DEFAULT_DXL_ID, DEFAULT_RDT_US).uart,

            tx_bus_state: TxBusState::default(),
            clock_trim_state: ClockTrimState::default(),
            usart_baud_state: UsartBaudState::default(),
            edge_dma_state: EdgeDmaState::default(),
            rx_dma_state: RxDmaState::default(),
            tx_scheduler_state: TxSchedulerState::default(),
            fast_last_scheduler_state: FastLastSchedulerState::default(),

            uart_tx: UartTx::new(baud),
            uart_rx: UartRx::new(baud),
            rx_seq: 0,
            edge_seq: 0,
        };
        s.rebuild_uart();
        s
    }

    pub fn with_dxl_id(mut self, dxl_id: Id) -> Self {
        self.dxl_id = dxl_id;
        self.rebuild_uart();
        self
    }

    pub fn with_rdt_us(mut self, rdt_us: u32) -> Self {
        self.rdt_us = rdt_us;
        self.rebuild_uart();
        self
    }

    pub fn clock(&self) -> Clock {
        self.clock
    }

    pub fn baud(&self) -> BaudRate {
        self.baud
    }

    pub fn dxl_id(&self) -> Id {
        self.dxl_id
    }

    pub fn rdt_us(&self) -> u32 {
        self.rdt_us
    }

    pub fn shared(&self) -> &Shared {
        &self.shared
    }

    pub fn tx_bus_ops(&self) -> Vec<TxBusOp> {
        self.tx_bus_state.operations()
    }

    pub fn tx_scheduler_ops(&self) -> Vec<ScheduleOp> {
        self.tx_scheduler_state.operations()
    }

    pub fn fast_last_ops(&self) -> Vec<FastLastSchedulerOp> {
        self.fast_last_scheduler_state.operations()
    }

    pub fn edge_dma_ops(&self) -> Vec<EdgeDmaOp> {
        self.edge_dma_state.operations()
    }

    pub fn baud_ops(&self) -> Vec<BaudRate> {
        self.usart_baud_state.operations()
    }

    pub fn trim_ops(&self) -> Vec<i8> {
        self.clock_trim_state.operations()
    }

    fn rebuild_uart(&mut self) {
        let built = build_uart(self.baud, self.dxl_id, self.rdt_us);
        self.uart = built.uart;
        self.tx_bus_state = built.tx_bus_state;
        self.clock_trim_state = built.clock_trim_state;
        self.usart_baud_state = built.usart_baud_state;
        self.edge_dma_state = built.edge_dma_state;
        self.rx_dma_state = built.rx_dma_state;
        self.tx_scheduler_state = built.tx_scheduler_state;
        self.fast_last_scheduler_state = built.fast_last_scheduler_state;
        self.dxl = Dxl::new();
        self.shared = Shared::new();
        self.uart_tx = UartTx::new(self.baud);
        self.uart_rx = UartRx::new(self.baud);
        self.rx_seq = 0;
        self.edge_seq = 0;
    }

    fn chip_tick(&self, at: SimTime) -> u16 {
        (self.clock.to_local(at) % 65536) as u16
    }

    fn freq_hz(&self) -> u64 {
        self.clock.freq_hz() as u64
    }

    /// Inbound falling edge — write a TIM2 tick into the codec's edge ring
    /// and advance NDTR on the spy state. The classifier walk
    /// (`on_rx_edge_advance`) and the `Dxl::poll` that follows are gated on
    /// HT/TC crossings of the edge ring: production V006 has no per-edge
    /// IRQ, only DMA1_CH7 HT (half-mark) and TC (wrap). Intra-half edges
    /// accumulate silently; the IDLE backstop (`handle_idle`) catches short
    /// packets that never trip HT.
    fn handle_falling_edge(&mut self, at: SimTime) {
        let tick = self.chip_tick(at);
        let slot = (self.edge_seq as usize) % EDGE_BUF_LEN;
        // SAFETY: `edges_addr()` returns the address of `HwRing<u16,
        // EDGE_BUF_LEN>::data[0]`; the slot lies inside that buffer because
        // we mod by EDGE_BUF_LEN. No concurrent reader/writer in the host-
        // side test sim.
        unsafe {
            let ptr = self.uart.edges_addr() as *mut u16;
            *ptr.add(slot) = tick;
        }
        self.edge_seq = self.edge_seq.wrapping_add(1);
        let pos = (self.edge_seq as usize) % EDGE_BUF_LEN;
        let remaining = (EDGE_BUF_LEN - pos) as u16;
        self.edge_dma_state.stage_remaining(remaining);

        let crossed_ht = pos == EDGE_BUF_LEN / 2;
        let crossed_tc = pos == 0;
        if crossed_ht || crossed_tc {
            self.edge_dma_state.stage_next_flags(DmaFlags {
                ht: crossed_ht,
                tc: crossed_tc,
            });
            self.uart.on_rx_edge_advance();
            self.poll_and_queue_tx(at);
        }
    }

    /// Decoded byte from the line model — write to the codec's RX byte ring
    /// and update NDTR. Does NOT call `Dxl::poll`: production polls on
    /// USART1-IDLE / DMA HT-TC, not per byte, and the dispatcher is
    /// reconstructed per poll — running it mid-packet would lose `inflight`
    /// between the Header and Crc events.
    fn handle_byte(&mut self, byte: u8, _at: SimTime) {
        let slot = (self.rx_seq as usize) % RX_BUF_LEN;
        // SAFETY: byte ring at a known address; slot is in-range.
        unsafe {
            let ptr = self.uart.rx_buf_addr() as *mut u8;
            *ptr.add(slot) = byte;
        }
        self.rx_seq = self.rx_seq.wrapping_add(1);
        let pos = (self.rx_seq as usize) % RX_BUF_LEN;
        let remaining = (RX_BUF_LEN - pos) as u16;
        self.rx_dma_state.stage_remaining(remaining);
    }

    fn handle_idle(&mut self, at: SimTime) {
        self.uart.on_rx_idle();
        self.poll_and_queue_tx(at);
    }

    /// Drive `Dxl::poll`, then look at any new TxBus / TxScheduler ops to
    /// surface bytes onto `uart_tx`. `StartNow` queues immediately starting
    /// at `t`; `Schedule` resolves `deadline_tick` to wall-clock and queues
    /// from there.
    fn poll_and_queue_tx(&mut self, t: SimTime) {
        let pre_bus = self.tx_bus_state.operations().len();
        let pre_sch = self.tx_scheduler_state.operations().len();

        {
            let mut bus = ServoBus {
                uart: &mut self.uart,
            };
            self.dxl.poll(&self.shared, &mut bus);
        }

        let bus_ops = self.tx_bus_state.operations();
        for op in &bus_ops[pre_bus..] {
            if let TxBusOp::StartNow { byte_count } = *op {
                self.queue_tx_bytes(byte_count, t);
            }
        }

        let sch_ops = self.tx_scheduler_state.operations();
        for op in &sch_ops[pre_sch..] {
            if let ScheduleOp::Schedule {
                deadline_tick,
                byte_count,
                ..
            } = *op
            {
                let fire_at = self.deadline_to_wall(deadline_tick, t);
                self.queue_tx_bytes(byte_count, fire_at);
            }
        }
    }

    fn queue_tx_bytes(&mut self, byte_count: u16, start_at: SimTime) {
        let bytes = unsafe {
            core::slice::from_raw_parts(self.uart.tx_buf_addr() as *const u8, byte_count as usize)
        };
        let stride = 10 * bit_period_ns(self.baud);
        for (i, &b) in bytes.iter().enumerate() {
            self.uart_tx.queue_byte(b, start_at + i as u64 * stride);
        }
    }

    /// Convert a chip-side `deadline_tick` (u16 TIM2 tick) to a wall-clock
    /// `SimTime` by computing the forward delta from `t` in chip ticks and
    /// converting via the device's clock frequency. Assumes the deadline is
    /// in the future from `t` (true for Plain / RDT-based schedules; not
    /// for already-passed deadlines, which the production code lets fire
    /// ASAP).
    fn deadline_to_wall(&self, deadline_tick: u16, t: SimTime) -> SimTime {
        let now_tick = self.chip_tick(t);
        let delta_ticks = deadline_tick.wrapping_sub(now_tick) as u64;
        let delta_ns = delta_ticks.saturating_mul(1_000_000_000) / self.freq_hz();
        t + delta_ns
    }
}

impl EventSource for Servo {
    fn next_event_time(&self) -> Option<SimTime> {
        [self.uart_rx.next_wake(), self.uart_tx.next_wake()]
            .into_iter()
            .flatten()
            .min()
    }

    fn advance(&mut self, t: SimTime) -> Vec<Effect> {
        use crate::sim::uart::RxEffect;
        let rx_effects = self.uart_rx.advance(t);
        for eff in rx_effects {
            match eff {
                RxEffect::ByteComplete { byte, .. } => self.handle_byte(byte, t),
                RxEffect::IdleDetected { .. } => self.handle_idle(t),
            }
        }
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
        if !rising {
            self.handle_falling_edge(at);
        }
    }

    fn reset(&mut self) {
        self.rebuild_uart();
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

struct BuiltUart {
    uart: ServoUart,
    tx_bus_state: TxBusState,
    clock_trim_state: ClockTrimState,
    usart_baud_state: UsartBaudState,
    edge_dma_state: EdgeDmaState,
    rx_dma_state: RxDmaState,
    tx_scheduler_state: TxSchedulerState,
    fast_last_scheduler_state: FastLastSchedulerState,
}

fn build_uart(baud: BaudRate, dxl_id: Id, rdt_us: u32) -> BuiltUart {
    let (mock_edge_dma, edge_dma_state) = mock_edge_dma();
    let (mock_clock_trim, clock_trim_state) = mock_clock_trim();
    let (mock_usart_baud, usart_baud_state) = mock_usart_baud();
    let (mock_rx_dma, rx_dma_state) = mock_rx_dma();
    let (mock_tx_scheduler, tx_scheduler_state) = mock_tx_scheduler();
    let (mock_tx_bus, tx_bus_state) = mock_tx_bus();
    let (mock_fast_last_scheduler, fast_last_scheduler_state) = mock_fast_last_scheduler();

    let codec: Codec<_, SoftwareCrcUmts, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN> =
        Codec::new(mock_edge_dma);
    let inner_clock = DxlClock::new(baud, mock_usart_baud, mock_clock_trim);
    let fast_last = FastLast::new(mock_fast_last_scheduler);
    let uart = DxlUart::new(
        codec,
        inner_clock,
        mock_rx_dma,
        mock_tx_scheduler,
        mock_tx_bus,
        fast_last,
        dxl_id.as_byte(),
        rdt_us,
    );

    BuiltUart {
        uart,
        tx_bus_state,
        clock_trim_state,
        usart_baud_state,
        edge_dma_state,
        rx_dma_state,
        tx_scheduler_state,
        fast_last_scheduler_state,
    }
}

struct ServoBus<'a, P: Providers, const RX: usize, const EDGE: usize, const TX: usize> {
    uart: &'a mut DxlUart<P, RX, EDGE, TX>,
}

impl<P: Providers, const RX: usize, const EDGE: usize, const TX: usize> DxlBus
    for ServoBus<'_, P, RX, EDGE, TX>
{
    fn poll<D: DxlDispatcher>(&mut self, dispatcher: &mut D) {
        self.uart
            .poll(|ev, ring, reply| dispatcher.on_event(ev, ring, reply));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::Sim;
    use crate::sim::host::Host;
    use dxl_protocol::types::Id;

    const SERVO_CLOCK: Clock = Clock::new(48_000_000);
    const BAUD: BaudRate = BaudRate::B115200;

    #[test]
    fn defaults_for_dxl_id_and_rdt_us_match_constants() {
        let mut sim = Sim::default();
        let id = sim.add_device(|id| Servo::new(id, SERVO_CLOCK, BAUD));
        let s = sim.device::<Servo>(id).unwrap();
        assert_eq!(s.dxl_id(), DEFAULT_DXL_ID);
        assert_eq!(s.rdt_us(), DEFAULT_RDT_US);
        assert_eq!(s.clock(), SERVO_CLOCK);
        assert_eq!(s.baud(), BAUD);
    }

    #[test]
    fn with_dxl_id_overrides_default() {
        let mut sim = Sim::default();
        let id = sim.add_device(|id| Servo::new(id, SERVO_CLOCK, BAUD).with_dxl_id(Id::new(0x07)));
        assert_eq!(sim.device::<Servo>(id).unwrap().dxl_id(), Id::new(0x07));
    }

    #[test]
    fn with_rdt_us_overrides_default() {
        let mut sim = Sim::default();
        let id = sim.add_device(|id| Servo::new(id, SERVO_CLOCK, BAUD).with_rdt_us(500));
        assert_eq!(sim.device::<Servo>(id).unwrap().rdt_us(), 500);
    }

    #[test]
    fn idle_servo_yields_no_next_event() {
        let mut sim = Sim::default();
        let id = sim.add_device(|id| Servo::new(id, SERVO_CLOCK, BAUD));
        assert_eq!(sim.device::<Servo>(id).unwrap().next_event_time(), None);
    }

    #[test]
    fn host_ping_to_dxl_id_zero_routes_through_servo_to_a_reply() {
        let mut sim = Sim::default();
        let host = sim.add_device(|id| Host::new(id, SERVO_CLOCK, BAUD));
        sim.add_device(|id| Servo::new(id, SERVO_CLOCK, BAUD).with_dxl_id(Id::new(0)));

        sim.advance(SimTime::from_ms(10), |sim, now| {
            sim.device_mut::<Host>(host)
                .unwrap()
                .send_ping(now, Id::new(0));
        });

        let rx = sim.device::<Host>(host).unwrap().rx_bytes();
        assert!(!rx.is_empty(), "host received nothing");
    }
}

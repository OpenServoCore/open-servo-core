//! `Servo` — DXL servo device under test. Composes `Dxl` (services-layer
//! dispatcher state), a `DxlUart` typed with the chip-lib-mirrored buffer
//! sizes from [`crate::sim::defaults`], `Shared` (control table), and the 7
//! spy state companions from `crate::mocks`. Drives inbound wire edges
//! through the codec's trait surface (byte ring + edge ring + IDLE
//! callback), invokes `Dxl::poll` to produce a Status reply, and re-encodes
//! the reply's TX bytes onto outbound wire edges via `UartTx`.

use std::any::Any;

use dxl_protocol::SoftwareCrcUmts;
use dxl_protocol::types::Id;
use osc_core::services::dxl::Dxl;
use osc_core::traits::{DxlBus, DxlDispatcher};
use osc_core::{BaudRate, ConfigDefaults, RegionStorage, Shared, StatusReturnLevel};
use osc_drivers::dxl::uart::DxlUart;
use osc_drivers::dxl::uart::clock::Clock as DxlClock;
use osc_drivers::dxl::uart::codec::Codec;
use osc_drivers::dxl::uart::fast_last::FastLast;
use osc_drivers::mocks::{FastLastSchedulerOp, ScheduleOp, TestProviders, TxBusOp};
use osc_drivers::traits::dxl::{DmaFlags, Providers, SendKind};

use crate::mocks::{
    ClockTrimState, EdgeDmaState, FastLastSchedulerState, RxDmaState, TelemetryState, TxBusState,
    TxSchedulerState, UsartBaudState, WireClockState, mock_clock_trim, mock_edge_dma,
    mock_fast_last_scheduler, mock_rx_dma, mock_telemetry, mock_tx_bus, mock_tx_scheduler,
    mock_usart_baud_with_comp, mock_wire_clock,
};
use crate::sim::defaults::{
    DEFAULT_BAUD, DEFAULT_RDT_US, EDGE_BUF_LEN, RX_BUF_LEN, TX_BUF_LEN, default_servo_clock,
};
use crate::sim::uart::{UartRx, UartTx};
use crate::sim::{Clock, DeviceId, Effect, EventSource, HsiClock, SimTime};

pub const DEFAULT_DXL_ID: Id = Id::new(1);
pub const DEFAULT_MODEL_NUMBER: u16 = 0xC0DE;
pub const DEFAULT_FIRMWARE_VERSION: u8 = 0x01;

/// Sim's stand-in for the firmware's boot-time `ConfigDefaults`. Stamped
/// into the control table by [`Servo::reset_table`] — sim has no EEPROM,
/// so this seed is re-applied on every power-cycle reset.
const SIM_CONFIG_DEFAULTS: ConfigDefaults = ConfigDefaults {
    pos_min_phys_urad: 0,
    pos_max_phys_urad: 0,
    vdd_mv: 0,
    dxl_id: DEFAULT_DXL_ID.as_byte(),
    dxl_baud: DEFAULT_BAUD,
    dxl_return_delay_2us: (DEFAULT_RDT_US / 2) as u8,
};

type ServoUart = DxlUart<TestProviders, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

pub struct Servo {
    id: DeviceId,
    /// Per-servo HSI model. Carries the chip's nominal HCLK, the
    /// test-controlled factory drift ratio (set via
    /// [`set_hsi_drift`](Self::set_hsi_drift)), and the latest absolute
    /// trim correction drained from [`ClockTrimState`]. Hardware-shaped,
    /// not a control-table register: persists across power-cycle resets.
    /// `clock()` returns the composed live HCLK.
    hsi_clock: HsiClock,

    /// Bus visibility. When `false`, the wire still delivers edges to
    /// `uart_rx` (logged for fidelity), but `handle_falling_edge` /
    /// `handle_byte` / `handle_idle` short-circuit, no dispatcher poll runs,
    /// and any TX edges produced by `uart_tx` are dropped. Distinct from
    /// `StatusReturnLevel::None`, which leaves the dispatcher running.
    connected: bool,
    /// Set by `disconnect(true)` (power-cycle); consumed by `connect()` to
    /// trigger `rebuild_chip_side` so the next bus activity sees a
    /// freshly-defaulted control table — matching the firmware's
    /// eeprom-less reset behaviour where any prior `set_*` writes are lost.
    pending_reset_on_connect: bool,

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
    /// Shared cell behind the `MockWireClock`'s `now()` — `chip_tick(at)`
    /// gets staged here at the top of every event handler that pokes the
    /// codec, so the driver self-sources the matching tick value from its
    /// `WireClock` provider.
    wire_clock_state: WireClockState,

    /// Wire-condition miss counters recorded by the driver composite.
    telemetry_state: TelemetryState,

    /// Next SysTick CMP-match wall-clock for the Fast Last fold body, if
    /// any. Set by `Schedule`, cleared by `Cancel` or by `advance` after
    /// firing. Surfaced via [`EventSource::next_event_time`] so the engine
    /// advances time to it; [`EventSource::advance`] calls `on_fold_step`
    /// at the fire instant.
    systick_fire: Option<SimTime>,
    /// Cursor into `fast_last_scheduler_state.operations()` — separates
    /// ops produced by `Dxl::poll` / `on_fold_step` since the last drain
    /// from earlier ones. Reset to 0 on `rebuild_uart` (the state vec is
    /// rebuilt fresh).
    fast_last_drained: usize,

    /// Wall-clock equivalent of the chip-side CC-compare that fires
    /// `on_tx_start` at the scheduled TX deadline. Set when a
    /// `TxScheduler::Schedule` op is staged; cleared after the body fires.
    /// `None` between replies (and on the `TxBus::start_now` path — that
    /// activation is inline, no CC-compare).
    tx_start_fire: Option<SimTime>,

    /// FastLast deferred-schedule stash — chip-side mirror of the
    /// `DxlTxScheduler::fast_last_stash` field. Populated when the driver
    /// calls `Schedule` with `SendKind::FastLast`; consumed by
    /// `CommitPending` (emitted from inside the FastLast walk's final
    /// anchor body via `FastLast::on_step`'s commit closure). Holds the
    /// lifted wall-clock deadline + byte count.
    tx_pending_stash: Option<(SimTime, u16)>,
    /// Cursor into `tx_scheduler_state.operations()` — separates ops
    /// produced since the last drain pass from earlier ones. Polled in
    /// `advance` after each `on_fold_step` so a `CommitPending` emitted by
    /// the walk's final body lands the burst within the same wall instant.
    tx_scheduler_drained: usize,

    uart_tx: UartTx,
    uart_rx: UartRx,
    rx_seq: u32,
    edge_seq: u32,

    /// Per-baud RX edge-stamp compensation handed to the driver's
    /// `UsartBaud::rx_edge_comp_ticks` mock and used by `handle_falling_edge`
    /// to offset the IC stamp value past the wire edge. Models the chip's
    /// IC-filter delay (or any equivalent stamp offset) so the driver-side
    /// compensation has something real to subtract. Default `0`: stamps land
    /// exactly at wire-edge time and the driver subtracts nothing — the
    /// shape every existing timing test exercises. Tests that exercise the
    /// compensation path set this to a non-zero value via
    /// [`Self::set_rx_edge_comp_ticks`].
    rx_edge_comp_ticks: u16,
}

impl Servo {
    pub fn new(id: DeviceId) -> Self {
        let mut s = Self {
            id,
            hsi_clock: HsiClock::new(default_servo_clock()),

            connected: true,
            pending_reset_on_connect: false,

            dxl: Dxl::new(),
            shared: Shared::new(),
            uart: build_uart(DEFAULT_BAUD, DEFAULT_DXL_ID, DEFAULT_RDT_US, 0).uart,

            tx_bus_state: TxBusState::default(),
            clock_trim_state: ClockTrimState::default(),
            usart_baud_state: UsartBaudState::default(),
            edge_dma_state: EdgeDmaState::default(),
            rx_dma_state: RxDmaState::default(),
            tx_scheduler_state: TxSchedulerState::default(),
            fast_last_scheduler_state: FastLastSchedulerState::default(),
            wire_clock_state: WireClockState::default(),
            telemetry_state: TelemetryState::default(),

            systick_fire: None,
            fast_last_drained: 0,

            tx_start_fire: None,
            tx_pending_stash: None,
            tx_scheduler_drained: 0,

            uart_tx: UartTx::new(DEFAULT_BAUD),
            uart_rx: UartRx::new(DEFAULT_BAUD),
            rx_seq: 0,
            edge_seq: 0,

            rx_edge_comp_ticks: 0,
        };
        s.rebuild_chip_side();
        s
    }

    /// Construct a `Servo` and run `init` against it before returning.
    /// Convenience wrapper for the `add_device` closure pattern, e.g.
    /// `sim.add_device(|id| Servo::setup(id, |s| s.set_dxl_id(Id::new(1))))`.
    pub fn setup(id: DeviceId, init: impl FnOnce(&mut Self)) -> Self {
        let mut s = Self::new(id);
        init(&mut s);
        s
    }

    /// Override the per-servo nominal HCLK. Resets any prior factory drift
    /// and applied trim — this is the "what chip is plugged in" knob, not a
    /// runtime adjustment. Hardware-shaped: survives a power-cycle reset.
    pub fn set_clock(&mut self, clock: Clock) {
        self.hsi_clock = HsiClock::new(clock);
    }

    /// Set the simulated factory HSI drift as a rational fraction:
    /// `live_factory = nominal × (den + num) / den`. Use `(1, 50)` for +2%,
    /// `(-1, 50)` for −2%, etc. The denominator must be non-zero.
    /// Hardware-shaped: survives a power-cycle reset (a real chip's factory
    /// HSI offset doesn't reset).
    pub fn set_hsi_drift(&mut self, num: i32, den: u32) {
        self.hsi_clock.set_factory_drift(num, den);
    }

    pub fn set_dxl_id(&mut self, dxl_id: Id) {
        self.shared
            .table
            .config
            .with_mut(|c| c.comms.id = dxl_id.as_byte());
        self.rebuild_uart();
    }

    pub fn set_rdt_us(&mut self, rdt_us: u32) {
        self.shared
            .table
            .config
            .with_mut(|c| c.comms.return_delay_2us = (rdt_us / 2) as u8);
        self.rebuild_uart();
    }

    pub fn set_baud(&mut self, baud: BaudRate) {
        self.shared
            .table
            .config
            .with_mut(|c| c.comms.baud_rate_idx = baud);
        self.rebuild_uart();
    }

    pub fn set_status_return_level(&mut self, level: StatusReturnLevel) {
        self.shared
            .table
            .config
            .with_mut(|c| c.comms.status_return_level = level);
    }

    /// Set the simulated per-baud RX edge-stamp compensation, in chip
    /// HCLK ticks. Stamps the IC delivers to the driver shift by this many
    /// ticks past the wire edge — modeling V006's TIM2_CH4 IC filter (or
    /// any equivalent stamp offset on a different chip). The driver's mock
    /// `UsartBaud::rx_edge_comp_ticks` returns the same value so the edge
    /// parser's read-time subtraction recovers wire-edge time. Hardware-
    /// shaped: survives a power-cycle reset (the IC filter pick is fixed
    /// by baud + clock-tree, not RAM state). Rebuilds the chip-side codec/
    /// UART so the new value plumbs through immediately.
    pub fn set_rx_edge_comp_ticks(&mut self, ticks: u16) {
        self.rx_edge_comp_ticks = ticks;
        self.rebuild_uart();
    }

    /// Currently-staged simulated RX edge-stamp compensation, in chip HCLK
    /// ticks. Tests sample this to align their expected wire-edge offsets
    /// with the value the chip-side driver will subtract.
    pub fn rx_edge_comp_ticks(&self) -> u16 {
        self.rx_edge_comp_ticks
    }

    /// Take the servo off the bus. Wire edges still arrive (uart_rx still
    /// advances for log fidelity) but the chip-side stays dormant: no
    /// dispatcher poll runs and no TX is emitted. When `reset` is `true`
    /// (power-line disconnect), the next [`connect`](Self::connect) wipes the
    /// control table back to [`SIM_CONFIG_DEFAULTS`] — any `set_*` writes
    /// made before disconnect are lost, matching the firmware's eeprom-less
    /// reset behaviour. When `reset` is `false` (data-line wiggle),
    /// reconnect resumes from whatever in-flight state was active at
    /// disconnect.
    pub fn disconnect(&mut self, reset: bool) {
        self.connected = false;
        if reset {
            self.pending_reset_on_connect = true;
        }
    }

    /// Restore the servo to the bus. Reverses a [`disconnect`](Self::disconnect)
    /// — if that disconnect was power-line (`reset = true`), the chip-side
    /// is rebuilt from defaults before the connect flag flips.
    pub fn connect(&mut self) {
        if self.pending_reset_on_connect {
            self.rebuild_chip_side();
            self.pending_reset_on_connect = false;
        }
        self.connected = true;
    }

    /// Flip the `lifecycle.torque_enable` lock bit. Writes to torque-gated
    /// config fields are rejected with `Access` while this is `true`.
    pub fn set_torque_enabled(&self, enabled: bool) {
        self.shared
            .table
            .control
            .with_mut(|c| c.lifecycle.torque_enable = enabled);
    }

    pub fn clock(&self) -> Clock {
        self.hsi_clock.live()
    }

    pub fn hsi_clock(&self) -> &HsiClock {
        &self.hsi_clock
    }

    pub fn baud(&self) -> BaudRate {
        self.shared.table.config.with(|c| c.comms.baud_rate_idx)
    }

    pub fn dxl_id(&self) -> Id {
        Id::new(self.shared.table.config.with(|c| c.comms.id))
    }

    pub fn rdt_us(&self) -> u32 {
        self.shared.table.config.with(|c| c.comms.return_delay_2us) as u32 * 2
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

    pub fn baud_ops(&self) -> Vec<BaudRate> {
        self.usart_baud_state.operations()
    }

    pub fn trim_ops(&self) -> Vec<i32> {
        self.clock_trim_state.operations()
    }

    /// Sub-step phase-error projection — chains through `DxlUart` to the
    /// integrator's residual×distance/window calculation. Tests sample
    /// this after driving drift convergence to assert the residual is
    /// signed correctly and bounded by the steady-phase deadband.
    pub fn projected_phase_error_hclk(&self, distance_hclk: u32) -> i32 {
        self.uart.projected_phase_error_hclk(distance_hclk)
    }

    fn rebuild_uart(&mut self) {
        let (baud, dxl_id, rdt_us) = self.shared.table.config.with(|c| {
            (
                c.comms.baud_rate_idx,
                Id::new(c.comms.id),
                c.comms.return_delay_2us as u32 * 2,
            )
        });
        let built = build_uart(baud, dxl_id, rdt_us, self.rx_edge_comp_ticks);
        self.uart = built.uart;
        self.tx_bus_state = built.tx_bus_state;
        self.clock_trim_state = built.clock_trim_state;
        self.usart_baud_state = built.usart_baud_state;
        self.edge_dma_state = built.edge_dma_state;
        self.rx_dma_state = built.rx_dma_state;
        self.tx_scheduler_state = built.tx_scheduler_state;
        self.fast_last_scheduler_state = built.fast_last_scheduler_state;
        self.wire_clock_state = built.wire_clock_state;
        self.telemetry_state = built.telemetry_state;
        self.dxl = Dxl::new();
        self.uart_tx = UartTx::new(baud);
        self.uart_rx = UartRx::new(baud);
        self.rx_seq = 0;
        self.edge_seq = 0;
        self.systick_fire = None;
        self.fast_last_drained = 0;
        self.tx_start_fire = None;
        self.tx_pending_stash = None;
        self.tx_scheduler_drained = 0;
        // Fresh ClockTrimState — reset HsiClock's cursor so subsequent
        // drain_ops calls see the new log from index 0.
        self.hsi_clock.reset_drain();
    }

    /// Models a power-up reset: wipe the control table and rebuild the
    /// chip-side codec / UART / spy states.
    fn rebuild_chip_side(&mut self) {
        self.reset_table();
        self.rebuild_uart();
    }

    /// Wipe `shared` to a freshly-defaulted state. Mirrors the firmware's
    /// eeprom-less reset: every config field returns to its boot constant,
    /// no `set_*` writes survive.
    fn reset_table(&mut self) {
        self.shared = Shared::new();
        self.shared.table.seed_config_defaults(&SIM_CONFIG_DEFAULTS);
        self.shared.table.config.with_mut(|c| {
            c.identity.model_number = DEFAULT_MODEL_NUMBER;
            c.identity.firmware_version = DEFAULT_FIRMWARE_VERSION;
        });
    }

    /// Full-width chip tick (WireClock u32 domain, truncated modulo 2³² to
    /// fit a u32). Used to stage `WireClock::now()`, derive wire-clock
    /// schedule deadlines, and resolve absolute u32 schedule deadlines back
    /// to wall time. ET ring IC stamps see the low 16 bits via `chip_tick
    /// as u16` — matches the chip-side TIM2/SysTick HCLK contract.
    fn chip_tick(&self, at: SimTime) -> u32 {
        self.hsi_clock.live().to_local(at) as u32
    }

    fn freq_hz(&self) -> u64 {
        self.hsi_clock.live().freq_hz() as u64
    }

    /// Inbound falling edge — write a TIM2 tick into the codec's edge ring
    /// and advance NDTR on the spy state. The classifier walk
    /// (`on_rx_edge_advance`) and the `Dxl::poll` that follows are gated on
    /// HT/TC crossings of the edge ring: production V006 has no per-edge
    /// IRQ, only DMA1_CH7 HT (half-mark) and TC (wrap). Intra-half edges
    /// accumulate silently; the IDLE backstop (`handle_idle`) catches short
    /// packets that never trip HT.
    fn handle_falling_edge(&mut self, at: SimTime) {
        // Stamp shifts by `rx_edge_comp_ticks` past the wire-edge time —
        // models the chip's IC-filter output delay. The driver subtracts
        // the same value at read-from-ring time (via Clock's
        // `rx_edge_comp_ticks` + EdgeParser's compensation), recovering the
        // true wire-edge tick. Tests pick a non-zero value to exercise
        // that round-trip; default `0` matches every existing timing test.
        let tick = self
            .chip_tick(at)
            .wrapping_add(self.rx_edge_comp_ticks as u32) as u16;
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
    }

    /// Decoded byte from the line model — write to the codec's RX byte ring
    /// and update NDTR. On `RX_BUF_LEN/2` (HT) and `RX_BUF_LEN` (TC)
    /// crossings, fires `on_rx_advance` (mirroring the production DMA1_CH5
    /// HT/TC ISR body) and drives the parser drain via `poll_and_queue_tx`
    /// — production runs `services.poll` from the same vector so drain
    /// cadence tracks complete-byte boundaries (`dxl-streaming-rx.md` §3 /
    /// §4.4 / §5.2).
    fn handle_byte(&mut self, byte: u8, at: SimTime) {
        log::trace!(
            "servo[{:?}]: handle_byte byte=0x{:02X} at={:?}",
            self.id,
            byte,
            at
        );
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

        // Per-byte status-start wake (USART1 RXNE trap model): while the
        // watch window is open every received byte routes one wake, so a
        // deferred FAST slot k > 0 observes the Status packet's first
        // byte within a byte-time of its arrival. Runs before the HT/TC
        // poll below, mirroring the chip where RXNE asserts at the byte's
        // stop bit. The wake can schedule (slot wire start, fold CMP) —
        // drain both op logs so the sim stages the fires.
        if self.rx_dma_state.status_start_watched() {
            self.wire_clock_state.stage_now(self.chip_tick(at));
            self.uart.on_status_start();
            self.drain_tx_scheduler_ops(at);
            self.drain_fast_last_ops(at);
        }

        let crossed_ht = pos == RX_BUF_LEN / 2;
        let crossed_tc = pos == 0;
        if crossed_ht || crossed_tc {
            self.rx_dma_state.stage_next_flags(DmaFlags {
                ht: crossed_ht,
                tc: crossed_tc,
            });
            self.wire_clock_state.stage_now(self.chip_tick(at));
            self.uart.on_rx_advance();
            self.poll_and_queue_tx(at);
        }
    }

    fn handle_idle(&mut self, at: SimTime) {
        log::trace!("servo[{:?}]: handle_idle at={:?}", self.id, at);
        self.wire_clock_state.stage_now(self.chip_tick(at));
        self.uart.on_rx_idle();
        self.poll_and_queue_tx(at);
    }

    /// Drive `Dxl::poll`, then look at any new TxBus / TxScheduler ops to
    /// surface bytes onto `uart_tx`. `StartNow` queues immediately starting
    /// at `t`; `Schedule` resolves the (packet_end_tick, delay_ticks) pair
    /// to wall-clock — for `SendKind::Plain` queues from there, for
    /// `SendKind::FastLast` stashes pending until `CommitPending` lands
    /// from the walk's final-anchor body.
    fn poll_and_queue_tx(&mut self, t: SimTime) {
        let pre_bus = self.tx_bus_state.operations().len();

        self.wire_clock_state.stage_now(self.chip_tick(t));
        {
            let mut bus = ServoBus {
                uart: &mut self.uart,
            };
            self.dxl.poll(&self.shared, &mut bus);
        }
        // The driver applies pending trim at every RX-side packet
        // boundary now (`Clock::on_rx_packet_end`), not just at
        // `on_tx_complete`. Drain the mock's log here so foreign-
        // instruction packets — which never produce a TX — still
        // commit their correction to the sim's live HSI.
        let ops = self.clock_trim_state.operations();
        self.hsi_clock.drain_ops(&ops);

        let bus_ops = self.tx_bus_state.operations();
        for op in &bus_ops[pre_bus..] {
            if let TxBusOp::StartNow { byte_count } = *op {
                log::trace!(
                    "servo[{:?}]: tx_bus StartNow byte_count={} t={:?}",
                    self.id,
                    byte_count,
                    t
                );
                self.uart_tx.queue_burst_indirect(0, byte_count, t);
            }
        }

        self.drain_tx_scheduler_ops(t);
        self.drain_fast_last_ops(t);
    }

    /// Walk new `ScheduleOp` entries since the last drain. Plain schedules
    /// queue the burst at the resolved wall-clock fire instant. FastLast
    /// schedules stash the deadline; the matching `CommitPending` (emitted
    /// by the walk's final-anchor body) takes the stash and queues the
    /// burst — by construction the commit lands within ~1 byte_time of the
    /// deadline, so `max(deadline, t)` clamps to "now" when the busy-wait
    /// exit ran slightly past the deadline (recursive-fire mode at 3M).
    fn drain_tx_scheduler_ops(&mut self, t: SimTime) {
        let sch_ops = self.tx_scheduler_state.operations();
        for op in &sch_ops[self.tx_scheduler_drained..] {
            match *op {
                ScheduleOp::Schedule {
                    deadline,
                    byte_count,
                    kind,
                } => {
                    let fire_at = self.deadline_to_wall(deadline, t);
                    log::trace!(
                        "servo[{:?}]: tx_scheduler Schedule kind={:?} deadline={} byte_count={} fire_at={:?} (t={:?})",
                        self.id,
                        kind,
                        deadline,
                        byte_count,
                        fire_at,
                        t
                    );
                    match kind {
                        SendKind::Plain => {
                            self.uart_tx.queue_burst_indirect(0, byte_count, fire_at);
                            self.tx_start_fire = Some(fire_at);
                        }
                        SendKind::FastLast => {
                            self.tx_pending_stash = Some((fire_at, byte_count));
                        }
                    }
                }
                ScheduleOp::CommitPending => {
                    if let Some((deadline_wall, byte_count)) = self.tx_pending_stash.take() {
                        let fire_at = deadline_wall.max(t);
                        log::trace!(
                            "servo[{:?}]: tx_scheduler CommitPending byte_count={} fire_at={:?} (t={:?})",
                            self.id,
                            byte_count,
                            fire_at,
                            t
                        );
                        self.uart_tx.queue_burst_indirect(0, byte_count, fire_at);
                        self.tx_start_fire = Some(fire_at);
                    }
                }
                ScheduleOp::Cancel => {
                    self.tx_pending_stash = None;
                }
            }
        }
        self.tx_scheduler_drained = sch_ops.len();
    }

    /// Walk new `FastLastSchedulerOp` entries since the last drain and
    /// resolve their absolute-u32 deadlines into wall-clock state.
    /// `SetBusyWaitDeadline` is a busy-wait threshold (no fire), `Schedule` derives
    /// the next CMP-match `SimTime`, `Cancel` clears it. Called after every
    /// `Dxl::poll` and every `on_fold_step` — both paths can append ops.
    fn drain_fast_last_ops(&mut self, t: SimTime) {
        let ops = self.fast_last_scheduler_state.operations();
        for op in &ops[self.fast_last_drained..] {
            match *op {
                FastLastSchedulerOp::SetBusyWaitDeadline { .. } => {
                    // Busy-wait threshold only — no wall-clock fire to stage.
                }
                FastLastSchedulerOp::Schedule { deadline } => {
                    let mut fire = self.deadline_to_wall(deadline, t);
                    if fire <= t {
                        // Past-CMP guard — chip-side scheduler clamps
                        // `cmp = max(cmp, now + 1)` so the match still
                        // latches. Sim mirrors with one chip-tick of
                        // forward slop (≈21 ns at 48 MHz).
                        let one_tick_ns = (1_000_000_000u64 / self.freq_hz()).max(1);
                        fire = t + one_tick_ns;
                    }
                    log::trace!(
                        "servo[{:?}]: fast_last Schedule deadline={} fire_at={:?} (t={:?})",
                        self.id,
                        deadline,
                        fire,
                        t
                    );
                    self.systick_fire = Some(fire);
                }
                FastLastSchedulerOp::Cancel => {
                    self.systick_fire = None;
                }
            }
        }
        self.fast_last_drained = ops.len();
    }

    /// Convert an absolute chip-side u32 deadline to a wall-clock `SimTime`
    /// by computing the forward delta from `t` in chip ticks and converting
    /// via the device's clock frequency. Past-deadline (delta < 0) returns
    /// `t` so the consumer can clamp to "fire ASAP" — production hardware
    /// behaves the same way (the match latches immediately).
    fn deadline_to_wall(&self, deadline: u32, t: SimTime) -> SimTime {
        let delta_ticks = deadline.wrapping_sub(self.chip_tick(t)) as i32;
        if delta_ticks <= 0 {
            return t;
        }
        let delta_ns = (delta_ticks as u64).saturating_mul(1_000_000_000) / self.freq_hz();
        t + delta_ns
    }
}

impl EventSource for Servo {
    fn next_event_time(&self) -> Option<SimTime> {
        [
            self.uart_rx.next_wake(),
            self.uart_tx.next_wake(),
            self.systick_fire,
            self.tx_start_fire,
            self.uart_tx.next_tc(),
        ]
        .into_iter()
        .flatten()
        .min()
    }

    fn advance(&mut self, t: SimTime) -> Vec<Effect> {
        use crate::sim::uart::RxEffect;
        let rx_effects = self.uart_rx.advance(t);
        if self.connected {
            for eff in rx_effects {
                match eff {
                    RxEffect::ByteComplete { byte, .. } => self.handle_byte(byte, t),
                    RxEffect::IdleDetected { .. } => self.handle_idle(t),
                }
            }
        }
        if self.connected {
            // Fast Last CMP-match fold body. Loop because `on_fold_step` may
            // re-schedule and the new CMP can land at `t + 1tick` (past-CMP
            // clamp); fire all due bodies before letting tx_buf settle so
            // any chain-CRC patch lands before the patched byte ships out
            // of `uart_tx.advance` below.
            while let Some(fire) = self.systick_fire {
                if fire > t {
                    break;
                }
                self.systick_fire = None;
                log::trace!("servo[{:?}]: on_fold_step fire_at={:?}", self.id, fire);
                self.uart.on_fold_step();
                self.drain_fast_last_ops(t);
                self.drain_tx_scheduler_ops(t);
            }
            // CC-compare body. On Fast Last replies this also runs the
            // post-fire tail fold inside `on_tx_start`, draining any GUARD
            // bytes still in the RX ring and patching the trailing chain
            // CRC into `tx_buf` BEFORE `uart_tx.advance` reads it below.
            if let Some(fire) = self.tx_start_fire
                && fire <= t
            {
                self.tx_start_fire = None;
                log::trace!("servo[{:?}]: on_tx_start fire_at={:?}", self.id, fire);
                self.uart.on_tx_start();
            }
        }
        // SAFETY: `tx_buf_addr()` returns the codec's heapless::Vec<u8,
        // TX_BUF_LEN> storage start; the first `tx_len()` bytes are
        // initialized (encoded by the most recent send_slot). Indirect
        // entries in uart_tx pending always point inside this initialized
        // prefix because they were queued from `byte_count` = tx_len at
        // StartNow time, and tx_len doesn't shrink until on_tx_complete
        // (which runs after all bytes have drained).
        let tx_buf = unsafe {
            core::slice::from_raw_parts(
                self.uart.tx_buf_addr() as *const u8,
                self.uart.tx_len() as usize,
            )
        };
        let tx = self.uart_tx.advance(t, tx_buf);
        if !self.connected {
            return Vec::new();
        }
        // USART TC body. Auto-gated off `UartTx`'s wire-busy state —
        // fires once the last queued byte's frame end is in the past.
        if let Some(fire) = self.uart_tx.take_tc_if_due(t) {
            log::trace!("servo[{:?}]: on_tx_complete fire_at={:?}", self.id, fire);
            let _ = self.uart.on_tx_complete();
            // `on_tx_complete` is where the driver's integrator commits any
            // pending trim correction to `T::apply_ppm` — pull the new
            // applied ppm into the sim's HSI model so subsequent wire-edge
            // stamps reflect the post-trim HCLK.
            let ops = self.clock_trim_state.operations();
            self.hsi_clock.drain_ops(&ops);
        }
        let effects: Vec<_> = tx
            .into_iter()
            .map(|(at_ns, rising)| Effect::WireEdge {
                at: SimTime::from_ns(at_ns),
                rising,
            })
            .collect();
        if !effects.is_empty() {
            log::trace!(
                "servo[{:?}]: advance t={:?} emit {} wire edges",
                self.id,
                t,
                effects.len()
            );
        }
        effects
    }

    fn receive_edge(&mut self, at: SimTime, rising: bool) {
        log::trace!(
            "servo[{:?}]: receive_edge at={:?} rising={} connected={}",
            self.id,
            at,
            rising,
            self.connected
        );
        self.uart_rx.receive_edge(at, rising);
        if !rising && self.connected {
            self.handle_falling_edge(at);
        }
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
    wire_clock_state: WireClockState,
    telemetry_state: TelemetryState,
}

fn build_uart(baud: BaudRate, dxl_id: Id, rdt_us: u32, rx_edge_comp_ticks: u16) -> BuiltUart {
    let (mock_edge_dma, edge_dma_state) = mock_edge_dma();
    let (mock_clock_trim, clock_trim_state) = mock_clock_trim();
    let (mock_usart_baud, usart_baud_state) = mock_usart_baud_with_comp(rx_edge_comp_ticks);
    let (mock_rx_dma, rx_dma_state) = mock_rx_dma();
    let (mock_tx_scheduler, tx_scheduler_state) = mock_tx_scheduler();
    let (mock_tx_bus, tx_bus_state) = mock_tx_bus();
    let (mock_fast_last_scheduler, fast_last_scheduler_state) = mock_fast_last_scheduler();
    let (mock_wire_clock, wire_clock_state) = mock_wire_clock();
    let (mock_telemetry, telemetry_state) = mock_telemetry();

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
        mock_wire_clock,
        mock_telemetry,
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
        wire_clock_state,
        telemetry_state,
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
    use crate::sim::{Host, Sim};
    use dxl_protocol::types::Id;

    #[test]
    fn defaults_for_dxl_id_and_rdt_us_match_constants() {
        let mut sim = Sim::default();
        let id = sim.add_device(Servo::new);
        let s = sim.servo(id);
        assert_eq!(s.dxl_id(), DEFAULT_DXL_ID);
        assert_eq!(s.rdt_us(), DEFAULT_RDT_US);
        assert_eq!(s.clock(), default_servo_clock());
        assert_eq!(s.baud(), DEFAULT_BAUD);
    }

    #[test]
    fn set_dxl_id_overrides_default() {
        let mut sim = Sim::default();
        let id = sim.add_device(|id| Servo::setup(id, |s| s.set_dxl_id(Id::new(0x07))));
        assert_eq!(sim.servo(id).dxl_id(), Id::new(0x07));
    }

    #[test]
    fn set_rdt_us_overrides_default() {
        let mut sim = Sim::default();
        let id = sim.add_device(|id| Servo::setup(id, |s| s.set_rdt_us(500)));
        assert_eq!(sim.servo(id).rdt_us(), 500);
    }

    #[test]
    fn idle_servo_yields_no_next_event() {
        let mut sim = Sim::default();
        let id = sim.add_device(Servo::new);
        assert_eq!(sim.servo(id).next_event_time(), None);
    }

    fn ping_and_settle(sim: &mut Sim, host: DeviceId, target: Id) {
        sim.with_host(host, |h| {
            h.send_ping(target);
            h.wait_for_reply();
        });
    }

    #[test]
    fn data_line_disconnect_silences_ping() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let servo = sim.add_device(Servo::new);
        sim.servo_mut(servo).disconnect(false);

        ping_and_settle(&mut sim, host, DEFAULT_DXL_ID);

        assert!(sim.host(host).rx_bytes().is_empty());
    }

    #[test]
    fn data_line_reconnect_resumes_without_reset() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let servo = sim.add_device(Servo::new);
        sim.servo(servo).set_torque_enabled(true);

        {
            let s = sim.servo_mut(servo);
            s.disconnect(false);
            s.connect();
        }
        ping_and_settle(&mut sim, host, DEFAULT_DXL_ID);

        assert!(!sim.host(host).rx_bytes().is_empty());
        let torque = sim
            .servo(servo)
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable);
        assert!(torque, "data-line disconnect preserves RAM state");
    }

    #[test]
    fn power_cycle_disconnect_resets_chip_side() {
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let servo = sim.add_device(Servo::new);
        sim.servo(servo).set_torque_enabled(true);

        {
            let s = sim.servo_mut(servo);
            s.disconnect(true);
            s.connect();
        }
        ping_and_settle(&mut sim, host, DEFAULT_DXL_ID);

        assert!(!sim.host(host).rx_bytes().is_empty());
        let torque = sim
            .servo(servo)
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable);
        assert!(!torque, "power-cycle reset wipes RAM state");
    }

    #[test]
    fn power_cycle_loses_prior_set_writes() {
        const CONFIGURED_ID: Id = Id::new(5);
        let mut sim = Sim::default();
        let host = sim.add_device(Host::new);
        let servo = sim.add_device(|id| Servo::setup(id, |s| s.set_dxl_id(CONFIGURED_ID)));

        {
            let s = sim.servo_mut(servo);
            s.disconnect(true);
            s.connect();
        }
        ping_and_settle(&mut sim, host, DEFAULT_DXL_ID);

        let s = sim.servo(servo);
        assert_eq!(s.dxl_id(), DEFAULT_DXL_ID);
        assert_eq!(
            s.shared().table.config.with(|c| c.comms.id),
            DEFAULT_DXL_ID.as_byte()
        );
        assert!(
            !sim.host(host).rx_bytes().is_empty(),
            "after power-cycle the servo answers at DEFAULT_DXL_ID, not the prior set value",
        );
    }
}

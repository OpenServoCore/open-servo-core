//! Per-servo PFIC occupancy model. The transport vectors share one PFIC
//! level on the chip, so a handler body occupies the CPU and every event landing
//! meanwhile *pends* -- a flag per vector, not a queue -- and a burst of same-
//! vector events coalesces into one late delivery, exactly as pended IRQs do
//! on silicon. Ring bytes are DMA and always land at their wire tick; only
//! handler invocations defer. Handler effects land at entry: the model
//! charges occupancy, not intra-body effect timing.
//!
//! An optional kernel lane models the ADC scan tick as a periodic preemptor
//! at a selectable level against the bus vectors, charged by drive arm.
//!
//! Costs at budget come from `osc_servo_core::budget`, the table the bench
//! test `budgets` holds the chip to, each with its interrupt entry and exit.

use osc_servo_core::budget::probe::{BusProbe, Windows};
use osc_servo_core::budget::{self, Body, EVENT_REGIMES, Frame, PHASES, REGIMES, Regime};
use osc_servo_core::kernel::{self, DECIM_SLOW, phase};

use super::core::TICKS_PER_US;

/// Kernel tick period: one ADC scan per PWM period, HCLK ticks.
pub const KERNEL_PERIOD: u64 = budget::TICK_PERIOD as u64;

const _: () = assert!(TICKS_PER_US == budget::TICKS_PER_US as u64);

/// Kernel tick bodies at budget by regime and phase, the tick block's work
/// and layout included.
pub static KERNEL_AT_BUDGET: [[u64; PHASES]; REGIMES] = kernel_bodies(false);

/// The lowest body the typical lane draws: a uniform draw from here to the
/// budget has the mean budget for its mean, or more where the maximum sits
/// over twice the mean (the floor stops at zero).
pub static KERNEL_FLOOR: [[u64; PHASES]; REGIMES] = kernel_bodies(true);

const fn kernel_bodies(floor: bool) -> [[u64; PHASES]; REGIMES] {
    let mut t = [[0; PHASES]; REGIMES];
    let mut r = 0;
    while r < t.len() {
        let mut p = 0;
        while p < PHASES {
            let regime = Regime::ALL[r];
            let max = budget::tick_body(regime, p);
            let mean = budget::tick_body_mean(regime, p);
            let body = if floor {
                (2 * mean).saturating_sub(max)
            } else {
                max
            };
            t[r][p] = (body + budget::ENTRY_EXIT) as u64;
            p += 1;
        }
        r += 1;
    }
    t
}

/// Sim-time cost of each `ServoBus` handler body, us. Zero (the default)
/// delivers every event at its wire tick -- the ideal-CPU model the logical
/// suites pin.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct HandlerCost {
    pub on_break_us: u32,
    pub on_deadline_us: u32,
    pub on_tx_complete_us: u32,
    /// Per own frame a body dispatches, on top of the body: the frame's
    /// feed, dispatch, verdict and commit (transport sec 5.9: 80-160 us per
    /// host frame on the chip). Past the wire time of a frame it is what
    /// lets a backlog grow through a burst; a flat body cost cannot, since
    /// one body resolves every frame whole in the ring.
    pub per_frame_us: u32,
}

impl HandlerCost {
    /// Frame type `f` at budget: the break wake and each reply arm at their
    /// bodies' budgets, the rest of the frame's budget on the body that
    /// dispatches it (with the entries of the two deadline bodies a reply
    /// takes on the chip), rounded up to whole us.
    pub fn at_budget(f: Frame) -> Self {
        Self::split(
            budget::body(Body::BreakWake),
            budget::body(Body::TxDone),
            budget::frame(f),
        )
    }

    /// The same split of the mean budgets.
    pub fn typical(f: Frame) -> Self {
        Self::split(
            budget::body_mean(Body::BreakWake),
            budget::body_mean(Body::TxDone),
            budget::frame_mean(f),
        )
    }

    fn split(wake: u32, arm: u32, frame: u32) -> Self {
        let entry = budget::ENTRY_EXIT;
        let rest = frame.saturating_sub(wake + Frame::REPLY_ARMS * arm);
        Self {
            on_break_us: us_ceil(wake + entry),
            on_deadline_us: 0,
            on_tx_complete_us: us_ceil(arm + entry),
            per_frame_us: us_ceil(rest + 2 * entry),
        }
    }

    /// The CPU this cost charges one replied frame, HCLK ticks.
    pub fn frame_ticks(&self) -> u64 {
        (self.on_break_us as u64
            + Frame::REPLY_ARMS as u64 * self.on_tx_complete_us as u64
            + self.per_frame_us as u64)
            * TICKS_PER_US
    }
}

fn us_ceil(ticks: u32) -> u32 {
    ticks.div_ceil(budget::TICKS_PER_US)
}

/// The transport vectors.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Vector {
    Compare,
    Break,
    TxDone,
}

/// The kernel tick's PFIC level against the bus vectors.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum KernelLevel {
    /// Bus bodies preempt the kernel; a scan that pends behind them merges
    /// with the next one.
    BelowBus,
    /// The kernel preempts bus bodies and stretches them by its own body.
    AboveBus,
}

/// What the host asks of the servo; the lane charges the drive arm it
/// leads to.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Motion {
    /// Torque off or parked: Open.
    Quiet,
    /// Closed for [`budget::HOLD_APPROACH_TICKS`] after the lane starts or
    /// the goal moves, then Open.
    Hold,
    /// Stepping, tracking or an OpenLoop drive, which the lane cannot tell
    /// apart: per phase the dearer of Closed and OpenLoop.
    Moving,
}

/// A periodic preemptor: one kernel body per scan, phases in turn.
#[derive(Copy, Clone)]
pub struct KernelLane {
    pub level: KernelLevel,
    pub motion: Motion,
    /// Body per phase by regime, HCLK ticks.
    pub phases: [&'static [u64]; REGIMES],
    /// Each body drawn uniformly from this floor up to its phase cost,
    /// seeded; `None` runs every body at its phase cost.
    pub floor: Option<([&'static [u64]; REGIMES], u64)>,
    pub tel: TelCosts,
    pub events: KernelEvents,
}

/// The TEL burst a kernel lane paces, HCLK ticks, on top of the body: a
/// row, a row 0's head and a bank, and the TX lane the kernel owns while
/// the burst streams - its poll on every tick, each CRC feed, DMA arm and
/// patch where it runs.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct TelCosts {
    pub row: u64,
    pub head: u64,
    pub bank: u64,
    pub poll: u64,
    pub feed: u64,
    pub arm: u64,
    pub patch: u64,
}

/// Event bodies on top of the tick that carries them, HCLK ticks: the SLOW
/// block's due stage on phase SLOW every DECIM_SLOW periods (by event row) and
/// its finish on the next period's, each holding the TX lane as the kernel
/// does, and the tick-load close where the chip's count lands it
/// ([`kernel::health_close_at`], the lane's first tick the boot tick), with
/// the catch-up when a tick was lost since the last one. A configuration
/// take runs on phase SPARE_A when the servo's count is even and new; it
/// holds no lane. The drive direction check's stage runs on every SPARE_B,
/// as an unproven servo's could, and holds no lane.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct KernelEvents {
    pub refresh: u64,
    pub slow_due: [u64; EVENT_REGIMES],
    pub finish: u64,
    pub drive: u64,
    pub health_close: u64,
    pub catch_up: u64,
}

impl KernelLane {
    /// The kernel at budget for `motion`: every body and event at its
    /// maximum.
    pub fn at_budget(level: KernelLevel, motion: Motion) -> Self {
        Self {
            level,
            motion,
            phases: KERNEL_AT_BUDGET.each_ref().map(|p| &p[..]),
            floor: None,
            tel: TelCosts {
                row: budget::TEL_ROW as u64,
                head: budget::TEL_HEAD as u64,
                bank: budget::TEL_BANK as u64,
                poll: budget::TEL_POLL as u64,
                feed: budget::TEL_FEED as u64,
                arm: budget::TEL_ARM as u64,
                patch: budget::TEL_PATCH as u64,
            },
            events: KernelEvents {
                refresh: budget::REFRESH as u64,
                slow_due: budget::SLOW_DUE.map(u64::from),
                finish: budget::SLOW_FINISH as u64,
                drive: budget::DRIVE_STAGE as u64,
                health_close: budget::HEALTH_CLOSE as u64,
                catch_up: budget::CATCH_UP as u64,
            },
        }
    }

    /// The longest `bus` ticks of bus work can take on the wall under this
    /// lane above the bus: every scan in the span preempts it, the first
    /// with the longest body (any phase with its events and a close), so
    /// `(bus + longest) / (1 - U)`, U the share of a medium period of the
    /// lane's heavier regime at each phase, a close in every period (TEL
    /// off).
    pub fn wall(&self, bus: u64) -> u64 {
        let n = self.phases[Regime::Open as usize].len();
        let body = |p: usize| {
            self.phases[self.heavier(p) as usize]
                .get(p)
                .copied()
                .unwrap_or(0)
        };
        let ev = &self.events;
        let busy = (0..n).map(body).sum::<u64>() + ev.health_close;
        let span = n as u64 * KERNEL_PERIOD;
        let at = |p: u8, extra: u64| body(p as usize) + extra;
        let slow_due = ev.slow_due[self.heavier(phase::SLOW as usize).event_row() as usize];
        let events = at(phase::SLOW, slow_due.max(ev.finish))
            .max(at(phase::SPARE_A, ev.refresh))
            .max(at(phase::SPARE_B, ev.drive));
        let longest = (0..n).map(body).max().unwrap_or(0).max(events);
        let longest = longest + ev.health_close + ev.catch_up;
        ((bus + longest) * span).div_ceil(span.saturating_sub(busy).max(1))
    }

    /// The regime `motion` charges at phase `p`, a hold before it parks.
    fn heavier(&self, p: usize) -> Regime {
        let at = |r: Regime| self.phases[r as usize].get(p).copied().unwrap_or(0);
        match self.motion {
            Motion::Quiet => Regime::Open,
            Motion::Hold => Regime::Closed,
            Motion::Moving if at(Regime::OpenLoop) > at(Regime::Closed) => Regime::OpenLoop,
            Motion::Moving => Regime::Closed,
        }
    }

    /// The kernel inside its budgets for `motion`: each body drawn below its
    /// maximum, at the mean budget on average, the TEL row at its mean and
    /// the lane, which has no mean row, at its budget.
    pub fn typical(level: KernelLevel, motion: Motion, seed: u64) -> Self {
        let at = Self::at_budget(level, motion);
        Self {
            floor: Some((KERNEL_FLOOR.each_ref().map(|p| &p[..]), seed)),
            tel: TelCosts {
                row: budget::TEL_ROW_MEAN as u64,
                ..at.tel
            },
            ..at
        }
    }
}

/// What the kernel lane saw, HCLK ticks where timed.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct KernelStats {
    pub scans: u64,
    pub entries: u64,
    /// Scans whose pend merged with an earlier one not yet entered.
    pub lost: u64,
    /// Scan to kernel entry.
    pub entry_latency_max: u64,
    pub entry_latency_sum: u64,
    /// Ticks the kernel added to the bus bodies it preempted.
    pub bus_stretch: u64,
    /// Bodies longer than one period, the chip's `tick_over_count`.
    pub over: u64,
    /// Ticks charged on the Closed rows.
    pub closed: u64,
    /// TEL ticks that ran a SLOW stage, and of those the
    /// ones the TX lane took a step on.
    pub stage_tel: u64,
    pub stage_steps: u64,
    /// Tick-load closes by phase.
    pub closes: [u64; PHASES],
    /// Configuration takes, and the most ticks from a committed set to the
    /// tick that took it.
    pub takes: u64,
    pub apply_max: u64,
    /// TEL rows copied on a take's tick, ticks a burst's start was held,
    /// and ticks whose row charge and copy disagree (the sim's own check).
    pub take_rows: u64,
    pub held_starts: u64,
    pub row_misses: u64,
    /// gamma(k) of the bodies, entry and exit excluded (probe scope).
    pub windows: Windows,
}

struct Kernel {
    lane: KernelLane,
    rng: u64,
    /// No scan after this tick.
    until: u64,
    phase: usize,
    periods: u64,
    /// The goal last seen, and the entry a hold's approach began at.
    goal: Option<i32>,
    goal_at: u64,
    lost_at_close: u64,
    regime: Regime,
    /// The running body runs a SLOW stage.
    stage: bool,
    /// The servo's configuration count as the next tick reads it, the one
    /// the last take saw, and the tick a set not yet taken was committed at.
    config_seq: u8,
    config_taken: u8,
    config_new_at: Option<u64>,
    /// The running body holds a burst's start (its phase is the take's, or
    /// a take is pending), and ran the take.
    hold_start: bool,
    took: bool,
    /// The running body, extensions included.
    body: u64,
    /// The scan pended and not yet entered.
    pend_at: Option<u64>,
    entered: u64,
    run_until: u64,
    retry_at: Option<u64>,
    stats: KernelStats,
}

#[derive(Default)]
pub struct Cpu {
    pub cost: HandlerCost,
    busy_until: u64,
    pend_compare: bool,
    pend_break: bool,
    pend_tx: bool,
    /// A preemption armed for the next bus body (`Sim::preempt_before_ring_read`).
    pub preempt: Option<u64>,
    /// A preempted body waiting out its preemption, and when it entered.
    pub deferred: Option<(Vector, u64)>,
    kernel: Option<Kernel>,
    /// The running bus body preempted a kernel body below it.
    kernel_preempted: bool,
    /// A `CpuFree` wake is in flight; at most one outstanding per servo.
    pub free_scheduled: bool,
    delivered_breaks: u64,
    entries: Entries,
    frames_max: u64,
    /// CPU the running bus body charged, HCLK ticks.
    body_ticks: u64,
    /// The chip's bus probe, fed the charged bodies.
    pub probe: BusProbe,
}

/// Handler bodies run, per vector.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Entries {
    pub compare: u64,
    pub break_wake: u64,
    pub tx_done: u64,
}

impl Entries {
    pub fn total(&self) -> u64 {
        self.compare + self.break_wake + self.tx_done
    }
}

impl Cpu {
    /// No bus vector can enter now.
    pub fn busy(&self, now: u64) -> bool {
        now < self.busy_until()
    }

    pub fn busy_until(&self) -> u64 {
        match &self.kernel {
            Some(k) if k.lane.level == KernelLevel::AboveBus => self.busy_until.max(k.run_until),
            _ => self.busy_until,
        }
    }

    pub fn pend(&mut self, v: Vector) {
        match v {
            Vector::Compare => self.pend_compare = true,
            Vector::Break => self.pend_break = true,
            Vector::TxDone => self.pend_tx = true,
        }
    }

    pub fn any_pend(&self) -> bool {
        self.pend_compare || self.pend_break || self.pend_tx || self.deferred.is_some()
    }

    /// Pop the highest-arbitration pended vector, if any. With the kernel
    /// on top, the TX arm and the break wake (LOW 0x80) go ahead of the
    /// deadline mux (LOW 0xC0), so a pending wake suspends a reclaim or
    /// kills a stale reply before a pending trigger acts.
    pub fn take_pend(&mut self) -> Option<Vector> {
        let below = matches!(&self.kernel, Some(k) if k.lane.level == KernelLevel::BelowBus);
        if !below {
            if self.pend_tx {
                self.pend_tx = false;
                return Some(Vector::TxDone);
            }
            if self.pend_break {
                self.pend_break = false;
                return Some(Vector::Break);
            }
        }
        if self.pend_compare {
            self.pend_compare = false;
            Some(Vector::Compare)
        } else if self.pend_break {
            self.pend_break = false;
            Some(Vector::Break)
        } else if self.pend_tx {
            self.pend_tx = false;
            Some(Vector::TxDone)
        } else {
            None
        }
    }

    /// Occupy the CPU with `v`'s body starting at `now`.
    pub fn charge(&mut self, now: u64, v: Vector) {
        let us = match v {
            Vector::Compare => {
                self.entries.compare += 1;
                self.cost.on_deadline_us
            }
            Vector::Break => {
                self.delivered_breaks += 1;
                self.entries.break_wake += 1;
                self.cost.on_break_us
            }
            Vector::TxDone => {
                self.entries.tx_done += 1;
                self.cost.on_tx_complete_us
            }
        };
        self.occupy(now, us as u64 * TICKS_PER_US);
    }

    /// The body just run dispatched `frames` own frames: extend its
    /// occupancy by their share.
    pub fn charge_frames(&mut self, frames: u64) {
        self.extend(frames * self.cost.per_frame_us as u64 * TICKS_PER_US);
        self.frames_max = self.frames_max.max(frames);
    }

    /// Hold `v` behind a preemption that started at its entry.
    pub fn defer(&mut self, now: u64, v: Vector, ticks: u64) {
        self.occupy(now, ticks);
        self.deferred = Some((v, now));
    }

    /// Book the bus body just run on the probe, as the chip stamps it.
    pub fn book(&mut self, v: Vector, dispatched: bool) {
        let p = &mut self.probe;
        if v == Vector::Break {
            p.mark_break();
        }
        if dispatched {
            p.mark_dispatch();
        }
        let body = match v {
            Vector::Compare => Body::Deadline,
            Vector::Break => Body::BreakWake,
            Vector::TxDone => Body::TxDone,
        };
        // Booked at entry, before any kernel preempts it: wall = net.
        let ticks = self.body_ticks as u32;
        p.body(ticks, ticks, 0, body);
    }

    fn occupy(&mut self, now: u64, ticks: u64) {
        self.busy_until = now + ticks;
        self.body_ticks = ticks;
        self.kernel_preempted = false;
        if let Some(k) = self.kernel.as_mut()
            && k.lane.level == KernelLevel::BelowBus
            && now < k.run_until
        {
            self.kernel_preempted = true;
            k.run_until += ticks;
        }
    }

    fn extend(&mut self, ticks: u64) {
        self.busy_until += ticks;
        self.body_ticks += ticks;
        if self.kernel_preempted
            && let Some(k) = self.kernel.as_mut()
        {
            k.run_until += ticks;
        }
    }

    pub fn start_kernel(&mut self, lane: KernelLane, until: u64) {
        let rng = lane.floor.map_or(0, |(_, seed)| seed);
        self.kernel = Some(Kernel {
            lane,
            rng: rng.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            until,
            phase: 0,
            periods: 0,
            goal: None,
            goal_at: 0,
            lost_at_close: 0,
            regime: Regime::Open,
            stage: false,
            config_seq: 0,
            config_taken: 0,
            config_new_at: None,
            hold_start: false,
            took: false,
            body: 0,
            pend_at: None,
            entered: 0,
            run_until: 0,
            retry_at: None,
            stats: KernelStats::default(),
        });
    }

    /// A scan completed at `now`: pend the kernel, or merge into the pend
    /// still waiting. Returns the next scan's tick while the lane runs.
    pub fn kernel_scan(&mut self, now: u64) -> Option<u64> {
        let k = self.kernel.as_mut()?;
        k.stats.scans += 1;
        if k.pend_at.is_some() {
            k.stats.lost += 1;
        } else {
            k.pend_at = Some(now);
        }
        let next = now + KERNEL_PERIOD;
        (next <= k.until).then_some(next)
    }

    /// Enter the pended kernel if its level lets it, else return the tick
    /// to retry at when no earlier retry is outstanding.
    pub fn enter_kernel(&mut self, now: u64) -> Option<u64> {
        let bus_until = self.busy_until;
        let bus_held = now < bus_until || self.any_pend();
        let k = self.kernel.as_mut()?;
        let pend_at = k.pend_at?;
        let blocked = if now < k.run_until {
            Some(k.run_until)
        } else if k.lane.level == KernelLevel::BelowBus && bus_held {
            Some(bus_until.max(now))
        } else {
            None
        };
        if let Some(at) = blocked {
            if k.retry_at.is_some_and(|r| r <= at) {
                return None;
            }
            k.retry_at = Some(at);
            return Some(at);
        }
        k.pend_at = None;
        if k.run_until - k.entered > KERNEL_PERIOD {
            k.stats.over += 1;
        }
        let tick = k.stats.entries;
        if tick > 0 {
            let entry_exit = budget::ENTRY_EXIT as u64;
            k.stats
                .windows
                .add(k.body.saturating_sub(entry_exit) as u32);
        }
        k.entered = now;
        let latency = now - pend_at;
        k.stats.entries += 1;
        k.stats.entry_latency_max = k.stats.entry_latency_max.max(latency);
        k.stats.entry_latency_sum += latency;
        let approach = budget::HOLD_APPROACH_TICKS as u64;
        k.regime = match k.lane.motion {
            Motion::Hold if tick - k.goal_at >= approach => Regime::Open,
            _ => k.lane.heavier(k.phase),
        };
        k.stats.closed += (k.regime == Regime::Closed) as u64;
        let r = k.regime as usize;
        let mut cost = k.lane.phases[r][k.phase];
        if let Some((floor, _)) = k.lane.floor {
            k.rng ^= k.rng << 13;
            k.rng ^= k.rng >> 7;
            k.rng ^= k.rng << 17;
            let lo = floor[r][k.phase].min(cost);
            cost = lo + k.rng % (cost - lo + 1);
        }
        let ev = k.lane.events;
        let at = k.periods % DECIM_SLOW as u64;
        let spare_a = k.phase == phase::SPARE_A as usize;
        let take = spare_a && k.config_seq & 1 == 0 && k.config_seq != k.config_taken;
        k.took = take;
        let stage = match k.phase as u8 {
            phase::SLOW if at == 0 => Some(ev.slow_due[k.regime.event_row() as usize]),
            phase::SLOW if at == 1 => Some(ev.finish),
            _ => None,
        };
        k.stage = stage.is_some();
        cost += stage.unwrap_or(0);
        if k.phase == phase::SPARE_B as usize {
            cost += ev.drive;
        }
        if take {
            cost += ev.refresh;
            k.config_taken = k.config_seq;
            k.stats.takes += 1;
            if let Some(from) = k.config_new_at.take() {
                k.stats.apply_max = k.stats.apply_max.max(tick - from);
            }
        }
        // the kernel's start rule, asked after the take
        k.hold_start = spare_a || k.config_seq != k.config_taken;
        if kernel::health_close_at(tick) {
            k.stats.closes[k.phase] += 1;
            cost += ev.health_close;
            if k.stats.lost > k.lost_at_close {
                cost += ev.catch_up;
            }
            k.lost_at_close = k.stats.lost;
        }
        k.phase += 1;
        if k.phase == k.lane.phases[r].len() {
            k.phase = 0;
            k.periods += 1;
        }
        k.body = cost;
        k.run_until = now + cost;
        if k.lane.level == KernelLevel::AboveBus && now < bus_until {
            self.busy_until += cost;
            k.stats.bus_stretch += cost;
        }
        None
    }

    /// The running or last kernel body's end.
    pub fn kernel_until(&self) -> Option<u64> {
        self.kernel.as_ref().map(|k| k.run_until)
    }

    /// The lane's TEL costs (zero without a lane).
    pub fn kernel_tel(&self) -> TelCosts {
        self.kernel.as_ref().map(|k| k.lane.tel).unwrap_or_default()
    }

    /// The running kernel body runs a SLOW stage: the TX lane holds.
    pub fn kernel_holds(&self) -> bool {
        self.kernel.as_ref().is_some_and(|k| k.stage)
    }

    /// The running body holds a burst not yet started: its phase is the
    /// take's, or a configuration take is pending.
    pub fn kernel_hold_start(&self) -> bool {
        self.kernel.as_ref().is_some_and(|k| k.hold_start)
    }

    /// The running body's TEL tick: whether a row was charged at entry,
    /// whether the feed copied one, and whether a burst start was held.
    pub fn kernel_row(&mut self, charged: bool, copied: bool, held: bool) {
        if let Some(k) = self.kernel.as_mut() {
            k.stats.take_rows += (k.took && copied) as u64;
            k.stats.row_misses += (charged != copied) as u64;
            k.stats.held_starts += held as u64;
        }
    }

    /// The running body's TEL tick: whether its TX lane took a step.
    pub fn kernel_lane_step(&mut self, stepped: bool) {
        if let Some(k) = self.kernel.as_mut()
            && k.stage
        {
            k.stats.stage_tel += 1;
            k.stats.stage_steps += stepped as u64;
        }
    }

    /// The goal register as the next kernel tick reads it: a move restarts
    /// a hold's approach.
    pub fn kernel_goal(&mut self, goal: i32) {
        if let Some(k) = self.kernel.as_mut()
            && k.goal.replace(goal).is_some_and(|g| g != goal)
        {
            k.goal_at = k.stats.entries;
        }
    }

    /// The servo's configuration count as the next tick reads it: a new
    /// even value is a committed set, timed from the tick it lands before.
    pub fn kernel_config(&mut self, seq: u8) {
        if let Some(k) = self.kernel.as_mut()
            && seq != k.config_seq
        {
            k.config_seq = seq;
            if seq & 1 == 0 && k.config_new_at.is_none() {
                k.config_new_at = Some(k.stats.entries);
            }
        }
    }

    /// Lengthen the kernel body ending at `now` by `ticks` of tail work.
    pub fn extend_kernel(&mut self, now: u64, ticks: u64) {
        let Some(k) = self.kernel.as_mut() else {
            return;
        };
        k.body += ticks;
        k.run_until += ticks;
        if k.lane.level == KernelLevel::AboveBus && now < self.busy_until {
            self.busy_until += ticks;
            k.stats.bus_stretch += ticks;
        }
    }

    /// A retry event popped: true when it is the outstanding one.
    pub fn take_kernel_retry(&mut self, now: u64) -> bool {
        match self.kernel.as_mut() {
            Some(k) if k.retry_at == Some(now) => {
                k.retry_at = None;
                true
            }
            _ => false,
        }
    }

    pub fn kernel_stats(&self) -> KernelStats {
        self.kernel.as_ref().map(|k| k.stats).unwrap_or_default()
    }

    /// `on_break` invocations actually delivered -- the coalescing observable
    /// (wire FE events minus this = pends that merged).
    pub fn delivered_breaks(&self) -> u64 {
        self.delivered_breaks
    }

    /// The most own frames one body dispatched -- how deep the ladder ran
    /// behind the wire.
    pub fn frames_max(&self) -> u64 {
        self.frames_max
    }

    pub fn entries(&self) -> Entries {
        self.entries
    }
}

#[cfg(test)]
mod tests {
    use osc_servo_core::budget::Events;
    use osc_servo_core::kernel::DECIM_MED;
    use osc_servo_core::{Mode, RegionStorage, SensorFrame, Shared};

    use super::*;
    use crate::plant::{BIAS, kernel, seed, vbus_raw};

    /// One bit per stage: a body's cost names the stages the lane charged.
    const COSTS: [u64; 4] = [1, 2, 4, 8];
    static FREE: [u64; PHASES] = [0; PHASES];

    fn write_config(sh: &Shared, f: impl FnOnce(&mut osc_servo_core::ControlTable)) {
        sh.config_commit_begin();
        sh.table.with_mut(f);
        sh.config_commit_end();
    }

    /// The core kernel tests' hot seat: the winding estimate runs on it.
    fn hot_seat(sh: &Shared) -> SensorFrame {
        write_config(sh, |t| {
            t.calib.thermal.th_rise_q016 = 1044;
            t.calib.thermal.ntc_raw_ref = 2048;
            t.calib.thermal.ntc_t_ref_cc = 2500;
            t.calib.thermal.ntc_k1_q88 = 563;
            t.calib.thermal.ntc_k2_q24 = 5800;
            t.control.lifecycle.torque_enable = true;
            t.control.lifecycle.mode = Mode::OpenLoop;
            t.control.lifecycle.goal_duty = 8000;
        });
        SensorFrame {
            pos: 2000,
            current: BIAS + 400,
            current_trough: BIAS,
            vmotor_a: 3000,
            vmotor_a_trough: 3000,
            vmotor_b: 40,
            vmotor_b_trough: 40,
            vcal: 1200,
            vbus_raw: vbus_raw(3000),
            ntc_raw: 2048,
        }
    }

    /// The lane's next body, reading the servo's configuration count `seq`.
    fn lane_tick(cpu: &mut Cpu, now: &mut u64, seq: u8) -> u64 {
        cpu.kernel_config(seq);
        cpu.kernel_scan(*now);
        cpu.enter_kernel(*now);
        let body = cpu.kernel_until().unwrap_or(*now) - *now;
        *now += KERNEL_PERIOD;
        body
    }

    /// The lane charges each stage on the tick the kernel runs it: every
    /// stage `Events::between` reads off a real kernel's tags (due, finish,
    /// configuration take, the drive direction check's)
    /// lands on a tick the lane charges it, and the SLOW block's due and
    /// finish on exactly the lane's.
    #[test]
    fn the_lane_charges_each_stage_on_the_kernels_tick() {
        let sh = Shared::new();
        seed(&sh);
        let f = hot_seat(&sh);
        let mut k = kernel();
        while !k.probe_tags().slow_due {
            k.on_scan(f, &sh);
        }
        let [slow, finish, refresh, drive] = COSTS;
        let mut cpu = Cpu::default();
        cpu.start_kernel(
            KernelLane {
                level: KernelLevel::AboveBus,
                motion: Motion::Quiet,
                phases: [&FREE; REGIMES],
                floor: None,
                tel: TelCosts::default(),
                events: KernelEvents {
                    refresh,
                    slow_due: [slow; EVENT_REGIMES],
                    finish,
                    drive,
                    health_close: 0,
                    catch_up: 0,
                },
            },
            u64::MAX,
        );
        let mut now = 0;
        for _ in 0..phase::SLOW {
            lane_tick(&mut cpu, &mut now, sh.config_seq());
        }
        let ticks = 400 * DECIM_SLOW as u32 * DECIM_MED as u32;
        let mut seen = [0u32; 4];
        for t in 0..ticks {
            if t == ticks / 2 {
                write_config(&sh, |_| {});
            }
            let body = lane_tick(&mut cpu, &mut now, sh.config_seq());
            let before = k.probe_tags();
            k.on_scan(f, &sh);
            let ev = Events::between(before, k.probe_tags());
            let ran = [ev.slow, ev.finish, ev.refresh, ev.drive];
            let charged = COSTS.map(|c| body & c != 0);
            assert_eq!(ran[..2], charged[..2], "tick {t}");
            assert!(
                !(charged[2] && charged[3]),
                "tick {t}: the drive stage stacked, {charged:?}"
            );
            for (n, (&r, &c)) in seen.iter_mut().zip(ran.iter().zip(&charged)) {
                assert!(!r || c, "tick {t}: ran {ran:?}, charged {charged:?}");
                *n += r as u32;
            }
        }
        assert!(seen.iter().all(|&n| n > 0), "{seen:?}");
    }

    /// An OpenLoop drive books on the OpenLoop rows: past the boot tick,
    /// every tick of a running OpenLoop drive.
    #[test]
    fn an_openloop_drive_books_on_the_openloop_rows() {
        use osc_servo_core::budget::probe::KernelProbe;
        let sh = Shared::new();
        seed(&sh);
        let f = hot_seat(&sh);
        let mut k = kernel();
        let mut probe = KernelProbe::ZERO;
        let mut booked = [0u32; REGIMES];
        let ticks = 4 * DECIM_SLOW as u32 * DECIM_MED as u32;
        for t in 0..ticks {
            k.on_scan(f, &sh);
            let r = probe.tick(600 + t % 100, || k.probe_tags(), false, 0, false);
            if let Some(r) = r {
                booked[r as usize] += 1;
            }
        }
        assert_eq!(booked, [0, 0, ticks - 1]);
    }

    /// A moving lane cannot tell stepping from an OpenLoop drive: at budget
    /// it charges each phase the dearer of the Closed and OpenLoop rows.
    #[test]
    fn a_moving_lane_charges_the_dearer_driven_row_at_each_phase() {
        let lane = KernelLane {
            events: KernelEvents::default(),
            ..KernelLane::at_budget(KernelLevel::AboveBus, Motion::Moving)
        };
        let mut cpu = Cpu::default();
        cpu.start_kernel(lane, u64::MAX);
        let mut now = 0;
        let [closed, openloop] =
            [Regime::Closed, Regime::OpenLoop].map(|r| KERNEL_AT_BUDGET[r as usize]);
        for p in 0..PHASES {
            let body = lane_tick(&mut cpu, &mut now, 0);
            assert_eq!(body, closed[p].max(openloop[p]), "phase {p}");
        }
    }
}

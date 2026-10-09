//! Per-servo PFIC occupancy model. The transport vectors share PFIC HIGH on
//! the chip, so a handler body occupies the CPU and every event landing
//! meanwhile *pends* -- a flag per vector, not a queue -- and a burst of same-
//! vector events coalesces into one late delivery, exactly as pended IRQs do
//! on silicon. Ring bytes are DMA and always land at their wire tick; only
//! handler invocations defer. Handler effects land at entry: the model
//! charges occupancy, not intra-body effect timing.
//!
//! An optional kernel lane models the ADC scan tick as a periodic preemptor
//! at a selectable level against the bus vectors.

use super::core::TICKS_PER_US;

/// Kernel tick period: one ADC scan per 20 kHz PWM period, HCLK ticks.
pub const KERNEL_PERIOD: u64 = 50 * TICKS_PER_US;

/// Kernel body per phase (CONTROL, OBSERVER, TRAJECTORY, LIMITS, VELOCITY,
/// RAIL, SLOW, PUBLISH, two free), HCLK ticks: cpu-probe v2 LOW-exclusive
/// means over three 20 s windows, torque off, no traffic.
pub const KERNEL_QUIET: [u64; 10] = [981, 1250, 732, 1087, 768, 1033, 763, 943, 702, 712];

/// The same instrument holding at centre.
pub const KERNEL_HOLD: [u64; 10] = [1023, 1250, 1239, 1096, 792, 1072, 763, 943, 702, 712];

/// An own 32 B READ at 3M as the chip's HIGH bodies cost it with the kernel
/// below the bus: cpu-probe v1 over the polling ladder: TIM2 16.0 us, SysTick 82.1 us in two bodies, USART1
/// 20.8 us in three TCs per frame. The sim spends the SysTick share over
/// three compare bodies (header, dispatch, trigger): 3 x 20 + 22.
pub const READ32_3M_COST: HandlerCost = HandlerCost {
    on_break_us: 16,
    on_deadline_us: 20,
    on_tx_complete_us: 7,
    per_frame_us: 22,
};

/// Sim-time cost of each `ServoBus` handler body, us. Zero (the default)
/// delivers every event at its wire tick -- the ideal-CPU model the logical
/// suites pin.
#[derive(Copy, Clone, Default)]
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

/// The transport vectors, in same-priority arbitration order (lowest
/// interrupt number delivers first: SysTick, then USART1 -- whose real body
/// drains RX errors before TC).
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

/// A periodic preemptor: one kernel body per scan, phases in turn.
#[derive(Copy, Clone)]
pub struct KernelLane {
    pub level: KernelLevel,
    /// Body per phase, HCLK ticks.
    pub phases: &'static [u64],
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
}

struct Kernel {
    lane: KernelLane,
    /// No scan after this tick.
    until: u64,
    phase: usize,
    /// The scan pended and not yet entered.
    pend_at: Option<u64>,
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

    /// Pop the highest-arbitration pended vector, if any.
    pub fn take_pend(&mut self) -> Option<Vector> {
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

    fn occupy(&mut self, now: u64, ticks: u64) {
        self.busy_until = now + ticks;
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
        if self.kernel_preempted
            && let Some(k) = self.kernel.as_mut()
        {
            k.run_until += ticks;
        }
    }

    pub fn start_kernel(&mut self, lane: KernelLane, until: u64) {
        self.kernel = Some(Kernel {
            lane,
            until,
            phase: 0,
            pend_at: None,
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
        let latency = now - pend_at;
        k.stats.entries += 1;
        k.stats.entry_latency_max = k.stats.entry_latency_max.max(latency);
        k.stats.entry_latency_sum += latency;
        let cost = k.lane.phases[k.phase];
        k.phase = (k.phase + 1) % k.lane.phases.len();
        k.run_until = now + cost;
        if k.lane.level == KernelLevel::AboveBus && now < bus_until {
            self.busy_until += cost;
            k.stats.bus_stretch += cost;
        }
        None
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

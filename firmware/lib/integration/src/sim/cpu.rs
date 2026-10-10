//! Per-servo PFIC occupancy model. The transport vectors share one PFIC
//! level on the chip, so a handler body occupies the CPU and every event landing
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

/// The same instrument under the chainload sine (N=1, +/-300 at 100 Hz): the
/// closed current loop runs on every tick.
pub const KERNEL_MOVING: [u64; 10] = [1644, 1889, 1876, 1710, 1732, 1700, 1375, 1556, 1313, 1322];

/// The six-field TEL step soak's kernel (Position hold, a 1200-count step
/// before each burst), HCLK ticks: [`KERNEL_MOVING`] plus 4 us per body,
/// each body spread over +/- [`KERNEL_STEP_SPREAD_US`], with
/// [`TEL_ENCODE_COST`] on top while the burst runs. Fitted to the bench
/// soak on the image with the two-arm stager: 19.6% of rows dropped and
/// 5.3-5.5% of ticks over one period, against 16.7-18.3% and 4.9-5.9% here
/// over eight seeds (DES pin `kernel_step_lane_overruns_as_the_step_soak`).
pub const KERNEL_STEP: [u64; 10] = {
    let mut p = KERNEL_MOVING;
    let mut i = 0;
    while i < p.len() {
        p[i] += 4 * TICKS_PER_US;
        i += 1;
    }
    p
};

pub const KERNEL_STEP_SPREAD_US: u64 = 4;

/// A frame start in a bus body: the CPU from the body's entry to its law
/// break, HCLK ticks. The stager vector (SW) seals the frame and triggers
/// (E: entry, the CRC read and the break setup, about 90 instructions; the
/// two-arm stager before it took 18.5 us, its 23.3 us stage-through-trigger,
/// cpu-probe v1, less [`TEL_TRIGGER_COST`]).
pub const TEL_STAGE_PRE: u64 = 120;

/// A TX completion that starts the next frame: its entry, the release and
/// the checks ahead of the break, HCLK ticks (E: about 100 instructions).
pub const TEL_TC_PRE: u64 = 128;

/// The trigger from its break on: the law-break spin (11 bit-times, 3.7 us
/// at 3M) and the DMA arm, HCLK ticks (E).
pub const TEL_TRIGGER_COST: u64 = 228;

/// The TEL stager vector when it starts no frame, HCLK ticks: entry, the
/// bus-state checks, exit (E: about 40 instructions, 1 us).
pub const TEL_POLL_COST: u64 = 48;

/// The kernel's per-tick TEL work while a burst runs, HCLK ticks: the
/// sample build and the encoder's append (E: about 190 instructions in the
/// kernel vector, 5 us).
pub const TEL_ENCODE_COST: u64 = 240;

/// An own 32 B READ at 3M as the chip's bus bodies cost it with the kernel
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

/// The transport vectors.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Vector {
    Compare,
    Break,
    TxDone,
    /// The TEL stager (SW), pended by every kernel tick of a burst.
    Stage,
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
    /// Bodies longer than one period, the chip's `tick_over_count`.
    pub over: u64,
}

struct Kernel {
    lane: KernelLane,
    /// Per-body spread: uniform in +/- this many ticks, and the generator.
    spread: u64,
    rng: u64,
    /// No scan after this tick.
    until: u64,
    phase: usize,
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
    pend_stage: bool,
    /// The running body holds a frame start (`TxGate`) to its break; the
    /// CPU it still owes after the break.
    pub tx_held: Option<u64>,
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
    pub stage: u64,
}

impl Entries {
    pub fn total(&self) -> u64 {
        self.compare + self.break_wake + self.tx_done + self.stage
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
            Vector::Stage => self.pend_stage = true,
        }
    }

    pub fn any_pend(&self) -> bool {
        self.pend_compare
            || self.pend_break
            || self.pend_tx
            || self.pend_stage
            || self.deferred.is_some()
            || self.tx_held.is_some()
    }

    /// Pop the highest-arbitration pended vector, if any. With the kernel
    /// on top, the TX arm and the break wake (LOW 0x80) go ahead of the
    /// deadline mux and the TEL stager (LOW 0xC0, SysTick's lower IRQ
    /// number first), so a pending wake suspends a reclaim or kills a stale
    /// reply before a pending trigger acts.
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
        } else if self.pend_stage {
            self.pend_stage = false;
            Some(Vector::Stage)
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
            Vector::Stage => {
                self.entries.stage += 1;
                self.occupy(now, TEL_POLL_COST);
                return;
            }
        };
        self.occupy(now, us as u64 * TICKS_PER_US);
    }

    /// The body that entered at `now` reaches its break after `pre` ticks
    /// of CPU and owes `post` more after it.
    pub fn hold_to_break(&mut self, now: u64, pre: u64, post: u64) {
        self.occupy(now, pre);
        self.tx_held = Some(post);
    }

    /// The held break went out at `now`: the body runs on for what it owes.
    pub fn resume_after_break(&mut self, now: u64) {
        if let Some(post) = self.tx_held.take() {
            self.occupy(now, post);
        }
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
            spread: 0,
            rng: 0,
            until,
            phase: 0,
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
        k.entered = now;
        let latency = now - pend_at;
        k.stats.entries += 1;
        k.stats.entry_latency_max = k.stats.entry_latency_max.max(latency);
        k.stats.entry_latency_sum += latency;
        let mut cost = k.lane.phases[k.phase];
        if k.spread != 0 {
            k.rng ^= k.rng << 13;
            k.rng ^= k.rng >> 7;
            k.rng ^= k.rng << 17;
            cost = (cost + k.rng % (2 * k.spread + 1)).saturating_sub(k.spread);
        }
        k.phase = (k.phase + 1) % k.lane.phases.len();
        k.run_until = now + cost;
        if k.lane.level == KernelLevel::AboveBus && now < bus_until {
            self.busy_until += cost;
            k.stats.bus_stretch += cost;
        }
        None
    }

    /// Spread every later kernel body uniformly over +/- `ticks`, from a
    /// generator seeded with `seed`.
    pub fn spread_kernel(&mut self, ticks: u64, seed: u64) {
        if let Some(k) = self.kernel.as_mut() {
            k.spread = ticks;
            k.rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
        }
    }

    /// The running or last kernel body's end.
    pub fn kernel_until(&self) -> Option<u64> {
        self.kernel.as_ref().map(|k| k.run_until)
    }

    /// Lengthen the kernel body ending at `now` by `ticks` of tail work.
    pub fn extend_kernel(&mut self, now: u64, ticks: u64) {
        let Some(k) = self.kernel.as_mut() else {
            return;
        };
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

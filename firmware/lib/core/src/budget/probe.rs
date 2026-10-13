//! The budget probe's records and their check. A bench build keeps one
//! [`KernelProbe`] (written only by the kernel tick) and one [`BusProbe`]
//! (written only at the bus level); the hardware test decodes both from a
//! debug-link dump and holds them against the table.

use super::{Body, CLOCK_HZ, Events, Frame, PHASES, Regime, TICKS_PER_US, WINDOW_TICKS};
use crate::kernel::phase;
use crate::tel::STREAM_SAMPLES_MAX;

/// A body longer than this is a debugger halt or a flash save, not a
/// budgeted body: counted, kept out of every statistic.
pub const HALT_TICKS: u32 = 1000 * TICKS_PER_US;

const _: () = assert!(HALT_TICKS <= u16::MAX as u32);

/// Kernel body histogram: 5 us bins, the last one open.
pub const HIST_BIN: u32 = 5 * TICKS_PER_US;
pub const HIST_BINS: usize = 16;

/// Bodies of one component: count, total and longest, HCLK ticks.
#[repr(C)]
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Load {
    pub n: u32,
    pub sum: u32,
    pub max: u32,
}

impl Load {
    pub const ZERO: Self = Self {
        n: 0,
        sum: 0,
        max: 0,
    };

    #[inline(always)]
    pub fn add(&mut self, d: u32) {
        self.n = self.n.wrapping_add(1);
        self.sum = self.sum.wrapping_add(d);
        self.max = self.max.max(d);
    }

    /// Host-side: it divides.
    pub fn mean(&self) -> u32 {
        self.sum.checked_div(self.n).unwrap_or(0)
    }
}

/// A [`Load`] whose maximum is kept apart, narrow.
#[repr(C)]
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Tally {
    pub n: u32,
    pub sum: u32,
}

impl Tally {
    pub const ZERO: Self = Self { n: 0, sum: 0 };
}

const RING: usize = WINDOW_TICKS[WINDOW_TICKS.len() - 1];
const K: usize = WINDOW_TICKS.len();

/// gamma(k), k in [`WINDOW_TICKS`]: the most the bodies of k consecutive
/// ticks took, HCLK ticks.
#[repr(C)]
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Windows {
    ring: [u16; RING],
    at: u32,
    sum: [u32; K],
    pub gamma: [u32; K],
}

impl Windows {
    pub const ZERO: Self = Self {
        ring: [0; RING],
        at: 0,
        sum: [0; K],
        gamma: [0; K],
    };

    /// The next tick's body (a halt never reaches here: it fits u16).
    #[inline(always)]
    pub fn add(&mut self, body: u32) {
        let body = body.min(u16::MAX as u32);
        let at = self.at as usize;
        for (j, k) in WINDOW_TICKS.into_iter().enumerate() {
            let out = if at >= k { at - k } else { at + RING - k };
            let old = self.ring.get(out).copied().unwrap_or(0) as u32;
            if let (Some(s), Some(g)) = (self.sum.get_mut(j), self.gamma.get_mut(j)) {
                *s = s.wrapping_add(body).wrapping_sub(old);
                *g = (*g).max(*s);
            }
        }
        if let Some(r) = self.ring.get_mut(at) {
            *r = body as u16;
        }
        self.at = if at + 1 < RING { at as u32 + 1 } else { 0 };
    }
}

/// A tick this long counts toward [`KernelProbe::long`]: the hold windows'
/// maxima sit above it, the parked ticks below.
pub const LONG_TICKS: u32 = 30 * TICKS_PER_US;

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct KernelProbe {
    /// Ticks with no event, by regime and phase, and the longest of each:
    /// [`Self::load`].
    pub tick: [[Tally; PHASES]; 2],
    pub tick_max: [[u16; PHASES]; 2],
    /// Longest TEL tick that banked nothing, by regime and phase.
    pub tel_max: [[u16; PHASES]; 2],
    /// Longest banking tick, by regime and phase.
    pub bank_max: [[u16; PHASES]; 2],
    /// Longest tick that closed a tick-load window, by regime and phase,
    /// whatever TEL work it also carried; no TEL or plain row holds one.
    pub health_max: [[u16; PHASES]; 2],
    /// TEL ticks that banked nothing, by regime, all phases.
    pub tel: [Load; 2],
    /// Ticks that took the configuration.
    pub refresh: Load,
    /// The SLOW block's due ticks by regime, its finish ticks, and the
    /// finishes that froze a thermal reference window; the thermometer's
    /// stages on phases SPARE_A and SPARE_B.
    pub slow: [Load; 2],
    pub finish: Load,
    pub close: Load,
    pub install: Load,
    pub side: Load,
    /// Ticks of at least [`LONG_TICKS`], by regime.
    pub long: [u32; 2],
    pub windows: Windows,
    /// Entry stamp to the later of the grid slot (last entry + one period)
    /// and the last tick's end plus [`super::ENTRY_EXIT`]: the most another
    /// context held the tick off.
    pub late_max: u32,
    pub halts: u32,
    /// Every kernel vector so far, with its interrupt entry and exit: what a
    /// bus body subtracts.
    pub busy: u32,
    /// Kernel vectors entered so far: what a bus body counts as nested.
    pub entries: u32,
    last_entry: u32,
    last_end: u32,
}

impl KernelProbe {
    pub const ZERO: Self = Self {
        tick: [[Tally::ZERO; PHASES]; 2],
        tick_max: [[0; PHASES]; 2],
        tel_max: [[0; PHASES]; 2],
        bank_max: [[0; PHASES]; 2],
        health_max: [[0; PHASES]; 2],
        tel: [Load::ZERO; 2],
        refresh: Load::ZERO,
        slow: [Load::ZERO; 2],
        finish: Load::ZERO,
        close: Load::ZERO,
        install: Load::ZERO,
        side: Load::ZERO,
        long: [0; 2],
        windows: Windows::ZERO,
        late_max: 0,
        halts: 0,
        busy: 0,
        entries: 0,
        last_entry: 0,
        last_end: 0,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// One kernel tick entered at `entry`, `body` ticks long, that ran
    /// `phase` in `regime` and carried `ev`. The boot tick counts toward
    /// `busy` only.
    #[inline(always)]
    pub fn tick(&mut self, entry: u32, body: u32, phase: usize, r: Regime, ev: Events, boot: bool) {
        self.busy = self.busy.wrapping_add(body);
        let due = self.last_entry.wrapping_add(super::TICK_PERIOD);
        let free = self.last_end.wrapping_add(super::ENTRY_EXIT);
        let slot = if (free.wrapping_sub(due) as i32) > 0 {
            free
        } else {
            due
        };
        let late = entry.wrapping_sub(slot);
        self.last_entry = entry;
        if boot {
            return;
        }
        if body > HALT_TICKS {
            self.halts = self.halts.wrapping_add(1);
            return;
        }
        if late < HALT_TICKS {
            self.late_max = self.late_max.max(late);
        }
        self.windows.add(body);
        let body16 = body as u16;
        let r = r as usize;
        if body >= LONG_TICKS
            && let Some(n) = self.long.get_mut(r)
        {
            *n = n.wrapping_add(1);
        }
        if ev.refresh {
            self.refresh.add(body);
        } else if ev.close {
            self.close.add(body);
        } else if ev.slow {
            if let Some(l) = self.slow.get_mut(r) {
                l.add(body);
            }
        } else if ev.finish {
            self.finish.add(body);
        } else if ev.install {
            self.install.add(body);
        } else if ev.side {
            self.side.add(body);
        } else if ev.health {
            if let Some(m) = self.health_max.get_mut(r).and_then(|m| m.get_mut(phase)) {
                *m = (*m).max(body16);
            }
        } else if ev.bank {
            if let Some(m) = self.bank_max.get_mut(r).and_then(|m| m.get_mut(phase)) {
                *m = (*m).max(body16);
            }
        } else if ev.tel {
            if let Some(l) = self.tel.get_mut(r) {
                l.add(body);
            }
            if let Some(m) = self.tel_max.get_mut(r).and_then(|m| m.get_mut(phase)) {
                *m = (*m).max(body16);
            }
        } else if let (Some(t), Some(m)) = (
            self.tick.get_mut(r).and_then(|t| t.get_mut(phase)),
            self.tick_max.get_mut(r).and_then(|m| m.get_mut(phase)),
        ) {
            t.n = t.n.wrapping_add(1);
            t.sum = t.sum.wrapping_add(body);
            *m = (*m).max(body16);
        }
    }

    /// The plain ticks of `r` at `phase`.
    pub fn load(&self, r: Regime, phase: usize) -> Load {
        let r = r as usize;
        let t = self.tick.get(r).and_then(|t| t.get(phase));
        let max = self.tick_max.get(r).and_then(|m| m.get(phase));
        Load {
            n: t.map_or(0, |t| t.n),
            sum: t.map_or(0, |t| t.sum),
            max: max.map_or(0, |&m| m.into()),
        }
    }

    /// The tick's vector reached its last stamp at `end`, `exit` its exit
    /// stamp: the probe's own update, and the vector's entry and exit, run
    /// inside any bus body it preempted.
    #[inline(always)]
    pub fn ended(&mut self, exit: u32, end: u32) {
        self.other(end.wrapping_sub(exit));
        self.last_end = end;
    }

    /// A body on the kernel's vector that is not a tick (shunt burst events),
    /// with the vector's entry and exit.
    #[inline(always)]
    pub fn other(&mut self, body: u32) {
        self.busy = self.busy.wrapping_add(body).wrapping_add(super::ENTRY_EXIT);
        self.entries = self.entries.wrapping_add(1);
    }

    /// The regime the window's TEL and event ticks are held to: Closed once
    /// any tick drove.
    pub fn drove(&self) -> Regime {
        if self.slow[1].n != 0 || self.tick[1].iter().any(|t| t.n != 0) {
            Regime::Closed
        } else {
            Regime::Open
        }
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        Some(Self {
            tick: core::array::from_fn(|_| {
                core::array::from_fn(|_| Tally {
                    n: r.word(),
                    sum: r.word(),
                })
            }),
            tick_max: core::array::from_fn(|_| r.halves()),
            tel_max: core::array::from_fn(|_| r.halves()),
            bank_max: core::array::from_fn(|_| r.halves()),
            health_max: core::array::from_fn(|_| r.halves()),
            tel: core::array::from_fn(|_| r.load()),
            refresh: r.load(),
            slow: core::array::from_fn(|_| r.load()),
            finish: r.load(),
            close: r.load(),
            install: r.load(),
            side: r.load(),
            long: core::array::from_fn(|_| r.word()),
            windows: Windows {
                ring: r.halves(),
                at: r.word(),
                sum: core::array::from_fn(|_| r.word()),
                gamma: core::array::from_fn(|_| r.word()),
            },
            late_max: r.word(),
            halts: r.word(),
            busy: r.word(),
            entries: r.word(),
            last_entry: r.word(),
            last_end: r.word(),
        })
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct BusProbe {
    /// Bus vector bodies less the kernel time inside them, by [`Body`].
    pub body: [Load; Body::ALL.len()],
    /// The body that set each net maximum: its time with the kernel inside,
    /// and the kernel vectors it nested.
    pub wall: [u32; Body::ALL.len()],
    pub nested: [u32; Body::ALL.len()],
    /// Host frames: every bus body from one break wake to the next. A
    /// window's last frame closes at [`Self::closed`].
    pub host: Load,
    pub halts: u32,
    /// Break-wake overflows: breaks, and a parked low's re-fires.
    pub breaks: u32,
    pub refires: u32,
    /// Overflows that found the line still low, and of those, the ones whose
    /// rising edge landed between the pin read and the park.
    pub parks: u32,
    pub rises: u32,
    acc: u32,
    /// [`SAW_BREAK`] and [`SAW_DISPATCH`] for the running body.
    flags: u32,
}

const SAW_BREAK: u32 = 1 << 0;
const SAW_DISPATCH: u32 = 1 << 1;

impl BusProbe {
    pub const ZERO: Self = Self {
        body: [Load::ZERO; Body::ALL.len()],
        wall: [0; Body::ALL.len()],
        nested: [0; Body::ALL.len()],
        host: Load::ZERO,
        halts: 0,
        breaks: 0,
        refires: 0,
        parks: 0,
        rises: 0,
        acc: 0,
        flags: 0,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// The running body is a break wake that found a break.
    #[inline(always)]
    pub fn mark_break(&mut self) {
        self.flags |= SAW_BREAK;
    }

    /// The running body dispatched a frame.
    #[inline(always)]
    pub fn mark_dispatch(&mut self) {
        self.flags |= SAW_DISPATCH;
    }

    /// A bus body of `excl` ticks ends, `wall` with the `nested` kernel
    /// vectors inside it. A break wake that dispatched books as
    /// [`Body::Deadline`].
    #[inline(always)]
    pub fn body(&mut self, excl: u32, wall: u32, nested: u32, b: Body) {
        let saw = self.flags;
        self.flags = 0;
        let brk = saw & SAW_BREAK != 0;
        let dispatched = saw & SAW_DISPATCH != 0;
        if excl > HALT_TICKS {
            self.halts = self.halts.wrapping_add(1);
            return;
        }
        let b = match b {
            Body::BreakWake if dispatched => Body::Deadline,
            b => b,
        };
        let i = b as usize;
        if let (Some(l), Some(w), Some(n)) = (
            self.body.get_mut(i),
            self.wall.get_mut(i),
            self.nested.get_mut(i),
        ) {
            if excl >= l.max {
                *w = wall;
                *n = nested;
            }
            l.add(excl);
        }
        if brk {
            self.fold();
        }
        self.acc = self.acc.wrapping_add(excl);
    }

    /// The record with its open frame closed: read after the traffic, the
    /// last frame has run every body it will.
    pub fn closed(mut self) -> Self {
        self.fold();
        self
    }

    /// Close the frame accumulated so far.
    #[inline(always)]
    fn fold(&mut self) {
        if self.acc != 0 {
            self.host.add(self.acc);
        }
        self.acc = 0;
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        Some(Self {
            body: core::array::from_fn(|_| r.load()),
            wall: core::array::from_fn(|_| r.word()),
            nested: core::array::from_fn(|_| r.word()),
            host: r.load(),
            halts: r.word(),
            breaks: r.word(),
            refires: r.word(),
            parks: r.word(),
            rises: r.word(),
            acc: r.word(),
            flags: r.word(),
        })
    }
}

/// A TEL step the `bench-telcost` image times (drivers `tel`): the arm's
/// pickup; the tick's row, landing a frame head, a plain row or the bank;
/// the lane's step, none, a frame's start or a CRC feed, patch or arm.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TelStep {
    Pickup,
    Head,
    Row,
    Bank,
    Poll,
    Start,
    Service,
}

impl TelStep {
    pub const ALL: [Self; 7] = [
        Self::Pickup,
        Self::Head,
        Self::Row,
        Self::Bank,
        Self::Poll,
        Self::Start,
        Self::Service,
    ];
}

const TEL_STEPS: usize = TelStep::ALL.len();

/// TEL steps by [`TelStep`]: count, total and longest, HCLK ticks, each
/// less `cal`, a back-to-back stamp pair's cost; a step longer than
/// [`HALT_TICKS`] is a halt and books nowhere. `tick`: the running kernel
/// tick's steps, which the tick's end takes.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TelCost {
    pub n: [u32; TEL_STEPS],
    pub sum: [u32; TEL_STEPS],
    pub max: [u16; TEL_STEPS],
    pub cal: u16,
    pub tick: u32,
}

impl TelCost {
    pub const ZERO: Self = Self {
        n: [0; TEL_STEPS],
        sum: [0; TEL_STEPS],
        max: [0; TEL_STEPS],
        cal: 0,
        tick: 0,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// Step `s` took `wall` ticks between its stamps.
    #[inline(always)]
    pub fn add(&mut self, s: TelStep, wall: u32) {
        if wall > HALT_TICKS {
            return;
        }
        let d = wall.saturating_sub(self.cal.into());
        let i = s as usize;
        if let (Some(n), Some(sum), Some(m)) =
            (self.n.get_mut(i), self.sum.get_mut(i), self.max.get_mut(i))
        {
            *n = n.wrapping_add(1);
            *sum = sum.wrapping_add(d);
            *m = (*m).max(d as u16);
        }
        self.tick = self.tick.wrapping_add(d);
    }

    pub fn load(&self, s: TelStep) -> Load {
        let i = s as usize;
        Load {
            n: self.n[i],
            sum: self.sum[i],
            max: self.max[i].into(),
        }
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        let n = core::array::from_fn(|_| r.word());
        let sum = core::array::from_fn(|_| r.word());
        let h: [u16; TEL_STEPS + 1] = r.halves();
        Some(Self {
            n,
            sum,
            max: core::array::from_fn(|i| h[i]),
            cal: h[TEL_STEPS],
            tick: r.word(),
        })
    }
}

/// The `bench-telcost` image's TEL ticks beside [`KernelProbe`], booked by
/// its rules. The image stamps every other burst: an unstamped burst's TEL
/// and banking ticks book their bodies, by regime and phase for matching
/// against the plain ticks; a stamped burst's book their steps' total. A
/// burst's first tick (pickup and head) books apart.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TelTally {
    pub tel_n: [[u16; PHASES]; 2],
    pub bank_n: [[u16; PHASES]; 2],
    pub tel_sum: [u32; 2],
    pub bank_sum: [u32; 2],
    pub first: [Load; 2],
    pub tel_steps: Tally,
    pub bank_steps: Tally,
}

impl TelTally {
    pub const ZERO: Self = Self {
        tel_n: [[0; PHASES]; 2],
        bank_n: [[0; PHASES]; 2],
        tel_sum: [0; 2],
        bank_sum: [0; 2],
        first: [Load::ZERO; 2],
        tel_steps: Tally::ZERO,
        bank_steps: Tally::ZERO,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// The tick [`KernelProbe::tick`] booked; `first`: it picked up an arm;
    /// `steps`: its stamped steps' total, under a stamped burst.
    #[inline(always)]
    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        body: u32,
        phase: usize,
        r: Regime,
        ev: Events,
        first: bool,
        steps: Option<u32>,
        boot: bool,
    ) {
        let apart = ev.refresh || ev.close || ev.slow || ev.finish || ev.install || ev.side;
        if boot || body > HALT_TICKS || apart || ev.health || !(ev.bank || ev.tel) {
            return;
        }
        let r = r as usize;
        let (n, sum, stepped) = match (ev.bank, first) {
            (true, _) => (&mut self.bank_n, &mut self.bank_sum, &mut self.bank_steps),
            (false, false) => (&mut self.tel_n, &mut self.tel_sum, &mut self.tel_steps),
            (false, true) => {
                if let Some(l) = self.first.get_mut(r) {
                    l.add(body);
                }
                return;
            }
        };
        if let Some(d) = steps {
            stepped.n = stepped.n.wrapping_add(1);
            stepped.sum = stepped.sum.wrapping_add(d);
        } else {
            if let Some(n) = n.get_mut(r).and_then(|n| n.get_mut(phase)) {
                *n = n.saturating_add(1);
            }
            if let Some(s) = sum.get_mut(r) {
                *s = s.wrapping_add(body);
            }
        }
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        Some(Self {
            tel_n: core::array::from_fn(|_| r.halves()),
            bank_n: core::array::from_fn(|_| r.halves()),
            tel_sum: core::array::from_fn(|_| r.word()),
            bank_sum: core::array::from_fn(|_| r.word()),
            first: core::array::from_fn(|_| r.load()),
            tel_steps: Tally {
                n: r.word(),
                sum: r.word(),
            },
            bank_steps: Tally {
                n: r.word(),
                sum: r.word(),
            },
        })
    }
}

/// A `bench-telcost` window's TEL cost, HCLK ticks per tick. Host-side.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TelReport {
    /// Unstamped TEL ticks that banked nothing and banking ticks over the
    /// plain ticks of their regime and phase, and their counts; `None`
    /// where one ran on a phase that had no plain tick.
    pub tel: Option<f32>,
    pub tel_n: u32,
    pub bank: Option<f32>,
    pub bank_n: u32,
    /// The mean TEL tick's excess, one tick in [`STREAM_SAMPLES_MAX`]
    /// banking.
    pub per_tick: Option<f32>,
    /// The stamped steps' total of the same mean tick, and the stamped
    /// ticks it is over; `None` without both kinds.
    pub steps: Option<f32>,
    pub steps_n: u32,
}

impl TelReport {
    pub fn of(k: &KernelProbe, t: &TelTally) -> Self {
        let tel = matched(k, &t.tel_n, t.tel_sum.map(u64::from), None);
        let bank = matched(k, &t.bank_n, t.bank_sum.map(u64::from), None);
        let w = 1.0 / STREAM_SAMPLES_MAX as f32;
        let mix = |row: f32, bank: f32| (1.0 - w) * row + w * bank;
        let mean = |s: Tally| (s.n != 0).then(|| s.sum as f32 / s.n as f32);
        let steps = match (mean(t.tel_steps), mean(t.bank_steps)) {
            (Some(r), Some(b)) => Some(mix(r, b)),
            _ => None,
        };
        let per_tick = match (tel, bank) {
            (Some((r, _)), Some((b, _))) => Some(mix(r, b)),
            _ => None,
        };
        Self {
            tel: tel.map(|(e, _)| e),
            tel_n: tel.map_or(0, |(_, n)| n),
            bank: bank.map(|(e, _)| e),
            bank_n: bank.map_or(0, |(_, n)| n),
            per_tick,
            steps,
            steps_n: t.tel_steps.n.wrapping_add(t.bank_steps.n),
        }
    }

    /// The unstamped TEL ticks of `r` over its plain ticks, phase by phase:
    /// the [`Component::TelRow`] mean with each phase's own plain mean.
    pub fn tel_row(k: &KernelProbe, t: &TelTally, r: Regime) -> Option<f32> {
        let i = r as usize;
        let mut sum = [0; 2];
        sum[i] = u64::from(t.tel_sum[i]);
        matched(k, &t.tel_n, sum, Some(r)).map(|(e, _)| e)
    }
}

/// Ticks counted `n` by regime and phase, totalling `sum` by regime, over
/// the plain ticks' mean of each one's regime and phase; and their count.
fn matched(
    k: &KernelProbe,
    n: &[[u16; PHASES]; 2],
    sum: [u64; 2],
    only: Option<Regime>,
) -> Option<(f32, u32)> {
    let (mut count, mut total, mut plain) = (0u64, 0u64, 0f64);
    for r in Regime::ALL {
        if only.is_some_and(|o| o != r) {
            continue;
        }
        let i = r as usize;
        total += sum[i];
        for p in 0..PHASES {
            let c = u64::from(n[i][p]);
            if c == 0 {
                continue;
            }
            let t = k.tick[i][p];
            if t.n == 0 {
                return None;
            }
            count += c;
            plain += c as f64 * f64::from(t.sum) / f64::from(t.n);
        }
    }
    (count != 0).then(|| (((total as f64 - plain) / count as f64) as f32, count as u32))
}

struct Words<'a>(&'a [u32]);

impl Words<'_> {
    fn word(&mut self) -> u32 {
        let (first, rest) = self.0.split_first().map_or((0, &[][..]), |(f, r)| (*f, r));
        self.0 = rest;
        first
    }

    fn load(&mut self) -> Load {
        Load {
            n: self.word(),
            sum: self.word(),
            max: self.word(),
        }
    }

    /// `N` halfwords, two to a little-endian word (`N` even).
    fn halves<const N: usize>(&mut self) -> [u16; N] {
        let mut w = 0;
        core::array::from_fn(|i| {
            if i % 2 == 0 {
                w = self.word();
            }
            (w >> (16 * (i % 2))) as u16
        })
    }
}

/// One measured window: whether it streamed, and the frames the host sent
/// (each with its rate).
pub struct Scenario<'a> {
    pub tel: bool,
    pub frames: &'a [(Frame, u32)],
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Component {
    Tick(Regime, usize),
    TelTick(Regime, usize),
    BankTick(Regime, usize),
    /// A tick that closed a tick-load window.
    Health(Regime, usize),
    /// The configuration take.
    Refresh,
    Slow(Regime),
    Finish,
    Close,
    Install,
    Side,
    /// The TEL ticks' mean over the same regime's plain ticks' (mean only).
    TelRow(Regime),
    Body(Body),
    HostFrame,
}

/// A component's window against its budget; `n` and `mean` are 0 where the
/// probe keeps only the maximum, `mean_budget` 0 where none is set. A bus
/// body's `max` is its net time; `wall` and `nested` are that body's time
/// with the kernel inside and the kernel vectors it nested (0 for every
/// other component).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Row {
    pub component: Component,
    pub n: u32,
    pub mean: u32,
    pub max: u32,
    pub wall: u32,
    pub nested: u32,
    pub budget: u32,
    pub mean_budget: u32,
}

impl Row {
    pub fn over(&self) -> bool {
        self.max > self.budget || self.mean_over()
    }

    pub fn mean_over(&self) -> bool {
        self.mean_budget != 0 && self.n >= super::MEAN_MIN_BODIES && self.mean > self.mean_budget
    }
}

/// Every budgeted component of a window, measured against the table.
pub fn rows(k: &KernelProbe, b: &BusProbe, s: &Scenario<'_>, mut f: impl FnMut(Row)) {
    for r in Regime::ALL {
        for p in 0..PHASES {
            let budget = super::kernel_tick(r, p, Events::default());
            let mean = super::tick_body_mean(r, p);
            f(load_row(Component::Tick(r, p), k.load(r, p), budget, mean));
        }
    }
    let tel = Events {
        tel: true,
        ..Events::default()
    };
    let bank = Events { bank: true, ..tel };
    let carried = Events {
        tel: s.tel,
        bank: s.tel,
        ..Events::default()
    };
    let health = Events {
        health: true,
        ..carried
    };
    for r in Regime::ALL {
        let i = r as usize;
        for p in 0..PHASES {
            let at = |m: &[[u16; PHASES]; 2]| m[i][p].into();
            let tick = |ev| super::kernel_tick(r, p, ev);
            f(max_row(Component::TelTick(r, p), at(&k.tel_max), tick(tel)));
            f(max_row(
                Component::BankTick(r, p),
                at(&k.bank_max),
                tick(bank),
            ));
            f(max_row(
                Component::Health(r, p),
                at(&k.health_max),
                tick(health),
            ));
        }
    }
    let r = k.drove();
    let refresh = Events {
        refresh: true,
        ..carried
    };
    let budget = super::kernel_tick(r, phase::SPARE_A as usize, refresh);
    f(load_row(Component::Refresh, k.refresh, budget, 0));
    let slow = Events {
        slow: true,
        ..carried
    };
    let p = crate::kernel::phase::SLOW as usize;
    for (r, l) in Regime::ALL.into_iter().zip(k.slow) {
        f(load_row(
            Component::Slow(r),
            l,
            super::kernel_tick(r, p, slow),
            0,
        ));
    }
    let finish = Events {
        finish: true,
        ..carried
    };
    let budget = super::kernel_tick(Regime::Closed, p, finish);
    f(load_row(Component::Finish, k.finish, budget, 0));
    let close = Events {
        close: true,
        ..finish
    };
    let budget = super::kernel_tick(Regime::Closed, p, close);
    f(load_row(Component::Close, k.close, budget, 0));
    let stages = [
        (Component::Install, k.install, crate::kernel::phase::SPARE_A),
        (Component::Side, k.side, crate::kernel::phase::SPARE_B),
    ];
    for (c, l, p) in stages {
        let ev = Events {
            install: c == Component::Install,
            side: c == Component::Side,
            ..carried
        };
        let budget = super::kernel_tick(Regime::Closed, p as usize, ev);
        f(load_row(c, l, budget, 0));
    }
    for (r, (plain, tel)) in Regime::ALL.into_iter().zip(k.tick.iter().zip(k.tel)) {
        let (n, sum) = plain.iter().fold((0u64, 0u64), |(n, sum), l| {
            (n + l.n as u64, sum + l.sum as u64)
        });
        let plain_mean = sum.checked_div(n).unwrap_or(0) as u32;
        f(Row {
            component: Component::TelRow(r),
            n: tel.n,
            mean: tel.mean().saturating_sub(plain_mean),
            max: 0,
            wall: 0,
            nested: 0,
            budget: 0,
            mean_budget: super::TEL_ROW_MEAN + super::TEL_LANE_MEAN,
        });
    }
    let slowest = s
        .frames
        .iter()
        .map(|&(_, baud)| baud)
        .min()
        .unwrap_or(super::BUDGET_BAUD_HZ);
    let bodies = Body::ALL.into_iter().zip(b.body).zip(b.wall).zip(b.nested);
    for (((body, l), wall), nested) in bodies {
        let spin = match body {
            Body::Deadline => super::spin_extra(slowest),
            _ => 0,
        };
        let budget = super::body(body) + spin;
        let mean = super::body_mean(body) + spin;
        f(Row {
            wall,
            nested,
            ..load_row(Component::Body(body), l, budget, mean)
        });
    }
    let host = || {
        s.frames
            .iter()
            .map(|&(fr, baud)| (fr, super::spin_extra(baud)))
    };
    let max = host().map(|(fr, spin)| super::frame(fr) + spin).max();
    let mean = host().map(|(fr, spin)| super::frame_mean(fr) + spin).max();
    f(load_row(
        Component::HostFrame,
        b.host,
        max.unwrap_or(0),
        mean.unwrap_or(0),
    ));
}

fn load_row(component: Component, l: Load, budget: u32, mean_budget: u32) -> Row {
    Row {
        component,
        n: l.n,
        mean: l.mean(),
        max: l.max,
        wall: 0,
        nested: 0,
        budget,
        mean_budget,
    }
}

fn max_row(component: Component, max: u32, budget: u32) -> Row {
    Row {
        component,
        n: 0,
        mean: 0,
        max,
        wall: 0,
        nested: 0,
        budget,
        mean_budget: 0,
    }
}

/// `ticks` in us, for reports.
pub fn us(ticks: u32) -> f32 {
    ticks as f32 * 1e6 / CLOCK_HZ as f32
}

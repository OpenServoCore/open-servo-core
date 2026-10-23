//! The budget probe's records and their check. A bench build keeps one
//! [`KernelProbe`] (written only by the kernel vector, after each tick's
//! exit stamp) and one [`BusProbe`] (written only at the bus level); the
//! hardware test decodes both from a debug-link dump and holds them against
//! the table. A `bench-window` build adds a [`WindowProbe`] beside them:
//! gamma(k), the long ticks and the entry lateness, which only some runs
//! read.

use super::{
    Body, CLOCK_HZ, EVENT_REGIMES, Events, Frame, PHASES, Regime, TICKS_PER_US, WINDOW_TICKS,
};
use crate::kernel::phases::loops::faults::stage_due;
use crate::kernel::{ProbeTags, phase};
use crate::tel::STREAM_SAMPLES_MAX;

impl Events {
    /// The events of the kernel tick between `before` and `after`, its tags
    /// on either side of the body; `tel`, `bank` and `health` are the
    /// caller's.
    pub fn between(before: ProbeTags, after: ProbeTags) -> Self {
        Self {
            refresh: after.config_seq != before.config_seq,
            slow: before.slow_due,
            finish: before.slow_finish,
            drive: before.phase == phase::SPARE_B
                && stage_due(before.drive)
                && after.drive != before.drive,
            ..Self::default()
        }
    }
}

/// A body longer than this is a debugger halt or a flash save, not a
/// budgeted body: counted, kept out of every statistic.
pub const HALT_TICKS: u32 = 1000 * TICKS_PER_US;

const _: () = assert!(HALT_TICKS <= u16::MAX as u32);

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

    #[inline(always)]
    pub fn add(&mut self, d: u32) {
        self.n = self.n.wrapping_add(1);
        self.sum = self.sum.wrapping_add(d);
    }

    /// Host-side: it divides.
    pub fn mean(&self) -> u32 {
        self.sum.checked_div(self.n).unwrap_or(0)
    }
}

const RING: usize = WINDOW_TICKS[WINDOW_TICKS.len() - 1];
const K: usize = WINDOW_TICKS.len();

const _: () = {
    let mut j = 0;
    while j < K {
        assert!(WINDOW_TICKS[j] >= 1 && WINDOW_TICKS[j] <= RING);
        j += 1;
    }
};

/// gamma(k), k in [`WINDOW_TICKS`]: the most the bodies of k consecutive
/// booked ticks took, HCLK ticks. Exact while k consecutive bodies sum
/// under 2^16 ticks (1.36 ms): a kernel that far behind drops ticks.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Windows {
    /// The running total of the bodies, mod 2^16, after each of the last
    /// `RING` ticks, held twice over (slot `i` and `i + RING`) so every
    /// lookback is a fixed offset from `at`, the slot the next tick takes.
    ring: [u16; 2 * RING],
    at: u32,
    pub gamma: [u32; K],
}

impl Windows {
    pub const ZERO: Self = Self {
        ring: [0; 2 * RING],
        at: 0,
        gamma: [0; K],
    };

    /// The next booked tick's body (under [`HALT_TICKS`], so 16 bits).
    #[inline(always)]
    pub fn add(&mut self, body: u32) {
        let at = if (self.at as usize) < RING {
            self.at as usize
        } else {
            core::hint::cold_path();
            0
        };
        let total = self.ring[at + RING - 1].wrapping_add(body as u16);
        for (g, k) in self.gamma.iter_mut().zip(WINDOW_TICKS) {
            let s = total.wrapping_sub(self.ring[at + RING - k]) as u32;
            if s > *g {
                core::hint::cold_path();
                *g = s;
            }
        }
        self.ring[at] = total;
        self.ring[at + RING] = total;
        self.at = if at + 1 < RING { at as u32 + 1 } else { 0 };
    }
}

impl Default for Windows {
    fn default() -> Self {
        Self::ZERO
    }
}

/// A tick this long counts toward [`WindowProbe::long`]: the hold windows'
/// maxima sit above it, the parked ticks below.
pub const LONG_TICKS: u32 = 30 * TICKS_PER_US;

/// The kernel ticks' uncommon instruments (`bench-window`): gamma(k), the
/// long ticks and the entry lateness. Fed every kernel tick after
/// [`KernelProbe::tick`] and every vector's last stamp.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WindowProbe {
    last_entry: u32,
    last_end: u32,
    /// Entry stamp to the later of the grid slot (last entry + one period)
    /// and the last tick's end plus [`super::ENTRY_EXIT`]: the most another
    /// context held the tick off.
    pub late_max: u32,
    /// Ticks of at least [`LONG_TICKS`], by event row.
    pub long: [u32; EVENT_REGIMES],
    pub windows: Windows,
}

impl WindowProbe {
    pub const ZERO: Self = Self {
        last_entry: 0,
        last_end: 0,
        late_max: 0,
        long: [0; EVENT_REGIMES],
        windows: Windows::ZERO,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// The tick entered at `entry`, `body` ticks long; `booked`: its regime
    /// when [`KernelProbe::tick`] booked it (neither the boot tick nor a
    /// halt).
    #[inline(always)]
    pub fn tick(&mut self, entry: u32, body: u32, booked: Option<Regime>) {
        let due = self.last_entry.wrapping_add(super::TICK_PERIOD);
        let free = self.last_end.wrapping_add(super::ENTRY_EXIT);
        let slot = if (free.wrapping_sub(due) as i32) > 0 {
            free
        } else {
            due
        };
        let late = entry.wrapping_sub(slot);
        self.last_entry = entry;
        let Some(r) = booked else {
            return;
        };
        if late < HALT_TICKS && late > self.late_max {
            core::hint::cold_path();
            self.late_max = late;
        }
        if let Some(n) = self.long.get_mut(r.event_row() as usize) {
            *n = n.wrapping_add((body >= LONG_TICKS) as u32);
        }
        self.windows.add(body);
    }

    /// The vector's last stamp, `end`.
    #[inline(always)]
    pub fn ended(&mut self, end: u32) {
        self.last_end = end;
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        Some(Self {
            last_entry: r.word(),
            last_end: r.word(),
            late_max: r.word(),
            long: core::array::from_fn(|_| r.word()),
            windows: Windows {
                ring: r.halves(),
                at: r.word(),
                gamma: core::array::from_fn(|_| r.word()),
            },
        })
    }
}

/// Plain ticks of one regime by phase, narrow: a phase's count stops at
/// `u16::MAX` and its total with it, so the mean stays exact over the
/// bodies counted; the longest keeps moving.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct NarrowLoad {
    pub n: [u16; PHASES],
    pub max: [u16; PHASES],
    pub sum: [u32; PHASES],
}

impl NarrowLoad {
    pub const ZERO: Self = Self {
        n: [0; PHASES],
        max: [0; PHASES],
        sum: [0; PHASES],
    };

    /// A body under [`HALT_TICKS`] at `phase`.
    #[inline(always)]
    pub fn add(&mut self, phase: usize, body: u16) {
        if let (Some(n), Some(sum), Some(m)) = (
            self.n.get_mut(phase),
            self.sum.get_mut(phase),
            self.max.get_mut(phase),
        ) {
            if *n < u16::MAX {
                *n += 1;
                *sum = sum.wrapping_add(body.into());
            }
            if body > *m {
                core::hint::cold_path();
                *m = body;
            }
        }
    }

    pub fn load(&self, phase: usize) -> Load {
        Load {
            n: self.n.get(phase).map_or(0, |&n| n.into()),
            sum: self.sum.get(phase).copied().unwrap_or(0),
            max: self.max.get(phase).map_or(0, |&m| m.into()),
        }
    }
}

/// The count at which a [`NarrowLoad`] phase stops.
pub const NARROW_N_MAX: u32 = u16::MAX as u32;

/// The kernel ticks of a window. The fields every tick touches lead, so
/// the update reaches them at short offsets.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct KernelProbe {
    /// Every kernel vector so far, with its interrupt entry and exit: what a
    /// bus body subtracts.
    pub busy: u32,
    /// Kernel vectors entered so far: what a bus body counts as nested.
    pub entries: u32,
    pub halts: u32,
    /// The kernel's tags at the last tick's end, the next tick's before,
    /// and the TEL batches banked by then, mod 2^8 (one a tick at most).
    before: ProbeTags,
    banks: u8,
    /// Ticks with no event, by event row and phase, and the longest of
    /// each, OpenLoop's narrow: [`Self::load`].
    pub tick: [[Tally; PHASES]; EVENT_REGIMES],
    pub tick_max: [[u16; PHASES]; EVENT_REGIMES],
    pub openloop: NarrowLoad,
    /// Longest TEL tick that banked nothing, by event row and phase.
    pub tel_max: [[u16; PHASES]; EVENT_REGIMES],
    /// Longest banking tick, by event row and phase.
    pub bank_max: [[u16; PHASES]; EVENT_REGIMES],
    /// Longest tick that closed a tick-load window, by event row and phase,
    /// whatever TEL work it also carried; no TEL or plain row holds one.
    pub health_max: [[u16; PHASES]; EVENT_REGIMES],
    /// TEL ticks that banked nothing, by event row, all phases.
    pub tel: [Tally; EVENT_REGIMES],
    /// Ticks that took the configuration.
    pub refresh: Load,
    /// The SLOW block's due ticks by event row and its finish ticks.
    pub slow: [Load; EVENT_REGIMES],
    pub finish: Load,
    /// The drive direction check's stages on phase SPARE_B.
    pub drive: Load,
}

impl KernelProbe {
    pub const ZERO: Self = Self {
        busy: 0,
        entries: 0,
        halts: 0,
        before: ProbeTags {
            phase: 0,
            config_seq: 0,
            booted: false,
            slow_due: false,
            slow_finish: false,
            regime: Regime::Open,
            drive: 0,
        },
        banks: 0,
        tick: [[Tally::ZERO; PHASES]; EVENT_REGIMES],
        tick_max: [[0; PHASES]; EVENT_REGIMES],
        openloop: NarrowLoad::ZERO,
        tel_max: [[0; PHASES]; EVENT_REGIMES],
        bank_max: [[0; PHASES]; EVENT_REGIMES],
        health_max: [[0; PHASES]; EVENT_REGIMES],
        tel: [Tally::ZERO; EVENT_REGIMES],
        refresh: Load::ZERO,
        slow: [Load::ZERO; EVENT_REGIMES],
        finish: Load::ZERO,
        drive: Load::ZERO,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// One kernel tick, `body` ticks from its entry stamp to its exit
    /// stamp, that left the kernel at `after()`; `tel`: the TEL channel streams;
    /// `banks`: the TEL batches banked so far; `health`: the tick-load
    /// counters closed a window. Its regime when the tick booked (neither
    /// the boot tick nor a halt). A tick whose tags held still, with no SLOW
    /// stage on it or on the next tick, books as a plain or TEL tick in
    /// line, reading only the tags it compares; any other goes through
    /// [`Self::event_tick`].
    #[inline(always)]
    pub fn tick(
        &mut self,
        body: u32,
        after: impl Fn() -> ProbeTags,
        tel: bool,
        banks: u32,
        health: bool,
    ) -> Option<Regime> {
        let b = self.before;
        let a = after();
        if b.booted
            && !health
            && banks as u8 == self.banks
            && a.config_seq == b.config_seq
            && a.regime == b.regime
            && a.drive == b.drive
            && !b.slow_due
            && !b.slow_finish
            && !a.slow_due
            && !a.slow_finish
        {
            self.before.phase = a.phase;
            let ev = Events {
                tel,
                ..Events::default()
            };
            self.book(body, b.phase as usize, b.regime, ev, false)
        } else {
            core::hint::cold_path();
            self.event_tick(body, after(), tel, banks, health)
        }
    }

    /// [`Self::tick`]'s events: what the tick carried between the carried
    /// tags and `after`, and its regime, phase and whether it booted.
    pub fn events(
        &self,
        after: ProbeTags,
        tel: bool,
        banks: u32,
        health: bool,
    ) -> (Events, Regime, usize, bool) {
        let b = self.before;
        let ev = Events {
            tel,
            bank: banks as u8 != self.banks,
            health,
            ..Events::between(b, after)
        };
        (
            ev,
            b.regime.booked(after.regime),
            b.phase as usize,
            !b.booted,
        )
    }

    #[inline(never)]
    #[rustc_align(4)]
    fn event_tick(
        &mut self,
        body: u32,
        after: ProbeTags,
        tel: bool,
        banks: u32,
        health: bool,
    ) -> Option<Regime> {
        let (ev, r, phase, boot) = self.events(after, tel, banks, health);
        self.before = after;
        self.banks = banks as u8;
        self.book(body, phase, r, ev, boot)
    }

    /// One kernel tick, `body` ticks long, that ran `phase` in `regime` and
    /// carried `ev`; its regime when it booked. The boot tick and a halt
    /// book nothing.
    #[inline(always)]
    pub fn book(
        &mut self,
        body: u32,
        phase: usize,
        regime: Regime,
        ev: Events,
        boot: bool,
    ) -> Option<Regime> {
        if boot {
            return None;
        }
        if body > HALT_TICKS {
            core::hint::cold_path();
            self.halts = self.halts.wrapping_add(1);
            return None;
        }
        let body16 = body as u16;
        let r = regime.event_row() as usize;
        let max16 = |m: &mut [[u16; PHASES]; EVENT_REGIMES]| {
            if let Some(m) = m.get_mut(r).and_then(|m| m.get_mut(phase))
                && body16 > *m
            {
                core::hint::cold_path();
                *m = body16;
            }
        };
        if ev.refresh {
            self.refresh.add(body);
        } else if ev.slow {
            if let Some(l) = self.slow.get_mut(r) {
                l.add(body);
            }
        } else if ev.finish {
            self.finish.add(body);
        } else if ev.drive {
            self.drive.add(body);
        } else if ev.health {
            max16(&mut self.health_max);
        } else if ev.bank {
            max16(&mut self.bank_max);
        } else if ev.tel {
            if let Some(t) = self.tel.get_mut(r) {
                t.add(body);
            }
            max16(&mut self.tel_max);
        } else if regime == Regime::OpenLoop {
            self.openloop.add(phase, body16);
        } else {
            if let Some(t) = self.tick.get_mut(r).and_then(|t| t.get_mut(phase)) {
                t.add(body);
            }
            if let Some(m) = self.tick_max.get_mut(r).and_then(|m| m.get_mut(phase))
                && body16 > *m
            {
                core::hint::cold_path();
                *m = body16;
            }
        }
        Some(regime)
    }

    /// The plain ticks of `r` at `phase`.
    pub fn load(&self, r: Regime, phase: usize) -> Load {
        if r == Regime::OpenLoop {
            return self.openloop.load(phase);
        }
        let r = r as usize;
        let t = self.tick.get(r).and_then(|t| t.get(phase));
        let max = self.tick_max.get(r).and_then(|m| m.get(phase));
        Load {
            n: t.map_or(0, |t| t.n),
            sum: t.map_or(0, |t| t.sum),
            max: max.map_or(0, |&m| m.into()),
        }
    }

    /// The tick's vector, entered at `entry`, reached its last stamp at
    /// `end`: its time with the probe's own update, and the vector's entry
    /// and exit, run inside any bus body it preempted.
    #[inline(always)]
    pub fn ended(&mut self, entry: u32, end: u32) {
        self.other(end.wrapping_sub(entry));
    }

    /// A body on the kernel's vector that is not a tick (shunt burst events),
    /// with the vector's entry and exit.
    #[inline(always)]
    pub fn other(&mut self, body: u32) {
        self.busy = self.busy.wrapping_add(body).wrapping_add(super::ENTRY_EXIT);
        self.entries = self.entries.wrapping_add(1);
    }

    /// The event row the window's TEL and event ticks are held to: Closed
    /// once any tick drove.
    pub fn drove(&self) -> Regime {
        let closed = Regime::Closed as usize;
        let driven = [Regime::Closed, Regime::OpenLoop]
            .into_iter()
            .any(|r| (0..PHASES).any(|p| self.load(r, p).n != 0));
        if driven || self.slow[closed].n != 0 {
            Regime::Closed
        } else {
            Regime::Open
        }
    }

    /// The plain ticks at `phase` of every regime that books on event row
    /// `r`: what that row's TEL ticks are matched against.
    pub fn plain(&self, r: Regime, phase: usize) -> Tally {
        Regime::ALL
            .into_iter()
            .filter(|q| q.event_row() == r)
            .map(|q| self.load(q, phase))
            .fold(Tally::ZERO, |a, t| Tally {
                n: a.n.wrapping_add(t.n),
                sum: a.sum.wrapping_add(t.sum),
            })
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        let busy = r.word();
        let entries = r.word();
        let halts = r.word();
        let [phase, config_seq, booted, slow_due] = r.word().to_le_bytes();
        let [slow_finish, regime, drive, banks] = r.word().to_le_bytes();
        Some(Self {
            busy,
            entries,
            halts,
            before: ProbeTags {
                phase,
                config_seq,
                booted: booted != 0,
                slow_due: slow_due != 0,
                slow_finish: slow_finish != 0,
                regime: *Regime::ALL.get(usize::from(regime))?,
                drive,
            },
            banks,
            tick: core::array::from_fn(|_| {
                core::array::from_fn(|_| Tally {
                    n: r.word(),
                    sum: r.word(),
                })
            }),
            tick_max: core::array::from_fn(|_| r.halves()),
            openloop: NarrowLoad {
                n: r.halves(),
                max: r.halves(),
                sum: core::array::from_fn(|_| r.word()),
            },
            tel_max: core::array::from_fn(|_| r.halves()),
            bank_max: core::array::from_fn(|_| r.halves()),
            health_max: core::array::from_fn(|_| r.halves()),
            tel: core::array::from_fn(|_| Tally {
                n: r.word(),
                sum: r.word(),
            }),
            refresh: r.load(),
            slow: core::array::from_fn(|_| r.load()),
            finish: r.load(),
            drive: r.load(),
        })
    }
}

#[cfg(test)]
impl KernelProbe {
    /// The record less the tags it carries between ticks.
    pub(crate) fn booked(&self) -> Self {
        Self {
            before: Self::ZERO.before,
            banks: 0,
            ..*self
        }
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
/// tick's steps, which the tick's end takes. `hook`: the running tick's
/// time inside the two TEL hooks, top to bottom, flag test included, on
/// every tick, which the tick's end takes.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TelCost {
    pub n: [u32; TEL_STEPS],
    pub sum: [u32; TEL_STEPS],
    pub max: [u16; TEL_STEPS],
    pub cal: u16,
    pub tick: u32,
    pub hook: u32,
}

impl TelCost {
    pub const ZERO: Self = Self {
        n: [0; TEL_STEPS],
        sum: [0; TEL_STEPS],
        max: [0; TEL_STEPS],
        cal: 0,
        tick: 0,
        hook: 0,
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
            hook: r.word(),
        })
    }
}

/// The `bench-telcost` image's TEL ticks beside [`KernelProbe`], booked by
/// its rules. The image stamps every other burst: an unstamped burst's TEL
/// and banking ticks book their bodies, by regime and phase for matching
/// against the plain ticks, and their hook time ([`TelCost::hook`]); a
/// stamped burst's book their steps' total. A burst's first tick (pickup
/// and head) books apart. The plain ticks book their hook time by regime.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TelTally {
    pub tel_n: [[u16; PHASES]; EVENT_REGIMES],
    pub bank_n: [[u16; PHASES]; EVENT_REGIMES],
    pub tel_sum: [u32; EVENT_REGIMES],
    pub bank_sum: [u32; EVENT_REGIMES],
    pub first: [Load; EVENT_REGIMES],
    pub tel_steps: Tally,
    pub bank_steps: Tally,
    pub hook_plain: [Tally; EVENT_REGIMES],
    pub hook_tel: u32,
    pub hook_bank: u32,
}

impl TelTally {
    pub const ZERO: Self = Self {
        tel_n: [[0; PHASES]; EVENT_REGIMES],
        bank_n: [[0; PHASES]; EVENT_REGIMES],
        tel_sum: [0; EVENT_REGIMES],
        bank_sum: [0; EVENT_REGIMES],
        first: [Load::ZERO; EVENT_REGIMES],
        tel_steps: Tally::ZERO,
        bank_steps: Tally::ZERO,
        hook_plain: [Tally::ZERO; EVENT_REGIMES],
        hook_tel: 0,
        hook_bank: 0,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// The tick [`KernelProbe::tick`] booked; `first`: it picked up an arm;
    /// `steps`: its stamped steps' total, under a stamped burst; `hook`: its
    /// time inside the TEL hooks.
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
        hook: u32,
        boot: bool,
    ) {
        let apart = ev.refresh || ev.slow || ev.finish || ev.drive;
        if boot || body > HALT_TICKS || apart || ev.health {
            return;
        }
        let r = r.event_row() as usize;
        if !(ev.bank || ev.tel) {
            if let Some(p) = self.hook_plain.get_mut(r) {
                p.add(hook);
            }
            return;
        }
        let (n, sum, stepped, hooks) = match (ev.bank, first) {
            (true, _) => (
                &mut self.bank_n,
                &mut self.bank_sum,
                &mut self.bank_steps,
                &mut self.hook_bank,
            ),
            (false, false) => (
                &mut self.tel_n,
                &mut self.tel_sum,
                &mut self.tel_steps,
                &mut self.hook_tel,
            ),
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
            *hooks = hooks.wrapping_add(hook);
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
            hook_plain: core::array::from_fn(|_| Tally {
                n: r.word(),
                sum: r.word(),
            }),
            hook_tel: r.word(),
            hook_bank: r.word(),
        })
    }
}

/// Twice the largest run-to-run sd of a window's TEL per tick, HCLK
/// ticks: 2.8 over four runs each of three telcost images in tel still
/// and tel stepping. The plain ticks a moving window matches against ran
/// in another plant state.
pub const TEL_PER_TICK_NOISE: f32 = 6.0;

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
    /// The same unstamped ticks' time inside the TEL hooks over the plain
    /// ticks' of their regime, mixed as `per_tick`: the TEL code's whole
    /// cost, flag tests included.
    pub hooks: Option<f32>,
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
        let hooks = match (
            hooked(&t.tel_n, t.hook_tel, &t.hook_plain),
            hooked(&t.bank_n, t.hook_bank, &t.hook_plain),
        ) {
            (Some(r), Some(b)) => Some(mix(r, b)),
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
            hooks,
        }
    }

    /// What the hooks run that no step stamps: the gates, the register
    /// saves around the out-of-line steps and the stamping flag's tests.
    pub fn glue(&self) -> Option<f32> {
        Some(self.hooks? - self.steps?)
    }

    /// The TEL tick's excess outside the hooks: the TX DMA's cycles taken
    /// from the rest of the tick, and the plain ticks' noise.
    pub fn outside(&self) -> Option<f32> {
        Some(self.per_tick? - self.hooks?)
    }

    /// The self-check: the steps nest in the hooks, and the hooks in the
    /// tick, less [`TEL_PER_TICK_NOISE`]; `None` without all three.
    pub fn check(&self) -> Option<bool> {
        let (s, h, x) = (self.steps?, self.hooks?, self.per_tick?);
        Some(s <= h && h <= x + TEL_PER_TICK_NOISE)
    }

    /// The unstamped TEL ticks of `r` over its plain ticks, phase by phase:
    /// the [`Component::TelRow`] mean with each phase's own plain mean.
    pub fn tel_row(k: &KernelProbe, t: &TelTally, r: Regime) -> Option<f32> {
        let i = r.event_row() as usize;
        let mut sum = [0; EVENT_REGIMES];
        sum[i] = u64::from(t.tel_sum[i]);
        matched(k, &t.tel_n, sum, Some(r.event_row())).map(|(e, _)| e)
    }
}

/// Ticks counted `n` by regime and phase with `sum` hook time in all, over
/// the plain ticks' hook time of each one's regime, per tick.
fn hooked(
    n: &[[u16; PHASES]; EVENT_REGIMES],
    sum: u32,
    plain: &[Tally; EVENT_REGIMES],
) -> Option<f32> {
    let (mut count, mut base) = (0u64, 0f64);
    for (n, p) in n.iter().zip(plain) {
        let c = n.iter().map(|&c| u64::from(c)).sum::<u64>();
        if c == 0 {
            continue;
        }
        if p.n == 0 {
            return None;
        }
        count += c;
        base += c as f64 * f64::from(p.sum) / f64::from(p.n);
    }
    (count != 0).then(|| ((f64::from(sum) - base) / count as f64) as f32)
}

/// Ticks counted `n` by regime and phase, totalling `sum` by regime, over
/// the plain ticks' mean of each one's regime and phase; and their count.
fn matched(
    k: &KernelProbe,
    n: &[[u16; PHASES]; EVENT_REGIMES],
    sum: [u64; EVENT_REGIMES],
    only: Option<Regime>,
) -> Option<(f32, u32)> {
    let (mut count, mut total, mut plain) = (0u64, 0u64, 0f64);
    for r in Regime::EVENT_ROWS {
        if only.is_some_and(|o| o != r) {
            continue;
        }
        let i = r as usize;
        total += sum[i];
        for (p, &c) in n[i].iter().enumerate() {
            let c = u64::from(c);
            if c == 0 {
                continue;
            }
            let t = k.plain(r, p);
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
    /// The drive direction check's stage.
    Drive,
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
    /// An OpenLoop plain row whose count stopped: its mean covers the first
    /// [`NARROW_N_MAX`] bodies.
    pub fn saturated(&self) -> bool {
        matches!(self.component, Component::Tick(Regime::OpenLoop, _)) && self.n >= NARROW_N_MAX
    }

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
    for r in Regime::EVENT_ROWS {
        let i = r as usize;
        for p in 0..PHASES {
            let at = |m: &[[u16; PHASES]; EVENT_REGIMES]| m[i][p].into();
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
    for (r, l) in Regime::EVENT_ROWS.into_iter().zip(k.slow) {
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
    let drive = Events {
        drive: true,
        ..carried
    };
    let budget = super::kernel_tick(Regime::Closed, phase::SPARE_B as usize, drive);
    f(load_row(Component::Drive, k.drive, budget, 0));
    for (r, tel) in Regime::EVENT_ROWS.into_iter().zip(k.tel) {
        let (n, sum) = (0..PHASES)
            .map(|p| k.plain(r, p))
            .fold((0u64, 0u64), |(n, sum), l| {
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

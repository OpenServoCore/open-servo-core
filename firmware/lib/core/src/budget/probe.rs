//! The budget probe's records and their check. A bench build keeps one
//! [`KernelProbe`] (written only by the kernel tick) and one [`BusProbe`]
//! (written only at the bus level); the hardware test decodes both from a
//! debug-link dump and holds them against the table.

use super::{Body, CLOCK_HZ, Events, Frame, PHASES, Regime, TICKS_PER_US, WINDOW_TICKS};
use crate::kernel::phase;

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

/// [`KernelProbe::refresh`]'s index for a refresh tick at `phase`.
fn refresh_stage(phase: usize) -> usize {
    (phase != phase::THERM_INSTALL as usize) as usize
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
    /// Longest TEL tick that banked nothing, by phase.
    pub tel_max: [u16; PHASES],
    /// Longest banking tick, by phase.
    pub bank_max: [u16; PHASES],
    /// TEL ticks that banked nothing, all phases.
    pub tel: Load,
    /// Ticks that ran a configuration refresh stage, by stage.
    pub refresh: [Load; 2],
    /// SLOW-due ticks by regime, and those that froze a thermal reference
    /// window; the thermometer's stages on phases THERM_INSTALL and
    /// THERM_SIDE.
    pub slow: [Load; 2],
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
        tel_max: [0; PHASES],
        bank_max: [0; PHASES],
        tel: Load::ZERO,
        refresh: [Load::ZERO; 2],
        slow: [Load::ZERO; 2],
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
            if let Some(l) = self.refresh.get_mut(refresh_stage(phase)) {
                l.add(body);
            }
        } else if ev.close {
            self.close.add(body);
        } else if ev.slow {
            if let Some(l) = self.slow.get_mut(r) {
                l.add(body);
            }
        } else if ev.install {
            self.install.add(body);
        } else if ev.side {
            self.side.add(body);
        } else if ev.bank {
            if let Some(m) = self.bank_max.get_mut(phase) {
                *m = (*m).max(body16);
            }
        } else if ev.tel {
            self.tel.add(body);
            if let Some(m) = self.tel_max.get_mut(phase) {
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
            tel_max: r.halves(),
            bank_max: r.halves(),
            tel: r.load(),
            refresh: core::array::from_fn(|_| r.load()),
            slow: core::array::from_fn(|_| r.load()),
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
    /// Host frames: every bus body from one break wake to the next own
    /// frame, the reply excepted. A window's last frame closes at
    /// [`Self::closed`].
    pub host: Load,
    /// TEL frames: every bus body from one own frame start to the next. A
    /// start counts as a TEL frame when the last break already had its
    /// reply: the host stays silent through a burst.
    pub tel: Load,
    pub halts: u32,
    /// Break-wake overflows: breaks, and a parked low's re-fires.
    pub breaks: u32,
    pub refires: u32,
    /// Overflows that found the line still low, and of those, the ones whose
    /// rising edge landed between the pin read and the park.
    pub parks: u32,
    pub rises: u32,
    acc: u32,
    /// [`SAW_BREAK`], [`SAW_START`] and [`SAW_DISPATCH`] for the running
    /// body; [`REPLIED`] and [`ACC_TEL`] across bodies.
    flags: u32,
}

const SAW_BREAK: u32 = 1 << 0;
const SAW_START: u32 = 1 << 1;
const SAW_DISPATCH: u32 = 1 << 2;
/// The last break already had its reply.
const REPLIED: u32 = 1 << 3;
/// The open frame is a TEL frame.
const ACC_TEL: u32 = 1 << 4;
const SAW: u32 = SAW_BREAK | SAW_START | SAW_DISPATCH;

impl BusProbe {
    pub const ZERO: Self = Self {
        body: [Load::ZERO; Body::ALL.len()],
        wall: [0; Body::ALL.len()],
        nested: [0; Body::ALL.len()],
        host: Load::ZERO,
        tel: Load::ZERO,
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

    /// The running body started an own frame (its break is on the wire).
    #[inline(always)]
    pub fn mark_start(&mut self) {
        self.flags |= SAW_START;
    }

    /// The running body dispatched a frame.
    #[inline(always)]
    pub fn mark_dispatch(&mut self) {
        self.flags |= SAW_DISPATCH;
    }

    /// A bus body of `excl` ticks ends, `wall` with the `nested` kernel
    /// vectors inside it. A stager body that started no frame books as
    /// [`Body::TelPoll`], a break wake that dispatched as [`Body::Deadline`].
    #[inline(always)]
    pub fn body(&mut self, excl: u32, wall: u32, nested: u32, b: Body) {
        let saw = self.flags;
        self.flags &= !SAW;
        let brk = saw & SAW_BREAK != 0;
        let started = saw & SAW_START != 0;
        let dispatched = saw & SAW_DISPATCH != 0;
        if excl > HALT_TICKS {
            self.halts = self.halts.wrapping_add(1);
            return;
        }
        let b = match b {
            Body::TelStage if !started => Body::TelPoll,
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
            self.fold(false);
            self.flags &= !REPLIED;
        }
        if started {
            if self.flags & REPLIED != 0 {
                self.fold(true);
            }
            self.flags |= REPLIED;
        }
        self.acc = self.acc.wrapping_add(excl);
    }

    /// The record with its open frame closed: read after the traffic, the
    /// last frame has run every body it will.
    pub fn closed(mut self) -> Self {
        self.fold(false);
        self
    }

    /// Close the frame accumulated so far; the next one is TEL if `tel`.
    #[inline(always)]
    fn fold(&mut self, tel: bool) {
        if self.acc != 0 {
            if self.flags & ACC_TEL != 0 {
                self.tel.add(self.acc);
            } else {
                self.host.add(self.acc);
            }
        }
        self.acc = 0;
        self.flags = if tel {
            self.flags | ACC_TEL
        } else {
            self.flags & !ACC_TEL
        };
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
            tel: r.load(),
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
    TelTick(usize),
    BankTick(usize),
    /// The refresh stage at this phase.
    Refresh(usize),
    Slow(Regime),
    Close,
    Install,
    Side,
    /// The TEL ticks' mean over the plain ticks' (mean only).
    TelEncode,
    Body(Body),
    HostFrame,
    TelFrame,
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
            let mean = super::KERNEL_MEAN[r as usize][p];
            f(load_row(Component::Tick(r, p), k.load(r, p), budget, mean));
        }
    }
    let r = k.drove();
    let tel = Events {
        tel: true,
        ..Events::default()
    };
    let bank = Events { bank: true, ..tel };
    for (p, (tel_max, bank_max)) in k.tel_max.into_iter().zip(k.bank_max).enumerate() {
        f(max_row(
            Component::TelTick(p),
            tel_max.into(),
            super::kernel_tick(r, p, tel),
        ));
        f(max_row(
            Component::BankTick(p),
            bank_max.into(),
            super::kernel_tick(r, p, bank),
        ));
    }
    let carried = Events {
        tel: s.tel,
        bank: s.tel,
        ..Events::default()
    };
    let refresh = Events {
        refresh: true,
        ..carried
    };
    for (p, l) in [phase::THERM_INSTALL, phase::THERM_SIDE]
        .map(usize::from)
        .into_iter()
        .zip(k.refresh)
    {
        let budget = super::kernel_tick(r, p, refresh);
        f(load_row(Component::Refresh(p), l, budget, 0));
    }
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
    let close = Events {
        close: true,
        ..slow
    };
    let budget = super::kernel_tick(Regime::Closed, p, close);
    f(load_row(Component::Close, k.close, budget, 0));
    let stages = [
        (
            Component::Install,
            k.install,
            crate::kernel::phase::THERM_INSTALL,
        ),
        (Component::Side, k.side, crate::kernel::phase::THERM_SIDE),
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
    let plain = k.tick.iter().flatten().fold((0u64, 0u64), |(n, sum), l| {
        (n + l.n as u64, sum + l.sum as u64)
    });
    let plain_mean = plain.1.checked_div(plain.0).unwrap_or(0) as u32;
    f(Row {
        component: Component::TelEncode,
        n: k.tel.n,
        mean: k.tel.mean().saturating_sub(plain_mean),
        max: 0,
        wall: 0,
        nested: 0,
        budget: 0,
        mean_budget: super::TEL_ENCODE_MEAN[r as usize],
    });
    let slowest = s
        .frames
        .iter()
        .map(|&(_, baud)| baud)
        .min()
        .unwrap_or(super::BUDGET_BAUD_HZ);
    let bodies = Body::ALL.into_iter().zip(b.body).zip(b.wall).zip(b.nested);
    for (((body, l), wall), nested) in bodies {
        let spin = match body {
            Body::Deadline | Body::TelStage => super::spin_extra(slowest),
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
        let frames = s.frames.iter().filter(|(fr, _)| *fr != Frame::TelBatch);
        frames.map(|&(fr, baud)| (fr, super::spin_extra(baud)))
    };
    let max = host().map(|(fr, spin)| super::frame(fr) + spin).max();
    let mean = host().map(|(fr, spin)| super::frame_mean(fr) + spin).max();
    f(load_row(
        Component::HostFrame,
        b.host,
        max.unwrap_or(0),
        mean.unwrap_or(0),
    ));
    let tel = Frame::TelBatch;
    f(load_row(
        Component::TelFrame,
        b.tel,
        super::frame(tel),
        super::frame_mean(tel),
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

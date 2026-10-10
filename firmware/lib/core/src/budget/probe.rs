//! The budget probe's records and their check. A bench build keeps one
//! [`KernelProbe`] (written only by the kernel tick) and one [`BusProbe`]
//! (written only at the bus level); the hardware test decodes both from a
//! debug-link dump and holds them against the table.

use super::{Body, CLOCK_HZ, Events, Frame, PHASES, Regime, TICKS_PER_US};

/// A body longer than this is a debugger halt or a flash save, not a
/// budgeted body: counted, kept out of every statistic.
pub const HALT_TICKS: u32 = 1000 * TICKS_PER_US;

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

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct KernelProbe {
    /// Ticks with no TEL work and no refresh, by phase.
    pub tick: [Load; PHASES],
    /// Longest TEL tick that banked nothing, by phase.
    pub tel_max: [u32; PHASES],
    /// Longest banking tick, by phase.
    pub bank_max: [u32; PHASES],
    /// TEL ticks that banked nothing, all phases.
    pub tel: Load,
    /// CONTROL ticks that rebuilt the configuration.
    pub refresh: Load,
    /// The ten ticks of each medium period, summed.
    pub period: Load,
    /// Every tick but the boot tick, by body.
    pub hist: [u32; HIST_BINS],
    pub halts: u32,
    /// Every kernel body so far: what a bus body subtracts.
    pub busy: u32,
    period_acc: u32,
}

impl KernelProbe {
    pub const ZERO: Self = Self {
        tick: [Load::ZERO; PHASES],
        tel_max: [0; PHASES],
        bank_max: [0; PHASES],
        tel: Load::ZERO,
        refresh: Load::ZERO,
        period: Load::ZERO,
        hist: [0; HIST_BINS],
        halts: 0,
        busy: 0,
        period_acc: 0,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// One kernel tick of `body` ticks that ran `phase` and carried `ev`.
    /// The boot tick counts toward `busy` only.
    #[inline(always)]
    pub fn tick(&mut self, body: u32, phase: usize, ev: Events, boot: bool) {
        self.busy = self.busy.wrapping_add(body);
        if boot {
            return;
        }
        if body > HALT_TICKS {
            self.halts = self.halts.wrapping_add(1);
            self.period_acc = 0;
            return;
        }
        if phase == 0 && self.period_acc != 0 {
            self.period.add(self.period_acc);
            self.period_acc = 0;
        }
        self.period_acc = self.period_acc.wrapping_add(body);
        let bin = ((body / HIST_BIN) as usize).min(HIST_BINS - 1);
        if let Some(h) = self.hist.get_mut(bin) {
            *h = h.wrapping_add(1);
        }
        if ev.refresh {
            self.refresh.add(body);
        } else if ev.bank {
            if let Some(m) = self.bank_max.get_mut(phase) {
                *m = (*m).max(body);
            }
        } else if ev.tel {
            self.tel.add(body);
            if let Some(m) = self.tel_max.get_mut(phase) {
                *m = (*m).max(body);
            }
        } else if let Some(l) = self.tick.get_mut(phase) {
            l.add(body);
        }
    }

    /// A body on the kernel's vector that is not a tick (shunt burst events).
    #[inline(always)]
    pub fn other(&mut self, body: u32) {
        self.busy = self.busy.wrapping_add(body);
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        Some(Self {
            tick: core::array::from_fn(|_| r.load()),
            tel_max: core::array::from_fn(|_| r.word()),
            bank_max: core::array::from_fn(|_| r.word()),
            tel: r.load(),
            refresh: r.load(),
            period: r.load(),
            hist: core::array::from_fn(|_| r.word()),
            halts: r.word(),
            busy: r.word(),
            period_acc: r.word(),
        })
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct BusProbe {
    /// Bus vector bodies less the kernel time inside them, by [`Body`].
    pub body: [Load; Body::ALL.len()],
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
    acc_tel: u32,
    replied: u32,
    saw_break: u32,
    saw_start: u32,
    saw_dispatch: u32,
}

impl BusProbe {
    pub const ZERO: Self = Self {
        body: [Load::ZERO; Body::ALL.len()],
        host: Load::ZERO,
        tel: Load::ZERO,
        halts: 0,
        breaks: 0,
        refires: 0,
        parks: 0,
        rises: 0,
        acc: 0,
        acc_tel: 0,
        replied: 0,
        saw_break: 0,
        saw_start: 0,
        saw_dispatch: 0,
    };

    pub const WORDS: usize = core::mem::size_of::<Self>() / 4;

    /// The running body is a break wake that found a break.
    #[inline(always)]
    pub fn mark_break(&mut self) {
        self.saw_break = 1;
    }

    /// The running body started an own frame (its break is on the wire).
    #[inline(always)]
    pub fn mark_start(&mut self) {
        self.saw_start = 1;
    }

    /// The running body dispatched a frame.
    #[inline(always)]
    pub fn mark_dispatch(&mut self) {
        self.saw_dispatch = 1;
    }

    /// A bus body of `excl` ticks ends. A stager body that started no frame
    /// books as [`Body::TelPoll`], a break wake that dispatched as
    /// [`Body::Deadline`].
    #[inline(always)]
    pub fn body(&mut self, excl: u32, b: Body) {
        let brk = core::mem::take(&mut self.saw_break) != 0;
        let started = core::mem::take(&mut self.saw_start) != 0;
        let dispatched = core::mem::take(&mut self.saw_dispatch) != 0;
        if excl > HALT_TICKS {
            self.halts = self.halts.wrapping_add(1);
            return;
        }
        let b = match b {
            Body::TelStage if !started => Body::TelPoll,
            Body::BreakWake if dispatched => Body::Deadline,
            b => b,
        };
        if let Some(l) = self.body.get_mut(b as usize) {
            l.add(excl);
        }
        if brk {
            self.fold(0);
            self.replied = 0;
        }
        if started {
            if self.replied != 0 {
                self.fold(1);
            }
            self.replied = 1;
        }
        self.acc = self.acc.wrapping_add(excl);
    }

    /// The record with its open frame closed: read after the traffic, the
    /// last frame has run every body it will.
    pub fn closed(mut self) -> Self {
        self.fold(0);
        self
    }

    /// Close the frame accumulated so far; the next one is TEL if `tel`.
    #[inline(always)]
    fn fold(&mut self, tel: u32) {
        if self.acc != 0 {
            if self.acc_tel != 0 {
                self.tel.add(self.acc);
            } else {
                self.host.add(self.acc);
            }
        }
        self.acc = 0;
        self.acc_tel = tel;
    }

    /// Decode a dump, little-endian words in field order.
    pub fn from_words(w: &[u32]) -> Option<Self> {
        if w.len() != Self::WORDS {
            return None;
        }
        let mut r = Words(w);
        Some(Self {
            body: core::array::from_fn(|_| r.load()),
            host: r.load(),
            tel: r.load(),
            halts: r.word(),
            breaks: r.word(),
            refires: r.word(),
            parks: r.word(),
            rises: r.word(),
            acc: r.word(),
            acc_tel: r.word(),
            replied: r.word(),
            saw_break: r.word(),
            saw_start: r.word(),
            saw_dispatch: r.word(),
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
}

/// One measured window: what ran, and the frames the host sent (each with
/// its rate).
pub struct Scenario<'a> {
    pub regime: Regime,
    pub tel: bool,
    pub frames: &'a [(Frame, u32)],
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Component {
    Tick(usize),
    TelTick(usize),
    BankTick(usize),
    Refresh,
    /// The TEL ticks' mean over the plain ticks' (mean only).
    TelEncode,
    Body(Body),
    HostFrame,
    TelFrame,
}

/// A component's window against its budget; `n` and `mean` are 0 where the
/// probe keeps only the maximum, `mean_budget` 0 where none is set.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Row {
    pub component: Component,
    pub n: u32,
    pub mean: u32,
    pub max: u32,
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
    let r = s.regime;
    let tel = Events {
        tel: true,
        ..Events::default()
    };
    let bank = Events {
        tel: true,
        bank: true,
        ..Events::default()
    };
    let ticks = k.tick.iter().zip(k.tel_max).zip(k.bank_max);
    for (p, ((t, tel_max), bank_max)) in ticks.enumerate() {
        let budget = super::kernel_tick(r, p, Events::default());
        let mean = super::KERNEL_MEAN[r as usize][p];
        f(load_row(Component::Tick(p), *t, budget, mean));
        f(max_row(
            Component::TelTick(p),
            tel_max,
            super::kernel_tick(r, p, tel),
        ));
        f(max_row(
            Component::BankTick(p),
            bank_max,
            super::kernel_tick(r, p, bank),
        ));
    }
    let refresh = Events {
        refresh: true,
        tel: s.tel,
        bank: s.tel,
    };
    f(load_row(
        Component::Refresh,
        k.refresh,
        super::kernel_tick(r, 0, refresh),
        0,
    ));
    let plain = k.tick.iter().fold((0u64, 0u64), |(n, sum), l| {
        (n + l.n as u64, sum + l.sum as u64)
    });
    let plain_mean = plain.1.checked_div(plain.0).unwrap_or(0) as u32;
    f(Row {
        component: Component::TelEncode,
        n: k.tel.n,
        mean: k.tel.mean().saturating_sub(plain_mean),
        max: 0,
        budget: 0,
        mean_budget: super::TEL_ENCODE_MEAN[r as usize],
    });
    let slowest = s
        .frames
        .iter()
        .map(|&(_, baud)| baud)
        .min()
        .unwrap_or(super::BUDGET_BAUD_HZ);
    for (body, l) in Body::ALL.into_iter().zip(b.body) {
        let spin = match body {
            Body::Deadline | Body::TelStage => super::spin_extra(slowest),
            _ => 0,
        };
        let budget = super::body(body) + spin;
        let mean = super::body_mean(body) + spin;
        f(load_row(Component::Body(body), l, budget, mean));
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
        budget,
        mean_budget: 0,
    }
}

/// `ticks` in us, for reports.
pub fn us(ticks: u32) -> f32 {
    ticks as f32 * 1e6 / CLOCK_HZ as f32
}

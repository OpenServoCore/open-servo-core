//! Timing budgets: the most CPU, in HCLK ticks, that each kernel phase, bus
//! vector body and frame may take on the chip. One table for three users:
//! the bench probe records the same components (`probe`), the hardware test
//! `budgets` fails a window whose maximum or mean exceeds its budget, and
//! the DES charges these values as its costs. While that test is green, the
//! DES runs no component cheaper than silicon.
//!
//! Each value is M (measured by the budget probe, the larger of two runs plus
//! 3%, at least 1 us; means plus 5%) or E (estimated, from the named
//! instrument plus the stated headroom). Kernel values are whole tick
//! bodies, entry stamp to exit stamp, by drive arm and medium phase; bus
//! values exclude the kernel time that preempted them. Interrupt entry and
//! exit fall outside every stamp: [`ENTRY_EXIT`] carries them for the DES.

pub mod probe;

#[cfg(test)]
mod tests;

use crate::kernel::DECIM_MED;

/// The clock every budget counts: the chip's HCLK (the chip asserts it).
pub const CLOCK_HZ: u32 = 48_000_000;
pub const TICKS_PER_US: u32 = CLOCK_HZ / 1_000_000;

/// Kernel tick rate, one ADC scan per PWM period (the chip asserts it).
pub const TICK_HZ: u32 = 20_000;
pub const TICK_PERIOD: u32 = CLOCK_HZ / TICK_HZ;

/// Medium phases per period; index = `kernel::phase` (8 and 9 run the fast
/// path alone).
pub const PHASES: usize = DECIM_MED as usize;

/// The drive arm a tick ran: the closed current loop is the work that sets
/// the budget. Off, Brake, Coast and OpenLoop share the open rows. A tick
/// counts as Closed when its arm or the previous tick's was: the first
/// parked tick measures the last driven window.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Regime {
    Open,
    Closed,
}

impl Regime {
    pub const ALL: [Regime; 2] = [Regime::Open, Regime::Closed];
}

/// Kernel tick body with no event on it, per regime and phase: the longest
/// a window may show.
///
/// Open, M: the quiet windows and the parked holds' Brake ticks (drive-arm
/// tagged), per phase the larger. Closed, M: the stepping windows. SLOW,
/// E: off its due period the phase runs the fast path and a counter, taken
/// at phase 9's row; the due body is [`SLOW_DUE`].
pub const KERNEL: [[u32; PHASES]; 2] = [
    [945, 1332, 1512, 1155, 762, 1182, 807, 1038, 684, 807],
    [1885, 2247, 2176, 2062, 2083, 2058, 1688, 1960, 1596, 1688],
];

/// The same bodies' mean over a window: the most it may show.
///
/// Open, M: the hold windows (the larger per phase than quiet), CONTROL
/// capped at its maximum. Closed, E: the stepping windows' means x 1.10,
/// the Closed ticks' mean over the window's (1628.9 / 1486.7, drive-arm
/// tagged stepping windows).
pub const KERNEL_MEAN: [[u32; PHASES]; 2] = [
    [945, 1222, 1195, 1044, 753, 1049, 723, 919, 661, 674],
    [1811, 2182, 2115, 1908, 1986, 1911, 1557, 1771, 1482, 1498],
];

/// A window's mean is judged only over this many bodies.
pub const MEAN_MIN_BODIES: u32 = 100;

/// The CONTROL tick that rebuilds the kernel's configuration snapshot, on
/// top of [`KERNEL`]; once per period at most, after a configuration
/// write. M: the refresh tick at rest, 1679, over the quiet CONTROL mean
/// 849.
pub const REFRESH: u32 = 855;

/// Every tick of a TEL burst (sample build and encode), on top of [`KERNEL`],
/// per regime. Open, E: TEL-active non-banking ticks 30.6 us less the 1.9
/// us staging poll and the 17.9 us quiet mean, plus 1.2 us. Closed, M: the
/// TEL ticks' mean over the plain ticks' while stepping, 672-730.
pub const TEL_ENCODE: [u32; 2] = [576, 760];

/// Its mean over a window: the TEL ticks' mean less the plain ticks'. Open,
/// M: 511-512. Closed: as [`TEL_ENCODE`].
pub const TEL_ENCODE_MEAN: [u32; 2] = [538, 760];

/// The tick that banks a TEL batch, on top of [`TEL_ENCODE`]. E: v2 banking
/// ticks 32.7 us against 30.6 us non-banking, plus 2 us.
pub const TEL_BANK: u32 = 192;

/// The SLOW block on its due tick (phase SLOW, every DECIM_SLOW periods),
/// on top of the phase's row. M: the stepping windows' SLOW maximum 2268
/// over the phase's mean 1347.
pub const SLOW_DUE: u32 = 949;

/// The due tick that closes a thermal reference window (two reciprocal
/// divides, the install, the cold check) in place of [`SLOW_DUE`]: at most
/// once per seat, 190 gated SLOW runs after it opens. E: the close path
/// priced 1700-1900 over the fast path; no loaded seat has run on the
/// bench.
pub const SLOW_CLOSE: u32 = 1900;

/// The staged close's halves at phases 8 and 9 of the due period, once the
/// close moves off the SLOW tick. E: the install and the cold check priced
/// at about 400 and 250, plus 10%.
pub const THERM_INSTALL: u32 = 440;
pub const THERM_COLD: u32 = 275;

/// The tick-load counters' close every [`HEALTH_CLOSE_TICKS`] ticks, and
/// the catch-up it adds after a lost tick. E: priced on the executed path.
pub const HEALTH_CLOSE: u32 = 93;
pub const HEALTH_CLOSE_TICKS: u32 = 16;
pub const CATCH_UP: u32 = 111;

/// Interrupt entry and exit around a stamped body: trampoline and prologue
/// to the entry stamp, epilogue and exit after the exit stamp. M: 37 + 43
/// (interrupt timing images, two runs, min == max).
pub const ENTRY_EXIT: u32 = 80;

/// A hold runs Closed this long after its goal moves, then parks. M: the
/// longest Closed episode after a 600-count step, 2899 / 3396 ticks
/// (drive-arm tagged hold windows).
pub const HOLD_APPROACH_TICKS: u32 = 3400;

/// The window lengths gamma(k) is kept for, in kernel ticks.
pub const WINDOW_TICKS: [usize; 4] = [2, 4, 10, 20];

/// gamma(k) on the bench by regime: the most the bodies of k consecutive
/// ticks took, 0 where no fill has measured it. k = 10, M lower bound: the
/// aligned medium periods' maximum, quiet and stepping.
pub const WINDOW_LOAD: [[u32; WINDOW_TICKS.len()]; 2] = [[0, 0, 8667, 0], [0, 0, 17640, 0]];

/// Bus vector bodies.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Body {
    /// Break wake (frame resolve, chain observe, deadline arm).
    BreakWake,
    /// TX arm completion.
    TxDone,
    /// Deadline mux (framer, dispatch, verdict, commit, trigger).
    Deadline,
    /// The TEL stager body that seals and starts a frame.
    TelStage,
    /// The TEL stager body that starts nothing.
    TelPoll,
}

impl Body {
    pub const ALL: [Body; 5] = [
        Body::BreakWake,
        Body::TxDone,
        Body::Deadline,
        Body::TelStage,
        Body::TelPoll,
    ];
}

/// Longest body of each bus vector at [`BUDGET_BAUD_HZ`], any traffic.
///
/// BreakWake, E: 18.2 us max (bus-on-top spine probe) plus 1.8 us. TxDone,
/// E: 11.1 us TEL arm max (same probe) plus 0.9 us. Deadline, E: the 62.5 us
/// READ dispatch body (v1 probe) plus a goal WRITE's rules and commit (31 us)
/// plus 16.5 us. TelStage, E: the stager max 22.5 us (v2 probe, SW stager)
/// plus 1.5 us. TelPoll, E: 0.8 us mean derived from the same window, plus
/// 1.2 us.
pub const fn body(b: Body) -> u32 {
    match b {
        Body::BreakWake => 960,
        Body::TxDone => 576,
        Body::Deadline => 5280,
        Body::TelStage => 1152,
        Body::TelPoll => 96,
    }
}

/// The same bodies' mean over a window.
///
/// BreakWake, E: 16.0 us (v1 probe, polling ladder). TxDone, E: 4.7 us per
/// arm (two-arm replies, bench probe) plus 1.3 us. Deadline, E: 82.1 us over
/// two bodies per READ (v1 probe) plus 4 us. TelStage, E: 20 us, the staging
/// share of the v2 SW stager window. TelPoll, E: 0.8 us, its polling share.
pub const fn body_mean(b: Body) -> u32 {
    match b {
        Body::BreakWake => 768,
        Body::TxDone => 288,
        Body::Deadline => 2160,
        Body::TelStage => 960,
        Body::TelPoll => 40,
    }
}

/// Frame types, each the bus CPU of one exchange: every bus body from its
/// break wake to the next frame's (a TEL batch: from its start to the next).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Frame {
    Ping,
    /// READ of 32 bytes.
    Read32,
    /// READ of the largest payload.
    Read240,
    /// WRITE of `goal_position` (the rule-heavy hot-loop register).
    Write,
    /// GREAD answered in slot 1 behind a silent slot 0 (reclaim).
    GreadSlot,
    /// One six-field TEL frame (16 rows) with its arms and the polls between.
    TelBatch,
}

impl Frame {
    pub const ALL: [Frame; 6] = [
        Frame::Ping,
        Frame::Read32,
        Frame::Read240,
        Frame::Write,
        Frame::GreadSlot,
        Frame::TelBatch,
    ];

    /// TX arms a reply streams in, each one TX-done body.
    pub const REPLY_ARMS: u32 = 2;
}

/// Bus CPU per frame at [`BUDGET_BAUD_HZ`].
///
/// Read32, E: 118.9 us (v1 probe: wake 16.0, deadline 82.1, three arms 20.8)
/// plus 6 us. Ping, E: the READ less its read dispatch (~5 us). Read240, E:
/// Read32 plus 5 us (payload streams by DMA). Write, E: Read32 plus the
/// goal rules and commit (37 us) plus 8 us. GreadSlot, E: Read32 plus the
/// chain bookkeeping and reclaim deadline (15 us). TelBatch, E: stage 22.5,
/// two arms, fifteen polls (v2 probe), plus 10 us.
pub const fn frame(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5520,
        Frame::Read32 => 6000,
        Frame::Read240 => 6240,
        Frame::Write => 8160,
        Frame::GreadSlot => 6720,
        Frame::TelBatch => 3360,
    }
}

/// The same frames' mean over a window. E: Read32 112 us (the v1 118.9 us
/// less the third arm, which two-arm replies dropped); Ping, Read240, Write
/// and GreadSlot shifted from it as in [`frame`]; TelBatch: stage 20, two
/// arms, fifteen polls.
pub const fn frame_mean(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5136,
        Frame::Read32 => 5376,
        Frame::Read240 => 5616,
        Frame::Write => 7152,
        Frame::GreadSlot => 6096,
        Frame::TelBatch => 2112,
    }
}

/// The rate the bus budgets are set at (the fleet rate).
pub const BUDGET_BAUD_HZ: u32 = 3_000_000;

/// The law break the trigger spins out: a bracketed 9-bit 0x00 through its
/// stop bit (protocol sec 3).
pub const LAW_BREAK_BIT_TIMES: u32 = 11;

/// The extra spin a trigger body (and so its frame) takes at `baud_hz` over
/// the [`BUDGET_BAUD_HZ`] budget. Host-side: it divides.
pub const fn spin_extra(baud_hz: u32) -> u32 {
    let spin = LAW_BREAK_BIT_TIMES * CLOCK_HZ / baud_hz;
    let at_budget = LAW_BREAK_BIT_TIMES * CLOCK_HZ / BUDGET_BAUD_HZ;
    spin.saturating_sub(at_budget)
}

/// Kernel tick budget: the base for `regime` and `phase`, with the events
/// the tick carried.
pub const fn kernel_tick(regime: Regime, phase: usize, ev: Events) -> u32 {
    let r = regime as usize;
    let mut t = KERNEL[r][phase % PHASES];
    if ev.refresh {
        t += REFRESH;
    }
    if ev.close {
        t += SLOW_CLOSE;
    } else if ev.slow {
        t += SLOW_DUE;
    }
    if ev.tel {
        t += TEL_ENCODE[r];
    }
    if ev.bank {
        t += TEL_BANK;
    }
    t
}

/// What a kernel tick carried beyond its phase.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Events {
    pub refresh: bool,
    pub tel: bool,
    pub bank: bool,
    /// The SLOW block ran; `close`: and installed a thermal reference.
    pub slow: bool,
    pub close: bool,
}

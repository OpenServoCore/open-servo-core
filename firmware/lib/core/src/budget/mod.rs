//! Timing budgets: the most CPU, in HCLK ticks, that each kernel phase, bus
//! vector body and frame may take on the chip. One table for three users:
//! the bench probe records the same components (`probe`), the hardware test
//! `budgets` fails a window whose maximum or mean exceeds its budget, and
//! the DES charges these values as its costs. While that test is green, the
//! DES runs no component cheaper than silicon.
//!
//! Each value is M (measured by the budget probe, the larger of two runs plus
//! 3% with at least 1 us of margin; means plus 5%) or E (estimated, from the
//! named instrument plus the stated headroom). Kernel values are whole tick
//! bodies, entry stamp to exit stamp, by drive arm and medium phase; bus
//! values exclude the kernel vectors that preempted them, each with its
//! interrupt entry and exit at [`ENTRY_EXIT`]. Interrupt entry and exit fall
//! outside every stamp: [`ENTRY_EXIT`] carries them for the DES.
//!
//! A row changes only with its evidence: a raised M comes from the probe on
//! the production image, a raised E names its new instrument, and the DES
//! pins re-run against the new table in the same change. A red window or
//! pin is never cleared by raising a row alone.

pub mod probe;

#[cfg(test)]
mod tests;

use crate::kernel::{DECIM_MED, phase};

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
/// M, drive-arm tagged on this image, a closing tick booked apart:
/// Open over the quiet, hold, parked hold and stepping windows' Open ticks,
/// Closed over the same windows' Closed ticks. Phase SLOW off its due
/// period is a plain tick; the due body is [`SLOW_DUE`].
pub const KERNEL: [[u32; PHASES]; 2] = [
    [818, 1070, 890, 932, 641, 956, 641, 806, 599, 593],
    [1403, 1610, 1684, 1469, 1613, 1520, 1181, 1346, 1139, 1127],
];

/// The same bodies' mean over a window: the most it may show.
///
/// M: per phase the largest window mean of the same windows, Closed from
/// the stepping windows.
pub const KERNEL_MEAN: [[u32; PHASES]; 2] = [
    [805, 1062, 867, 915, 622, 951, 611, 795, 578, 566],
    [1234, 1528, 1495, 1307, 1454, 1367, 994, 1175, 956, 948],
];

/// A window's mean is judged only over this many bodies.
pub const MEAN_MIN_BODIES: u32 = 100;

/// The two stages that rebuild the kernel's configuration snapshot after a
/// configuration write, on top of [`KERNEL`], each at most once per period
/// and in place of the thermometer's stage on that tick: the fast half, the
/// thermometer's config and its bound at phase THERM_INSTALL; the rest of
/// the medium half and the floors at THERM_SIDE. REFRESH_1, E: the stage
/// tick over the same tick without the write, 486, priced on the release
/// image (fetch model on the emulated trace, quiet and stepping alike),
/// plus 10%; its M from one tick per run, 525, does not lower it.
/// REFRESH_2, M, above its E 462: the stage tick over the window's plain
/// maximum at the phase, one tick per run in each 1M window.
pub const REFRESH_1: u32 = 535;
pub const REFRESH_2: u32 = 474;

/// Every tick of a TEL burst (row gate, sample build and the out-of-line
/// row), on top of [`KERNEL`], per regime, the widest row 3M admits (11 B).
/// M for the six-field 9 B rows plus E for the widest row, 18: priced at
/// 426 against six packed fields' 408. Six-field M: the TEL ticks' maximum
/// over the plain maximum, drive-arm tagged, the largest phase: Open 570
/// (tel still), Closed 525 (stepping).
pub const TEL_ENCODE: [u32; 2] = [636, 591];

/// Its mean over a window: the TEL ticks' mean less the plain ticks'. M
/// for six fields, the largest window mean, Open 438 (stepping), Closed 389
/// (stepping), plus the same 18.
pub const TEL_ENCODE_MEAN: [u32; 2] = [478, 427];

/// The tick that banks a TEL batch, on top of [`TEL_ENCODE`], any row
/// width. M: the banking ticks' maximum over the TEL ticks', the largest
/// phase of either regime, 150 (Open, stepping).
pub const TEL_BANK: u32 = 198;

/// The TX lane a TEL burst's kernel owns (protocol sec 5.6), on top of
/// the TEL tick. E: each priced on the release image's executed path with
/// the fetch model (per straight run 3 HCLK a flash word spanned and 3 for
/// the taken transfer into it, or 1 an instruction and 2 a load or store
/// when that is more), plus 10%. TEL_POLL: the dearest tick with no lane
/// step, a frame's end that starts nothing (90 priced; the gate and the TX
/// poll alone 51-66). The lane takes at most one step a tick, never on a
/// tick that lands a head or a trailer, each over the poll: TEL_FEED, a
/// frame's start (CRC engine reset, the written bytes' feed, the break)
/// on the tick the frame before it ended (321 with the poll), or the feed
/// of the rest after a bank (207); TEL_ARM, the DMA arm behind the break
/// with the LAST frame's resume cursor (270); TEL_PATCH, the CRC read, its
/// odd fold and the 2 B store (240). TEL_HEAD: row 0 over a plain row, the
/// frame header, head and start rule (90).
pub const TEL_POLL: u32 = 99;
pub const TEL_HEAD: u32 = 99;
pub const TEL_FEED: u32 = 255;
pub const TEL_ARM: u32 = 198;
pub const TEL_PATCH: u32 = 165;

const fn max(a: u32, b: u32) -> u32 {
    if a > b { a } else { b }
}

/// The most a TEL tick that banks nothing carries beyond its row: the poll
/// and one lane step, or row 0's head on a tick the lane leaves be; a
/// `held` tick (a SLOW or thermometer stage) takes no step.
pub const fn tel_lane(held: bool) -> u32 {
    let step = if held {
        0
    } else {
        max(max(TEL_FEED, TEL_ARM), TEL_PATCH)
    };
    TEL_POLL + max(TEL_HEAD, step)
}

/// The lane's share of a TEL tick over a window, for the mean budgets: a
/// 16-row frame's polls and steps spread over its ticks. E: priced (a tick
/// with no step 51-66, 21 on the head and bank ticks; over it a start 231,
/// the arm 210, the rest's feed 156, the patch 189, the head 90) and
/// averaged, 107, plus 10%.
pub const TEL_LANE_MEAN: u32 = 118;

/// The SLOW block's first stage on its due tick (phase SLOW, every
/// DECIM_SLOW periods): the permit lease, undervoltage and the
/// thermometer's step on the tick's window, on top of the phase's row, per
/// regime. E: the dearest path, a tracked seat (any drive that holds
/// current), priced at 405 over the plain tick, plus 10%.
pub const SLOW_DUE: [u32; 2] = [446, 446];

/// The second stage on the next period's SLOW phase: the thermometer's
/// temperature, its seat rules and overtemperature. E: a tracked seat,
/// priced at 330 over the plain tick, plus 10%.
pub const SLOW_FINISH: u32 = 363;

/// The finish that freezes a thermal reference window, in place of
/// [`SLOW_FINISH`]: at most once per seat, 190 gated SLOW runs after it
/// opens. E: priced at 330 over the plain tick, plus 10%.
pub const SLOW_CLOSE: u32 = 363;

/// The thermometer's stages on the spare phases (`kernel::phase`), on top
/// of their rows. THERM_INSTALL: a frozen window's install the period
/// after its finish (two reciprocal divides). THERM_SIDE: the NTC the
/// period before a SLOW tick (a conversion and its bound once a second),
/// or the derate and a cold check the period after a finish. THERM_INSTALL, E: priced at 531 over the plain
/// tick, plus 10%; no bench window has run one. THERM_SIDE, M, above its E
/// 631: the stage tick over the window's plain maximum at the phase, 591
/// (hold); in the TEL windows over the TEL tick's maximum, at most 402,
/// since [`kernel_tick`] stacks the TEL row on it.
pub const THERM_INSTALL: u32 = 585;
pub const THERM_SIDE: u32 = 639;

/// The tick-load counters' close every [`HEALTH_CLOSE_TICKS`] ticks, on top
/// of the tick it lands on, and the catch-up it adds after a lost tick.
/// HEALTH_CLOSE, M: the closing ticks' maximum over the plain maximum (in
/// the TEL windows over the TEL ticks'), the largest phase of either
/// regime, 260 (Open p5, tel still; 150 outside the TEL windows). The
/// probe books a close ahead of a bank, so that cell may hold a bank too.
/// CATCH_UP, E: priced on the executed path.
pub const HEALTH_CLOSE: u32 = 308;
pub const HEALTH_CLOSE_TICKS: u32 = 2 * PHASES as u32;
pub const CATCH_UP: u32 = 111;

/// The phase every tick-load close lands on: the lightest Closed phase that
/// carries no SLOW or thermometer stage and no refresh.
pub const HEALTH_CLOSE_PHASE: u8 = phase::PUBLISH;

const _: () = assert!((HEALTH_CLOSE_TICKS as usize).is_multiple_of(PHASES));

/// Whether executed tick `tick` closes a tick-load window. The chip counts
/// executed ticks from the boot tick, which runs phase CONTROL, and a shunt
/// burst runs no tick and keeps the count, so tick `tick` runs phase
/// `tick` modulo [`PHASES`]; the first close only sets the mark.
pub const fn health_close_at(tick: u64) -> bool {
    tick % HEALTH_CLOSE_TICKS as u64 == HEALTH_CLOSE_PHASE as u64
}

/// Whether a tick-load close can land on `phase` ([`health_close_at`]).
pub const fn health_close_lands(phase: usize) -> bool {
    phase % PHASES == HEALTH_CLOSE_PHASE as usize
}

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
/// ticks took. M: the quiet windows (Open) and the stepping windows
/// (Closed).
pub const WINDOW_LOAD: [[u32; WINDOW_TICKS.len()]; 2] =
    [[1951, 3470, 8216, 15705], [3450, 6519, 14522, 27944]];

/// The same with six fields streaming at 3M. M: TEL still (Open) and TEL
/// stepping (Closed).
pub const WINDOW_LOAD_TEL: [[u32; WINDOW_TICKS.len()]; 2] =
    [[2894, 5246, 12480, 24296], [4318, 8067, 18419, 35634]];

/// Bus vector bodies.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Body {
    /// Break wake (frame resolve, chain observe, deadline arm).
    BreakWake,
    /// TX arm completion.
    TxDone,
    /// Deadline mux (framer, dispatch, verdict, commit, trigger).
    Deadline,
}

impl Body {
    pub const ALL: [Body; 3] = [Body::BreakWake, Body::TxDone, Body::Deadline];
}

/// Longest body of each bus vector at [`BUDGET_BAUD_HZ`], any traffic.
///
/// M: each body's net maximum over every budget window, a 1M window's
/// Deadline less [`spin_extra`]. Deadline: the goal WRITE at 1M; BreakWake:
/// the first wake after a burst's LAST, which takes the TX lane back from
/// the kernel (997, TEL stepping); the rest from the TEL stepping window.
pub const fn body(b: Body) -> u32 {
    match b {
        Body::BreakWake => 1045,
        Body::TxDone => 426,
        Body::Deadline => 7116,
    }
}

/// The same bodies' mean over a window.
///
/// M: each body's largest window mean (at least [`MEAN_MIN_BODIES`]
/// bodies), the same spin rule. BreakWake: the GREAD slot window; Deadline:
/// the goal WRITE; TxDone: TEL stepping, with the stager wake on release
/// (316).
pub const fn body_mean(b: Body) -> u32 {
    match b {
        Body::BreakWake => 803,
        Body::TxDone => 332,
        Body::Deadline => 5770,
    }
}

/// Frame types, each the bus CPU of one exchange: every bus body from its
/// break wake to the next frame's.
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
}

impl Frame {
    pub const ALL: [Frame; 5] = [
        Frame::Ping,
        Frame::Read32,
        Frame::Read240,
        Frame::Write,
        Frame::GreadSlot,
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
/// chain bookkeeping and reclaim deadline (15 us).
pub const fn frame(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5520,
        Frame::Read32 => 6000,
        Frame::Read240 => 6240,
        Frame::Write => 8160,
        Frame::GreadSlot => 6720,
    }
}

/// The same frames' mean over a window. E: Read32 112 us (the v1 118.9 us
/// less the third arm, which two-arm replies dropped); Ping, Read240, Write
/// and GreadSlot shifted from it as in [`frame`].
pub const fn frame_mean(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5136,
        Frame::Read32 => 5376,
        Frame::Read240 => 5616,
        Frame::Write => 7152,
        Frame::GreadSlot => 6096,
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
        t += refresh(phase);
    }
    if ev.slow {
        t += SLOW_DUE[r];
    }
    if ev.close {
        t += SLOW_CLOSE;
    } else if ev.finish {
        t += SLOW_FINISH;
    }
    if ev.install {
        t += THERM_INSTALL;
    }
    if ev.side {
        t += THERM_SIDE;
    }
    if ev.tel {
        // The lane leaves a banking tick be (its gate is under the poll),
        // and a stage tick, so the dearest TEL tick is the bank's or a lane
        // step's, a stage tick's the bank's or a head's. A one-row batch's
        // head rides in TEL_BANK, whose M bank cost the trailer alone now
        // undercuts (priced 72, the head 90).
        let held = ev.slow || ev.finish || ev.close || ev.install || ev.side;
        let bank = if ev.bank { TEL_BANK + TEL_POLL } else { 0 };
        t += TEL_ENCODE[r] + max(tel_lane(held), bank);
    }
    if ev.health {
        t += HEALTH_CLOSE;
    }
    t
}

/// The refresh stage row a tick at `phase` carries.
pub const fn refresh(phase: usize) -> u32 {
    if phase % PHASES == phase::THERM_INSTALL as usize {
        REFRESH_1
    } else {
        REFRESH_2
    }
}

/// What a kernel tick carried beyond its phase.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Events {
    pub refresh: bool,
    pub tel: bool,
    pub bank: bool,
    /// The SLOW block's first stage ran; `finish`: its second, and
    /// `close`: that froze a thermal reference window.
    pub slow: bool,
    pub finish: bool,
    pub close: bool,
    /// The thermometer's stages on phases THERM_INSTALL and THERM_SIDE.
    pub install: bool,
    pub side: bool,
    /// The tick-load counters closed a window.
    pub health: bool,
}

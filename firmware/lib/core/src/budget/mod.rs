//! Timing budgets: the most CPU, in HCLK ticks, that each kernel phase, bus
//! vector body and frame may take on the chip. One table for three users:
//! the bench probe records the same components (`probe`), the hardware test
//! `budgets` fails a window whose maximum or mean exceeds its budget, and
//! the DES charges these values as its costs. While that test is green, the
//! DES runs no component cheaper than silicon.
//!
//! Each value is M (measured by the budget probe, the largest of its runs
//! plus 3% with at least 1 us of margin; means plus 5%) or E (estimated,
//! from the named instrument plus the stated headroom). Kernel values are
//! whole tick bodies, entry stamp to exit stamp, by drive arm and medium
//! phase; bus values exclude the kernel vectors that preempted them, each
//! with its interrupt entry and exit at [`ENTRY_EXIT`]. Interrupt entry and
//! exit fall outside every stamp: [`ENTRY_EXIT`] carries them for the DES.
//!
//! A row changes only with its evidence: a raised M comes from the probe on
//! the production image, a raised E names its new instrument, and the DES
//! pins re-run against the new table in the same change. A red window or
//! pin is never cleared by raising a row alone.
//!
//! Every kernel row, maximum and mean, carries [`LAYOUT`] on top. The
//! allowance counts against the hard budgets (the tick period, the
//! turnaround deadline, every DES pin at budget, the stack reserve): fit is
//! judged with it, and it is never a reason to exceed one.

pub mod probe;

#[cfg(test)]
mod tests;

use osc_protocol::wire::LAW_BREAK_BIT_TIMES;

use crate::kernel::{DECIM_MED, TICK_HZ, phase};

/// The clock every budget counts: the chip's HCLK (the chip asserts it).
pub const CLOCK_HZ: u32 = 48_000_000;
pub const TICKS_PER_US: u32 = CLOCK_HZ / 1_000_000;

pub const TICK_PERIOD: u32 = CLOCK_HZ / TICK_HZ;

/// Medium phases per period; index = `kernel::phase` (8 runs the fast path
/// alone, but for a configuration take).
pub const PHASES: usize = DECIM_MED as usize;

/// The drive arm a tick ran: the closed current loop is the work that sets
/// the budget. Off, Brake and Coast share the open rows; OpenLoop has its
/// own, over the open rows on every phase and over the closed means on
/// some. A tick books its own arm or the previous tick's
/// ([`Regime::booked`]): the first parked tick measures the last driven
/// window. Event, TEL, bank and gamma(k) rows fold OpenLoop into Closed
/// ([`Regime::event_row`]).
#[repr(u8)]
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub enum Regime {
    #[default]
    Open,
    Closed,
    OpenLoop,
}

impl Regime {
    pub const ALL: [Regime; 3] = [Regime::Open, Regime::Closed, Regime::OpenLoop];
    pub const EVENT_ROWS: [Regime; 2] = [Regime::Open, Regime::Closed];

    /// The row a tick of this regime books and is budgeted on when it
    /// carries an event, a TEL row or a bank.
    pub const fn event_row(self) -> Regime {
        match self {
            Regime::OpenLoop => Regime::Closed,
            r => r,
        }
    }

    /// The regime of a tick between arms `self` and `other`: Closed over
    /// OpenLoop over Open. A tick between Closed and OpenLoop is a mode
    /// change, and the closed loop sets the budget.
    pub const fn booked(self, other: Regime) -> Regime {
        match (self, other) {
            (Regime::Closed, _) | (_, Regime::Closed) => Regime::Closed,
            (Regime::OpenLoop, _) | (_, Regime::OpenLoop) => Regime::OpenLoop,
            _ => Regime::Open,
        }
    }
}

pub const REGIMES: usize = Regime::ALL.len();
pub const EVENT_REGIMES: usize = Regime::EVENT_ROWS.len();

/// Kernel tick body with no event on it, per regime and phase: the longest
/// a window may show.
///
/// M, every gated window's ticks by drive arm, on the image the table
/// ships with, a closing tick booked apart, 5 runs: Open over the Open
/// ticks, Closed over the Closed ticks, OpenLoop over the openloop
/// window's OpenLoop ticks. A report-only window feeds no row. Each cell
/// is its measured maximum with the rule's margin; the bodies hold the
/// tick block's work and every phase's stages. The LIMITS column carries
/// [`SPEED_FOLD`] and [`UNSET_FLAG`] on top, which the measurement
/// predates.
/// Phase SLOW off its due period is a plain tick; the due body is
/// [`SLOW_DUE`].
pub const KERNEL: [[u32; PHASES]; REGIMES] = with_limits_cells([
    [684, 1002, 849, 912, 534, 915, 519, 699, 510, 537],
    [1269, 1635, 1605, 1500, 1422, 1614, 1092, 1395, 1155, 1236],
    [1254, 1611, 1125, 1446, 1089, 1455, 1062, 1269, 1053, 1134],
]);

/// The same bodies' mean over a window: the most it may show.
///
/// M: per phase the largest window mean over the same windows and ticks,
/// judged over [`MEAN_MIN_BODIES`], with the rule's margin, at most the
/// maximum's row (OpenLoop phase 5: the bodies barely vary); the LIMITS
/// column with [`SPEED_FOLD`] and [`UNSET_FLAG`], as the maximum's.
pub const KERNEL_MEAN: [[u32; PHASES]; REGIMES] = with_limits_cells([
    [618, 985, 818, 886, 498, 902, 486, 675, 476, 501],
    [1201, 1543, 1547, 1443, 1388, 1460, 1015, 1179, 1005, 1176],
    [1244, 1546, 1111, 1446, 1074, 1455, 1045, 1263, 1036, 1121],
]);

/// E, on the LIMITS column of every regime, maximum and mean: Current
/// mode's speed fold under the speed ceiling, where the measured phase
/// had none. Matched release builds, each path through the phase priced
/// as [`REFRESH`]: Current mode's longest, the stall permit's with the
/// fold binding, 26 more instructions (five loads and stores, sixteen
/// ALU, one multiply, four more taken transfers), 82.25 HCLK, with no
/// permit 80.25; Position and Velocity take the mode test and the fold's
/// one compare, 15.75, and OpenLoop 27. Torque off in Current mode runs
/// the fold too, so the Open row carries it. The longest delta plus 10%,
/// rounded up; the bench must refill the rows.
pub const SPEED_FOLD: u32 = 91;

/// E, on the LIMITS column of every regime, maximum and mean: the `UNSET`
/// bit the configuration take leaves for `limit_flags`, where the measured
/// phase composed none. Matched release builds, priced as [`REFRESH`]: one
/// load and one OR on the tail every path through the phase shares, 5.25
/// HCLK. The delta plus 10%, rounded up; the bench must refill the rows.
/// CONTROL's belt, one shift where a mode match was, prices 9.25 to 15.25
/// under the measured phase on every steady path and keeps its row.
pub const UNSET_FLAG: u32 = 6;

const fn with_limits_cells(mut rows: [[u32; PHASES]; REGIMES]) -> [[u32; PHASES]; REGIMES] {
    let mut r = 0;
    while r < REGIMES {
        rows[r][phase::LIMITS as usize] += SPEED_FOLD + UNSET_FLAG;
        r += 1;
    }
    rows
}

/// A window's mean is judged only over this many bodies.
pub const MEAN_MIN_BODIES: u32 = 100;

/// The configuration take after a write, on top of [`KERNEL`], at most once
/// per period, at phase SPARE_A: the input span's word copy, the belt and
/// the `UNSET` bit, the two speeds' and the cap's resolution, the current
/// loop's resistance, the winding estimate's config, the continuous rating
/// and its leak, the winding resistance on the new `r_q12`
/// (`Medium::r_hat`) and the terminal floor's store. It carries no TEL
/// row: a burst starts neither on SPARE_A nor while a take is pending, and
/// the host is silent through one. E, an upper bound: the take's longest
/// path on matched release disassembly is under 781 HCLK, including the
/// rating, its two log calls, the resistance's 115.25, the speeds' 34 (22
/// more instructions, two halfword stores where the span copy stored their
/// word) and the unset limits' 56.25 (the belt over both limits, the
/// `UNSET` byte and the cap's resolution: 22 more instructions on the
/// longest path, thirteen loads and stores, most of them spills, and ten
/// ALU, one taken transfer fewer); the row keeps its earlier, larger price
/// until the probe re-prices it.
/// The fetch model prices loads/stores at 3.25, ALU at 2, multiplies at
/// 10, taken transfers at 6, and call plus return at 12 HCLK.
pub const REFRESH: u32 = 824;

/// Code layout on top of every kernel row, maximum and mean: one source
/// compiles to tick bodies that differ by register allocation and block
/// placement from build to build, and between the bench image the rows
/// are measured on and the release image. E: the bench image's excess
/// over the release image's on the same change, priced per phase on the
/// release image with the fetch model (as [`TEL_POLL`]), on mean rows.
pub const LAYOUT: u32 = 35;

/// A plain tick's budget at `regime` and `phase`: its row and layout.
pub const fn tick_body(regime: Regime, phase: usize) -> u32 {
    KERNEL[regime as usize][phase % PHASES] + LAYOUT
}

/// The same plain ticks' mean over a window.
pub const fn tick_body_mean(regime: Regime, phase: usize) -> u32 {
    KERNEL_MEAN[regime as usize][phase % PHASES] + LAYOUT
}

/// Every tick of a TEL burst, on top of [`KERNEL`] and the lane: the row
/// gate, the out-of-line row (bookkeeping, the shorter chunk's CPU copy
/// and the longer chunk's DMA arm); the widest, two chunks. E, priced as
/// [`TEL_POLL`]: the row tick over the same tick with nothing owed, 153
/// with two chunks (129 with one), plus 10%. A row costs the same every
/// tick: the mean is the row.
pub const TEL_ROW: u32 = 169;
pub const TEL_ROW_MEAN: u32 = TEL_ROW;

/// The tick that banks a TEL batch, on top of [`TEL_ROW`]: the bank's
/// stores. E: 39 priced, plus 10%.
pub const TEL_BANK: u32 = 43;

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
/// out-of-line head: the frame header, the stamp and fault read from the
/// tick block, the head and the start rule (133); a burst's first row 0
/// also takes the arm from the mailbox (250).
pub const TEL_POLL: u32 = 99;
pub const TEL_HEAD: u32 = 275;
pub const TEL_FEED: u32 = 255;
pub const TEL_ARM: u32 = 198;
pub const TEL_PATCH: u32 = 165;

const fn max(a: u32, b: u32) -> u32 {
    if a > b { a } else { b }
}

/// The most a TEL tick that banks nothing carries beyond its row: the poll
/// and one lane step, or row 0's head on a tick the lane leaves be; a
/// `held` tick (a SLOW stage) takes no step.
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
/// the arm 210, the rest's feed 156, the patch 189, the head 177) and
/// averaged, 112, plus 10%.
pub const TEL_LANE_MEAN: u32 = 124;

/// The SLOW block's first stage on its due tick (phase SLOW, every
/// DECIM_SLOW periods): the permit lease, undervoltage and the two heats,
/// on top of the phase's row, per regime. M: the due bodies' maximum over
/// the phase's plain maximum that sets its row, every gated window with no
/// TEL, 5 runs, with the rule's margin: Open 608 over 471; Closed 1146
/// over 1044, the maximum from the openloop window, its OpenLoop ticks
/// booked Closed.
pub const SLOW_DUE: [u32; EVENT_REGIMES] = [185, 150];

/// The second stage on the next period's SLOW phase: the NTC's check and
/// quadratic, the two rise multiplies, the cutoff fault and the winding
/// resistance (`Medium::r_hat`: two multiplies and three stores). M as
/// [`SLOW_DUE`], over the Closed row the stage is budgeted on: 1416, the
/// openloop window's, over 1044; plus [`R_AT_TEMPERATURE`], which the
/// measurement predates.
pub const SLOW_FINISH: u32 = 420 + R_AT_TEMPERATURE;

/// E, over the measured finish: the winding resistance read at the
/// estimate's temperature over 25.00 C, where the measured stage raised
/// the stored value by the rise alone. Matched release builds, the
/// stage's common path priced as [`REFRESH`]: 13 more instructions (one
/// load, ten ALU, one fewer taken transfer, and the caller's two loads),
/// 25.75 HCLK. The delta plus 10%, rounded up; the bench must refill the
/// finish.
pub const R_AT_TEMPERATURE: u32 = 29;

/// The tick-load counters' close every `kernel::HEALTH_CLOSE_TICKS` ticks, on top
/// of the tick it lands on, and the catch-up it adds after a lost tick.
/// HEALTH_CLOSE, M: r3-refill, the closing ticks' maximum over the plain
/// maximum (in the TEL windows over the TEL ticks'), either regime, every
/// window with the report-only one, 3 runs: 228 (Closed, hold raw pos;
/// 159 in the TEL windows). The probe books a close ahead of a bank, so
/// that cell may hold a bank too. CATCH_UP, E: priced on the executed path,
/// one step. The first close after a hold-off of many periods runs the
/// step once a period owed, up to `kernel::HEALTH_LOST_MAX`, and costs
/// several hundred HCLK more; no kernel row books it when the hold-off is
/// a probe dump, which reads the kernel record before that close runs.
pub const HEALTH_CLOSE: u32 = 276;
pub const CATCH_UP: u32 = 111;

/// Interrupt entry and exit around a stamped body: trampoline and prologue
/// to the entry stamp, epilogue and exit after the exit stamp. M: 37 + 43
/// (interrupt timing images, two runs, min == max).
pub const ENTRY_EXIT: u32 = 80;
/// Its entry share, the scan TC to the entry stamp.
pub const ENTRY: u32 = 37;

/// The next scans land in the tick block this long after the scan TC that
/// starts a tick, each at its first slot (`providers::sensors::scan` on the
/// chip asserts the geometry): the trough scan half a period on, the crest
/// scan a period on, each less the crest scan's length.
pub const NEXT_TROUGH: u32 = 832;
pub const NEXT_CREST: u32 = 2032;

/// Past the entry stamp, the latest a tick moves the trough's samples out
/// of the next trough scan's way. E: priced as [`TEL_POLL`], 96, plus 10%.
pub const TROUGH_DONE: u32 = 106;

/// Past the entry stamp, the latest a tick reads the crest scan in the
/// tick block or arms a TEL row's copy of it, before any stage that runs
/// ahead of the fast step: RAIL's rail and pot reads behind a head and a
/// two-chunk row. E: priced as [`TEL_POLL`], 1011, plus 10%.
pub const FRAME_DONE: u32 = 1113;

/// A hold runs Closed this long after its goal moves, then parks. M: the
/// longest Closed episode after a 600-count step, 2899 / 3396 ticks
/// (drive-arm tagged hold windows).
pub const HOLD_APPROACH_TICKS: u32 = 3400;

/// The window lengths gamma(k) is kept for, in kernel ticks.
pub const WINDOW_TICKS: [usize; 4] = [2, 4, 10, 20];

/// gamma(k) on the bench by regime: the most the bodies of k consecutive
/// ticks took, events included. M: the quiet windows (Open) and the
/// stepping windows (Closed), 3 runs, on the window-record image, with the
/// rule's margin. The record updates after the exit stamp, outside every
/// body; that image's quiet-window plain maxima read 4 to 7 ticks over
/// the bench image's, layout.
pub const WINDOW_LOAD: [[u32; WINDOW_TICKS.len()]; 2] =
    [[1781, 3155, 7015, 13428], [3225, 6073, 13729, 27067]];

/// The same with six fields streaming at 3M. M as [`WINDOW_LOAD`]: TEL
/// still (Open) and TEL stepping (Closed).
pub const WINDOW_LOAD_TEL: [[u32; WINDOW_TICKS.len()]; 2] =
    [[2441, 4405, 9888, 18956], [4091, 7362, 16665, 32527]];

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
/// M: r3-refill, each body's net maximum over every window, a 1M window's
/// Deadline less [`spin_extra`], 3 runs: BreakWake 978 (TEL stepping),
/// TxDone 305 (stepping), Deadline 5953 (the TEL chunk WRITE, arm10).
pub const fn body(b: Body) -> u32 {
    match b {
        Body::BreakWake => 1026,
        Body::TxDone => 353,
        Body::Deadline => 6132,
    }
}

/// The same bodies' mean over a window.
///
/// M: r3-refill, each body's largest window mean (at least
/// [`MEAN_MIN_BODIES`] bodies), the same spin rule, 3 runs: BreakWake 756
/// (arm10), Deadline 4496 (the goal WRITE). TxDone 261 (ping 1M) on a
/// build-image.sh image: cargo hashes the path of every path dependency
/// into the crate metadata, so the release layout links the ISR bodies in
/// another function order than a checkout build, and the checkout's 243
/// (stepping) does not transfer.
pub const fn body_mean(b: Body) -> u32 {
    match b {
        Body::BreakWake => 794,
        Body::TxDone => 275,
        Body::Deadline => 4721,
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
    /// A WRITE or COMMIT whose commit reaches the kernel's inputs.
    ConfigWrite,
}

impl Frame {
    pub const ALL: [Frame; 6] = [
        Frame::Ping,
        Frame::Read32,
        Frame::Read240,
        Frame::Write,
        Frame::GreadSlot,
        Frame::ConfigWrite,
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
/// chain bookkeeping and reclaim deadline (15 us). ConfigWrite, E: Write
/// plus [`CONFIG_COUNT`].
pub const fn frame(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5520,
        Frame::Read32 => 6000,
        Frame::Read240 => 6240,
        Frame::Write => 8160,
        Frame::GreadSlot => 6720,
        Frame::ConfigWrite => 8160 + CONFIG_COUNT,
    }
}

/// The configuration count's odd/even pair around a commit that reaches
/// the kernel's inputs, over a goal WRITE's commit. E: 21 on the release
/// disassembly, each side a load, a store and four address and add ops
/// (load/store 3.25, ALU 2 HCLK) less the taken transfer a goal WRITE takes
/// past it (6 against 2), plus 10%.
pub const CONFIG_COUNT: u32 = 23;

/// The same frames' mean over a window. E: Read32 112 us (the v1 118.9 us
/// less the third arm, which two-arm replies dropped); Ping, Read240, Write,
/// GreadSlot and ConfigWrite shifted from it as in [`frame`].
pub const fn frame_mean(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5136,
        Frame::Read32 => 5376,
        Frame::Read240 => 5616,
        Frame::Write => 7152,
        Frame::GreadSlot => 6096,
        Frame::ConfigWrite => 7152 + CONFIG_COUNT,
    }
}

/// The rate the bus budgets are set at (the fleet rate).
pub const BUDGET_BAUD_HZ: u32 = 3_000_000;

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
    let r = regime.event_row() as usize;
    let mut t = tick_body(regime, phase);
    if ev.refresh {
        t += REFRESH;
    }
    if ev.slow {
        t += SLOW_DUE[r];
    }
    if ev.finish {
        t += SLOW_FINISH;
    }
    if ev.tel {
        // The lane leaves a banking tick be (its gate is under the poll),
        // and a stage tick, so the dearest TEL tick is the bank's or a lane
        // step's, a stage tick's the bank's or a head's. A one-row batch's
        // head and bank land on one tick: under a step's.
        let held = ev.slow || ev.finish;
        let bank = if ev.bank { TEL_BANK + TEL_POLL } else { 0 };
        t += TEL_ROW + max(tel_lane(held), bank);
    }
    if ev.health {
        t += HEALTH_CLOSE;
    }
    t
}

/// What a kernel tick carried beyond its phase.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Events {
    pub refresh: bool,
    pub tel: bool,
    pub bank: bool,
    /// The SLOW block's first stage ran; `finish`: its second.
    pub slow: bool,
    pub finish: bool,
    /// The tick-load counters closed a window.
    pub health: bool,
}

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
//! judged with it, and it is never a reason to exceed one. The thermal rows
//! ([`SLOW_FINISH`], [`SLOW_CLOSE`], [`THERM_INSTALL`], [`THERM_SIDE`]) are
//! excepted while the thermometer's stages are being redesigned: their pins
//! report and do not gate fit.

pub mod probe;

#[cfg(test)]
mod tests;

use osc_protocol::wire::LAW_BREAK_BIT_TIMES;

use crate::kernel::{DECIM_MED, TICK_HZ};

/// The clock every budget counts: the chip's HCLK (the chip asserts it).
pub const CLOCK_HZ: u32 = 48_000_000;
pub const TICKS_PER_US: u32 = CLOCK_HZ / 1_000_000;

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
/// Closed over the same windows' Closed ticks. Open phases 0, 5 and 7, M:
/// r3-refill, every gated window's Open ticks, 3 runs, less
/// [`TICK_PUBLISH`]. Phase SLOW off its due period is a plain tick; the due
/// body is [`SLOW_DUE`].
pub const KERNEL: [[u32; PHASES]; 2] = [
    [811, 1070, 890, 932, 641, 954, 641, 803, 599, 593],
    [1403, 1610, 1684, 1469, 1613, 1520, 1181, 1346, 1139, 1127],
];

/// The same bodies' mean over a window: the most it may show.
///
/// M: per phase the largest window mean of the same windows, Closed from
/// the stepping windows. Open phases 0 and 7, M: r3-refill, every gated
/// window's largest Open mean, 3 runs, less [`TICK_PUBLISH`].
pub const KERNEL_MEAN: [[u32; PHASES]; 2] = [
    [781, 1062, 867, 915, 622, 951, 611, 782, 578, 566],
    [1234, 1528, 1495, 1307, 1454, 1367, 994, 1175, 956, 948],
];

/// A window's mean is judged only over this many bodies.
pub const MEAN_MIN_BODIES: u32 = 100;

/// The configuration take after a write, on top of [`KERNEL`], at most once
/// per period, at phase SPARE_A and in place of the thermometer's
/// stage there: the input span's word copy, the Ke belt, the thermometer's
/// config, its bound and the terminal floor's store. It carries no TEL row:
/// a burst starts neither on SPARE_A nor while a take is pending, and
/// the host is silent through one. E: 557, the take and the bound it calls
/// on the release disassembly with the fetch model (load/store 3.25, ALU 2,
/// mul 10, taken transfer 6, call and return 12 HCLK; 84 loads and stores,
/// 353, and the bound's worst path, 204), plus 10%. The drive direction
/// check's `drive_polarity` compare adds 11.25, 15.75 on a change: 573,
/// inside the row.
pub const REFRESH: u32 = 613;

/// The tick block's work on every tick, on top of [`KERNEL`], whose bodies
/// predate it, by regime and phase: the scan landed by the ADC DMA (the
/// trough's copies, the crest's shunt copy and the taps named by terminal,
/// where the bodies copied the frame), the publish (the held current, the
/// differential, the duty and the tick flags) and the TEL gate. E: each
/// phase's plain tick over the same tick of the image the rows were
/// measured on, priced on the release image with the fetch model (as
/// [`TEL_POLL`]), per drive arm (Closed; the dearer of Brake and Off for
/// Open), plus 10%; a phase that got cheaper, PUBLISH with its dropped
/// duplicates, is not credited. The crest's shunt copy, two 4-byte
/// instructions (6), is 7 in every cell. The release image prices under
/// every row.
pub const TICK_PUBLISH: [[u32; PHASES]; 2] = [
    [9, 38, 18, 7, 15, 28, 18, 7, 13, 16],
    [12, 65, 42, 32, 25, 35, 45, 7, 39, 43],
];

/// Code layout on top of every kernel row, maximum and mean: one source
/// compiles to tick bodies that differ by register allocation and block
/// placement from build to build, and between the bench image the rows
/// are measured on and the release image. E: the bench image's excess
/// over the release image's on the same change, priced per phase as
/// [`TICK_PUBLISH`], on mean rows.
pub const LAYOUT: u32 = 35;

/// A plain tick's budget at `regime` and `phase`: its row, the tick
/// block's work and layout.
pub const fn tick_body(regime: Regime, phase: usize) -> u32 {
    let (r, p) = (regime as usize, phase % PHASES);
    KERNEL[r][p] + TICK_PUBLISH[r][p] + LAYOUT
}

/// The same plain ticks' mean over a window.
pub const fn tick_body_mean(regime: Regime, phase: usize) -> u32 {
    let (r, p) = (regime as usize, phase % PHASES);
    KERNEL_MEAN[r][p] + TICK_PUBLISH[r][p] + LAYOUT
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
/// the arm 210, the rest's feed 156, the patch 189, the head 177) and
/// averaged, 112, plus 10%.
pub const TEL_LANE_MEAN: u32 = 124;

/// The SLOW block's first stage on its due tick (phase SLOW, every
/// DECIM_SLOW periods): the permit lease, undervoltage and the
/// thermometer's step on the tick's window, on top of the phase's row, per
/// regime. E: the dearest path, a tracked seat (any drive that holds
/// current), priced at 405 over the plain tick, plus 10%; no window has run
/// it (r3-refill measured at most Open 252, Closed 351).
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

/// The drive direction check's stage on phase SPARE_B, on top of its row:
/// a leg's end against the pot, its verdict or an anchor, while the wiring
/// is unproven, on a SPARE_B no thermometer stage takes. Above both its E
/// and its M: E, the dearest, a leg's end through the position table,
/// priced on the release disassembly with the fetch model as [`REFRESH`]
/// at 193 over the plain tick (the verdict that latches 181, an anchor
/// 120), plus 10%, 212; M 219, the bench's two runs. It stays inside the
/// thermometer's stage there, so the SPARE_B envelope is the same. The
/// plain check rides the SPARE_B row: 9.25 proven, at most 40.5 unproven.
pub const DRIVE_STAGE: u32 = 231;

const _: () = assert!(DRIVE_STAGE <= THERM_SIDE);

/// The tick-load counters' close every `kernel::HEALTH_CLOSE_TICKS` ticks, on top
/// of the tick it lands on, and the catch-up it adds after a lost tick.
/// HEALTH_CLOSE, M: r3-refill, the closing ticks' maximum over the plain
/// maximum (in the TEL windows over the TEL ticks'), either regime, every
/// window with the report-only one, 3 runs: 228 (Closed, hold raw pos;
/// 159 in the TEL windows). The probe books a close ahead of a bank, so
/// that cell may hold a bank too. CATCH_UP, E: priced on the executed path.
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
pub const NEXT_TROUGH: u32 = 888;
pub const NEXT_CREST: u32 = 2088;

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
/// ticks took. M: the quiet windows (Open) and the stepping windows
/// (Closed). Closed and Open gamma(2) and gamma(4), M: r3-refill, the same
/// windows, 3 runs.
pub const WINDOW_LOAD: [[u32; WINDOW_TICKS.len()]; 2] =
    [[1861, 3461, 8216, 15705], [3446, 6459, 14369, 27820]];

/// The same with six fields streaming at 3M. M: r3-refill, TEL still
/// (Open) and TEL stepping (Closed), 3 runs.
pub const WINDOW_LOAD_TEL: [[u32; WINDOW_TICKS.len()]; 2] =
    [[2806, 4886, 11628, 22137], [4249, 7831, 17150, 33348]];

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
/// (arm10), TxDone 243 (stepping, the goal WRITE), Deadline 4496 (the goal
/// WRITE).
pub const fn body_mean(b: Body) -> u32 {
    match b {
        Body::BreakWake => 794,
        Body::TxDone => 256,
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
    let r = regime as usize;
    let mut t = tick_body(regime, phase);
    if ev.refresh {
        t += REFRESH;
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
    if ev.drive {
        t += DRIVE_STAGE;
    }
    if ev.tel {
        // The lane leaves a banking tick be (its gate is under the poll),
        // and a stage tick, so the dearest TEL tick is the bank's or a lane
        // step's, a stage tick's the bank's or a head's. A one-row batch's
        // head and bank land on one tick: under a step's.
        let held = ev.slow || ev.finish || ev.close || ev.install || ev.side;
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
    /// The SLOW block's first stage ran; `finish`: its second, and
    /// `close`: that froze a thermal reference window.
    pub slow: bool,
    pub finish: bool,
    pub close: bool,
    /// The thermometer's stages on phases SPARE_A and SPARE_B.
    pub install: bool,
    pub side: bool,
    /// The drive direction check's stage on phase SPARE_B.
    pub drive: bool,
    /// The tick-load counters closed a window.
    pub health: bool,
}

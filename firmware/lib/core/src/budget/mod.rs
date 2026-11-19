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
/// M, every gated window's ticks by drive arm, the TEL windows' plain
/// ticks among them, a closing tick booked apart, one run of the budgets
/// hardware test on a `scripts/build-image.sh` bench image of this stack,
/// which carries the window-current work, so each cell upper-bounds the
/// kernels below it: Open over the Open ticks, Closed over the Closed
/// ticks, OpenLoop over the openloop window's OpenLoop ticks. A
/// report-only window feeds no row. Each cell is its measured maximum
/// with the rule's margin; the bodies hold the tick block's work, the
/// entry on the injected group, the window term off the reading, the motor
/// write ahead of the voltage side, the reversal's no-reading test, the
/// wait for the crest scan and every phase's stages, RAIL's window term on
/// the rail among them. The Closed and OpenLoop rows carry
/// [`WINDOW_CARRY`] on top, every row [`DEADMAN`], phase TRAJECTORY
/// [`PARK_GATE`] and phases TRAJECTORY and VELOCITY [`HELD`], which the
/// measurement predates.
/// Phase SLOW off its due period is a plain tick; the due body is
/// [`SLOW_DUE`].
pub const KERNEL: [[u32; PHASES]; REGIMES] = with_e_cells([
    [942, 1176, 1053, 1374, 834, 1239, 810, 969, 795, 855],
    [1401, 1719, 1694, 1685, 1608, 1694, 1242, 1434, 1224, 1500],
    [1365, 1638, 1218, 1605, 1218, 1605, 1200, 1380, 1185, 1302],
]);

/// The same bodies' mean over a window: the most it may show.
///
/// M: per phase the largest window mean over the same windows and ticks,
/// judged over [`MEAN_MIN_BODIES`], with the rule's margin, at most the
/// maximum's row (Open phase 5, OpenLoop phases 0, 3, 5 and 7: the bodies
/// barely vary); with [`WINDOW_CARRY`], [`DEADMAN`], [`PARK_GATE`] and
/// [`HELD`], as the maximum's.
pub const KERNEL_MEAN: [[u32; PHASES]; REGIMES] = with_e_cells([
    [931, 1171, 1039, 1224, 821, 1239, 790, 957, 771, 834],
    [1364, 1619, 1593, 1628, 1556, 1650, 1197, 1348, 1179, 1437],
    [1365, 1579, 1212, 1605, 1212, 1605, 1196, 1380, 1178, 1300],
]);

/// E, on every Closed and OpenLoop cell, maximum and mean, and on
/// [`WRITE_DONE`] and [`FRAME_DONE`]: the window term's carry
/// (`WindowTerm::take`) on the reading ahead of the write. An Open tick
/// measures a window its own arm left undriven and takes no reading. The
/// take loads the carry, adds it and stores the new one, a halfword each:
/// 8.5 HCLK priced on the executed path of the release image with the
/// fetch model ([`REFRESH`]), no credit for the rounding RAIL no longer
/// adds; plus 10%, rounded up. The bench must refill the rows.
pub const WINDOW_CARRY: u32 = 10;

/// E, on every cell by phase, maximum and mean: the bus dead-man (kernel
/// sec 2.7), against the image without it. Priced on the executed path of
/// the release image with the fetch model ([`REFRESH`]): the streaming
/// flag the step takes, a load and a load, 6.5 HCLK on every phase but
/// TRAJECTORY's, whose path is unchanged; CONTROL on top the generation's
/// read, 8.5, the countdown's period inline, 56.75 on its dearest path (a
/// lapse), and its start on the enable edge, 11.75, with no credit for the
/// branch the period shares with the edge's test. Plus 10%, rounded up.
/// The bench must refill the rows.
pub const DEADMAN: [u32; PHASES] = [92, 8, 0, 8, 8, 8, 8, 8, 8, 8];

/// E, on phase TRAJECTORY of every regime, maximum and mean: the hold's
/// load gate (`position::unloaded`), against the image without it. Priced
/// on the executed path of the release image with the fetch model
/// ([`REFRESH`]): within twice the band of a stopped goal, the disturbance
/// and the friction level loaded, a word and a halfword, and eight ALU
/// (three for the magnitude, the shift, the compare, its sense, the join,
/// and one the allocation moved into the dwell's compare), 22.5 HCLK; off
/// that path the same instructions as before. Every regime, since a Position
/// hold can run on an Open tick (parked) and on an OpenLoop one (the first
/// TRAJECTORY after a mode change from OpenLoop). Plus 10%, rounded up.
/// The bench must refill the rows.
pub const PARK_GATE: u32 = 25;

/// E, on phases TRAJECTORY and VELOCITY of every regime, maximum and mean:
/// the loaded hold (`Life::Held`, the velocity integral frozen), against
/// the image with the load gate. Priced on the executed path of the
/// release image with the fetch model ([`REFRESH`]), from the phase's entry
/// to its return. TRAJECTORY, dearest at rest where the hold is taken: the
/// hold's kind out of the two flags, two ALU (403.25 HCLK against 401.25
/// held or driven in the zone, 364.75 against 362.75 parked); past the
/// zone 401.25 as before, and a moving profile skips the band's multiply
/// for the same 576. VELOCITY, on a driven period: the freeze's test, two
/// ALU, and the integral's saturation one taken transfer where the image
/// without it falls through, less a taken transfer in its clamp, 6 HCLK; a
/// held period skips the charge. Every regime, as [`PARK_GATE`]. Plus 10%,
/// rounded up. The bench must refill the rows.
pub const HELD: [u32; PHASES] = held_cells();

const fn held_cells() -> [u32; PHASES] {
    let mut cells = [0; PHASES];
    cells[phase::TRAJECTORY as usize] = 3;
    cells[phase::VELOCITY as usize] = 7;
    cells
}

const fn with_e_cells(mut rows: [[u32; PHASES]; REGIMES]) -> [[u32; PHASES]; REGIMES] {
    let mut p = 0;
    while p < PHASES {
        rows[Regime::Closed as usize][p] += WINDOW_CARRY;
        rows[Regime::OpenLoop as usize][p] += WINDOW_CARRY;
        let mut r = 0;
        while r < REGIMES {
            rows[r][p] += DEADMAN[p];
            r += 1;
        }
        p += 1;
    }
    let mut r = 0;
    while r < REGIMES {
        rows[r][phase::TRAJECTORY as usize] += PARK_GATE;
        let mut p = 0;
        while p < PHASES {
            rows[r][p] += HELD[p];
            p += 1;
        }
        r += 1;
    }
    rows
}

/// A window's mean is judged only over this many bodies.
pub const MEAN_MIN_BODIES: u32 = 100;

/// The configuration take after a write, on top of [`KERNEL`], at most once
/// per period, at phase SPARE_A: the input span's word copy, the belt and
/// the `UNSET` bit, the two speeds' and the cap's resolution, the current
/// loop's resistance, the brush term's size and fade, the winding
/// estimate's config, the continuous rating and its leak, the winding
/// resistance on the new `r_q12`
/// (`Medium::r_hat`) and the terminal floor's store. It carries no TEL
/// row: a burst starts neither on SPARE_A nor while a take is pending, and
/// the host is silent through one. E, an upper bound: the take's longest
/// path on matched release disassembly is under 781 HCLK, including the
/// rating, its two log calls, the resistance's 115.25, the speeds' 34 (22
/// more instructions, two halfword stores where the span copy stored their
/// word) and the unset limits' 56.25 (the belt over both limits, the
/// `UNSET` byte and the cap's resolution: 22 more instructions on the
/// longest path, thirteen loads and stores, most of them spills, and ten
/// ALU, one taken transfer fewer). The window term's fence and
/// conductance add at most 54.75 over it (the take's net on matched
/// release disassembly: seven loads and stores, two multiplies, six ALU,
/// no credit for three fewer transfers), so the path is under 836, the row.
/// The fetch model prices loads/stores at 3.25, ALU at 2, multiplies at
/// 10, taken transfers at 6, and call plus return at 12 HCLK.
pub const REFRESH: u32 = 836;

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
/// TEL, on the image and run of [`KERNEL`], with the rule's margin: Open
/// 876 over 762, the 1M write window's; Closed 1305 over 1194, the
/// stepping window's.
pub const SLOW_DUE: [u32; EVENT_REGIMES] = [162, 159];

/// The second stage on the next period's SLOW phase: the NTC's check and
/// quadratic, the two rise multiplies, the cutoff fault and the winding
/// resistance read at the estimate's temperature (`Medium::r_hat`). M as
/// [`SLOW_DUE`], over the Closed row the stage is budgeted on: 1584, the
/// stepping window's, over 1194.
pub const SLOW_FINISH: u32 = 438;

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
/// (interrupt timing images, two runs, min == max), on the scan DMA's
/// vector; the tick's ADC vector is the same trampoline.
pub const ENTRY_EXIT: u32 = 80;
/// The injected group's end (JEOC), which starts a tick, to the entry
/// stamp. E: the stamps of a streaming-entry bench image, 93 to 110 HCLK
/// past the crest over about 930,000 driven ticks, less JEOC's 56 past it
/// by the reference manual's arithmetic (`providers::sensors::scan`); the
/// least, 37, is the trampoline's measured share above.
pub const ENTRY: u32 = 54;

/// Past the injected group's end, each at the HCLK the chip's geometry
/// asserts (`providers::sensors::scan`): the trough's update event, which
/// loads a motor write that lands before it; the crest scan's end, which
/// the tick waits for after its write; and the next trough and crest
/// scans' first slot in the tick block.
pub const WRITE_DEADLINE: u32 = 1144;
pub const CREST_SCAN_DONE: u32 = 424;
pub const NEXT_TROUGH: u32 = 1200;
pub const NEXT_CREST: u32 = 2456;

/// Past the entry stamp, the latest the tick's motor write lands: the shunt
/// from the injected group, the window take, the overcurrent trip, the
/// drive arm and the compare stores. E: priced on the executed path of the
/// release image with the fetch model ([`REFRESH`]): the Closed arm's
/// dearest, a wide code, the settle band's lookup, an overcurrent count
/// and every clamp of the current loop, 768.75 HCLK (709 with none);
/// OpenLoop's, the duty limiter's call included, 704.5; and the window
/// term ahead of the sign, two loads, a multiply, three ALU and the moved
/// compare, 24.5. Plus 10%, rounded up; and [`WINDOW_CARRY`].
pub const WRITE_DONE: u32 = 873 + WINDOW_CARRY;

/// Past the entry stamp, the latest a tick moves the trough's samples out
/// of the next trough scan's way, at the tick's entry. E: priced as
/// [`TEL_POLL`], 96, plus 10%.
pub const TROUGH_DONE: u32 = 106;

/// Past the entry stamp, the latest a tick reads the crest scan in the
/// tick block or arms a TEL row's copy of it, before any stage that runs
/// ahead of the fast step: RAIL's rail and pot reads behind a head and a
/// two-chunk row, the wait for the scan ([`SCAN_WAIT`]) apart. E: priced
/// as [`TEL_POLL`], 1011, the window term ahead of the write, 24.5 as in
/// [`WRITE_DONE`] (RAIL puts it on the rail after its reads), and the
/// streaming flag the step takes ahead of the reads, 6.5 as in
/// [`DEADMAN`], plus 10%; and [`WINDOW_CARRY`].
pub const FRAME_DONE: u32 = 1147 + WINDOW_CARRY;

/// E, on [`FRAME_DONE`] in the scan deadline, which the measured
/// [`KERNEL`] rows do not reach: the tick entered on the injected group's
/// end, where the priced reads entered on the scan's DMA. The entry clears
/// JEOC and takes the shunt from the group; after the write it reads the
/// timer's direction and keeps it, polls and clears the scan's TC, lands
/// the crest taps and checks for a tap swap; the window take and the tail
/// each test the late write. Priced on the executed path of the release
/// image with the fetch model ([`REFRESH`]): 141.25 HCLK against the old
/// entry's 99.75, plus 10%, rounded up.
pub const STREAM_ENTRY: u32 = 46;

/// E, on [`FRAME_DONE`] in the scan deadline by regime ([`Regime`] order):
/// the wait for the crest scan after the write. At most
/// [`CREST_SCAN_DONE`] less the least [`ENTRY`], 37, less the shortest
/// path to the poll, priced as [`STREAM_ENTRY`] less 10%: a faulted Off
/// tick's 257.5 HCLK, a Closed tick's endstop brake's 297.75; OpenLoop's
/// shortest path passes the scan's end. Rounded up.
pub const SCAN_WAIT: [u32; REGIMES] = [156, 120, 0];

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
/// TxDone 305 (stepping), Deadline 5953 (the TEL chunk WRITE, arm10); the
/// Deadline with [`BUS_TOUCH`] and [`COMMIT_TOUCH`], which the measurement
/// predates.
pub const fn body(b: Body) -> u32 {
    match b {
        Body::BreakWake => 1026,
        Body::TxDone => 353,
        Body::Deadline => 6132 + BUS_TOUCH + COMMIT_TOUCH,
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
/// (stepping) does not transfer. The Deadline with the touches, as the
/// maximum's.
pub const fn body_mean(b: Body) -> u32 {
    match b {
        Body::BreakWake => 794,
        Body::TxDone => 275,
        Body::Deadline => 4721 + BUS_TOUCH + COMMIT_TOUCH,
    }
}

/// E, on every frame dispatched to this servo and on the Deadline body that
/// dispatches it, maximum and mean: the bus generation the dead-man reloads
/// on (`Shared::bus_touch`). Priced on the executed path of the release
/// image with the fetch model ([`REFRESH`]): the dispatch's arms join at
/// one exit for it, a taken transfer and the base address's reload, 10,
/// and the touch, 10.5; COMMIT_TOUCH, a commit's own touch, 10.5 on its
/// longest path. Plus 10%, rounded up. The bench must refill the rows.
pub const BUS_TOUCH: u32 = 23;
pub const COMMIT_TOUCH: u32 = 12;

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
/// plus [`CONFIG_COUNT`]. Each with [`BUS_TOUCH`], a Write's and a
/// ConfigWrite's with [`COMMIT_TOUCH`] too.
pub const fn frame(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5520 + BUS_TOUCH,
        Frame::Read32 => 6000 + BUS_TOUCH,
        Frame::Read240 => 6240 + BUS_TOUCH,
        Frame::Write => 8160 + BUS_TOUCH + COMMIT_TOUCH,
        Frame::GreadSlot => 6720 + BUS_TOUCH,
        Frame::ConfigWrite => 8160 + CONFIG_COUNT + BUS_TOUCH + COMMIT_TOUCH,
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
/// GreadSlot and ConfigWrite shifted from it as in [`frame`], with the
/// touches.
pub const fn frame_mean(f: Frame) -> u32 {
    match f {
        Frame::Ping => 5136 + BUS_TOUCH,
        Frame::Read32 => 5376 + BUS_TOUCH,
        Frame::Read240 => 5616 + BUS_TOUCH,
        Frame::Write => 7152 + BUS_TOUCH + COMMIT_TOUCH,
        Frame::GreadSlot => 6096 + BUS_TOUCH,
        Frame::ConfigWrite => 7152 + CONFIG_COUNT + BUS_TOUCH + COMMIT_TOUCH,
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

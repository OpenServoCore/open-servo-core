//! Timing budgets: the most CPU, in HCLK ticks, that each kernel phase, bus
//! vector body and frame may take on the chip. One table for three users:
//! the bench probe records the same components (`probe`), the hardware test
//! `budgets` fails a window whose maximum or mean exceeds its budget, and
//! the DES charges these values as its costs. While that test is green, the
//! DES runs no component cheaper than silicon.
//!
//! Each value is M (measured by the budget probe) or E (estimated, from the
//! named instrument plus the stated headroom). Kernel values are whole tick
//! bodies, entry stamp to exit stamp, by motion regime and medium phase; bus
//! values exclude the kernel time that preempted them. Interrupt entry and
//! exit fall outside every stamp: [`ENTRY_EXIT`] carries them for the DES.

pub mod probe;

#[cfg(test)]
mod tests;

use crate::kernel::DECIM_MED;

/// The clock every budget counts: the chip's HCLK (the chip asserts it).
pub const CLOCK_HZ: u32 = 48_000_000;
pub const TICKS_PER_US: u32 = CLOCK_HZ / 1_000_000;

/// Medium phases per period; index = `kernel::phase` (8 and 9 run the fast
/// path alone).
pub const PHASES: usize = DECIM_MED as usize;

/// What the motor does while the kernel runs: the closed current loop and the
/// trajectory are the regime-dependent work.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Regime {
    /// Torque off.
    Quiet,
    /// Position mode holding a goal.
    Hold,
    /// Position mode stepping or tracking.
    Moving,
}

impl Regime {
    pub const ALL: [Regime; 3] = [Regime::Quiet, Regime::Hold, Regime::Moving];
}

/// Kernel tick body with the TEL burst idle and no configuration rebuild,
/// per regime and phase: the longest a window may show.
///
/// Quiet and Hold, E: [`KERNEL_MEAN`] plus 3 us; SLOW: the v1 SLOW bin max
/// 27.3 us over 1265 SLOW runs plus 1 us. Moving, E: [`KERNEL_MEAN`] plus
/// 4 us; OBSERVER and SLOW, the two phases of the longest bodies on record,
/// at the moving maximum 48.5 us (kernel-above-bus probe, 2.59M entries,
/// chainload moving).
pub const KERNEL: [[u32; PHASES]; 3] = [
    [1125, 1394, 876, 1231, 912, 1177, 1358, 1087, 846, 856],
    [1167, 1394, 1383, 1240, 936, 1216, 1358, 1087, 846, 856],
    [2028, 2328, 2260, 2094, 2116, 2084, 2328, 1940, 1697, 1706],
];

/// The same bodies' mean over a window: the most it may show.
///
/// Quiet and Hold, E: cpu-probe v2 LOW-exclusive means (three 20 s windows,
/// kernel below the bus). Moving, E: v2 means under the chainload sine plus
/// 4 us, the step soak's heavier kernel (its fit: 16.7-18.3% of rows dropped
/// and 4.9-5.9% of ticks over against the bench's 19.6% and 5.3-5.5%).
pub const KERNEL_MEAN: [[u32; PHASES]; 3] = [
    [981, 1250, 732, 1087, 768, 1033, 763, 943, 702, 712],
    [1023, 1250, 1239, 1096, 792, 1072, 763, 943, 702, 712],
    [1836, 2081, 2068, 1902, 1924, 1892, 1567, 1748, 1505, 1514],
];

/// A window's mean is judged only over this many bodies.
pub const MEAN_MIN_BODIES: u32 = 100;

/// The CONTROL tick that rebuilds the kernel's configuration snapshot, on
/// top of [`KERNEL`]. E: the boot tick's CONTROL max 37.5 us against the
/// 19.4 us CONTROL mean (v1 bins), plus 2 us.
pub const REFRESH: u32 = 960;

/// Every tick of a TEL burst (sample build and encode), on top of [`KERNEL`],
/// per regime. Quiet and Hold, E: v2 TEL-active non-banking ticks 30.6 us
/// less the 1.9 us staging poll and the 17.9 us quiet mean, plus 1.2 us.
/// Moving, E: about 190 instructions; the closed loop already holds most of
/// the sample.
pub const TEL_ENCODE: [u32; 3] = [576, 576, 240];

/// Its mean over a window: the TEL ticks' mean less the plain ticks'. E:
/// Quiet and Hold the derivation above without headroom; Moving 4 us.
pub const TEL_ENCODE_MEAN: [u32; 3] = [518, 518, 192];

/// The tick that banks a TEL batch, on top of [`TEL_ENCODE`]. E: v2 banking
/// ticks 32.7 us against 30.6 us non-banking, plus 2 us.
pub const TEL_BANK: u32 = 192;

/// Interrupt entry and exit around a stamped body: hardware stacking, vector
/// fetch, trampoline. E: 3-6 us of unattributed time per frame of about five
/// bus entries (v1 probe, polling ladder against quiet).
pub const ENTRY_EXIT: u32 = 48;

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
}

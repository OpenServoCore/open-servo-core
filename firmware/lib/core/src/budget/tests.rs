use super::probe::{
    BusProbe, Component, HALT_TICKS, KernelProbe, LONG_TICKS, Load, NARROW_N_MAX, NarrowLoad,
    Scenario, TEL_PER_TICK_NOISE, Tally, TelCost, TelReport, TelStep, TelTally, WindowProbe,
    Windows, rows,
};
use super::*;
use crate::kernel::{HEALTH_CLOSE_TICKS, ProbeTags, health_close_at, health_close_lands, phase};

const PLAIN: Events = Events {
    refresh: false,
    tel: false,
    bank: false,
    slow: false,
    finish: false,
    close: false,
    install: false,
    side: false,
    drive: false,
    health: false,
};
const OPEN: Regime = Regime::Open;

/// A kernel tick on the grid: tick `n` entered `n` periods in; its regime
/// when it booked.
fn tick(
    k: &mut KernelProbe,
    n: u32,
    body: u32,
    phase: usize,
    r: Regime,
    ev: Events,
) -> Option<Regime> {
    let entry = n * TICK_PERIOD;
    let booked = k.book(body, phase, r, ev, n == 0);
    k.ended(entry, entry + body);
    booked
}

fn words<T>(t: &T) -> &[u32] {
    // SAFETY: the probe records are repr(C) runs of u32.
    unsafe {
        core::slice::from_raw_parts(t as *const T as *const u32, core::mem::size_of::<T>() / 4)
    }
}

/// [`tick`] with the window record fed beside it.
fn both(
    k: &mut KernelProbe,
    w: &mut WindowProbe,
    n: u32,
    body: u32,
    phase: usize,
    r: Regime,
    ev: Events,
) {
    let booked = tick(k, n, body, phase, r, ev);
    w.tick(n * TICK_PERIOD, body, booked);
    w.ended(n * TICK_PERIOD + body);
}

#[test]
fn kernel_ticks_book_by_what_they_carried() {
    let mut k = KernelProbe::ZERO;
    let mut w = WindowProbe::ZERO;
    let closed = Regime::Closed;
    both(&mut k, &mut w, 0, 1000, 0, OPEN, PLAIN);
    both(&mut k, &mut w, 1, 1100, 1, OPEN, PLAIN);
    both(
        &mut k,
        &mut w,
        2,
        1500,
        2,
        OPEN,
        Events { tel: true, ..PLAIN },
    );
    let bank = Events {
        tel: true,
        bank: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 3, 1700, 3, OPEN, bank);
    let refresh = Events {
        refresh: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 4, 2000, 8, OPEN, refresh);
    both(&mut k, &mut w, 5, HALT_TICKS + 1, 4, OPEN, PLAIN);
    both(&mut k, &mut w, 6, LONG_TICKS, 1, closed, PLAIN);
    both(
        &mut k,
        &mut w,
        7,
        1900,
        6,
        closed,
        Events {
            slow: true,
            ..PLAIN
        },
    );
    let close = Events {
        finish: true,
        close: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 8, 2200, 6, closed, close);
    let install = Events {
        install: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 9, 1600, 8, closed, install);
    let side = Events {
        side: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 10, 1700, 9, closed, side);
    assert_eq!(k.load(OPEN, 0), Load::ZERO, "the boot tick books nothing");
    let one = |b| Load {
        n: 1,
        sum: b,
        max: b,
    };
    assert_eq!(
        (k.load(OPEN, 1), k.load(closed, 1)),
        (one(1100), one(LONG_TICKS))
    );
    assert_eq!((k.tel_max[0][2], k.bank_max[0][3]), (1500, 1700));
    assert_eq!(
        (k.refresh, k.slow[1], k.close),
        (one(2000), one(1900), one(2200))
    );
    assert_eq!((k.install, k.side), (one(1600), one(1700)));
    assert_eq!(
        k.load(closed, 8),
        Load::ZERO,
        "the stages stay off the plain rows"
    );
    assert_eq!((k.halts, w.long), (1, [3, 5]));
    both(&mut k, &mut w, 11, 1800, 8, OPEN, refresh);
    assert_eq!(
        k.refresh,
        Load {
            n: 2,
            sum: 3800,
            max: 2000
        }
    );
    let finish = Events {
        finish: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 12, 1750, 6, closed, finish);
    assert_eq!(k.finish, one(1750));
    let bodies = 1000
        + 1100
        + 1500
        + 1700
        + 2000
        + HALT_TICKS
        + 1
        + LONG_TICKS
        + 1900
        + 2200
        + 1600
        + 1700
        + 1800
        + 1750;
    assert_eq!((k.busy, k.entries), (bodies + 13 * ENTRY_EXIT, 13));
    assert_eq!(k.drove(), Regime::Closed);
}

/// A tick books the arm it ran or the one before it, Closed over OpenLoop
/// over Open, carried in its tags or read around it; a window that drove
/// either way holds its event ticks to the Closed row.
#[test]
fn a_tick_books_closed_over_openloop_over_open() {
    use Regime::{Closed, Open, OpenLoop};
    let tags = |regime| ProbeTags {
        booted: true,
        regime,
        ..ProbeTags::default()
    };
    let mut k = KernelProbe::ZERO;
    let booked = |k: &mut KernelProbe, r| k.tick(1000, || tags(r), false, 0, false);
    assert_eq!(booked(&mut k, Open), None, "the boot tick");
    let arms = [OpenLoop, OpenLoop, Closed, OpenLoop, Open, Open];
    let got = arms.map(|r| booked(&mut k, r));
    let want = [OpenLoop, OpenLoop, Closed, Closed, OpenLoop, Open].map(Some);
    assert_eq!(got, want);
    assert_eq!(k.drove(), Closed);
    let mut k = KernelProbe::ZERO;
    for r in [Open, OpenLoop, OpenLoop, Open] {
        booked(&mut k, r);
    }
    assert_eq!(k.drove(), Closed);
    for (a, b) in Regime::ALL
        .into_iter()
        .flat_map(|a| Regime::ALL.map(|b| (a, b)))
    {
        assert_eq!(a.booked(b), b.booked(a), "{a:?} {b:?}");
    }
}

/// An OpenLoop phase's count stops at [`NARROW_N_MAX`] and its total with
/// it, so its mean stays exact over the bodies counted; its longest keeps
/// moving, and the report marks the row saturated.
#[test]
fn an_openloop_count_saturates_with_its_sum() {
    let p = 3;
    let mut l = NarrowLoad::ZERO;
    for _ in 0..NARROW_N_MAX {
        l.add(p, 1000);
    }
    let full = l.load(p);
    assert_eq!(
        (full.n, full.sum, full.max),
        (NARROW_N_MAX, NARROW_N_MAX * 1000, 1000)
    );
    l.add(p, 1200);
    let after = l.load(p);
    assert_eq!((after.n, after.sum, after.max), (full.n, full.sum, 1200));
    assert_eq!(after.mean(), 1000);
    let mut k = KernelProbe::ZERO;
    k.openloop = l;
    let s = Scenario {
        tel: false,
        frames: &[],
    };
    let mut saturated = heapless::Vec::<Component, 4>::new();
    rows(&k, &BusProbe::ZERO, &s, |r| {
        if r.saturated() {
            let _ = saturated.push(r.component);
        }
    });
    assert_eq!(saturated, [Component::Tick(Regime::OpenLoop, p)]);
}

/// An OpenLoop tick books its plain body on its own row and anything it
/// carries on Closed's: a TEL row, a bank, a close, a SLOW stage, and the
/// window record's long ticks.
#[test]
fn openloop_events_fold_into_the_closed_rows() {
    let ol = Regime::OpenLoop;
    let (closed, p) = (Regime::Closed as usize, 2);
    let mut k = KernelProbe::ZERO;
    let mut w = WindowProbe::ZERO;
    let tel = Events { tel: true, ..PLAIN };
    both(&mut k, &mut w, 0, 1000, p, ol, PLAIN);
    both(&mut k, &mut w, 1, 1100, p, ol, PLAIN);
    both(&mut k, &mut w, 2, 1200, p, ol, tel);
    both(&mut k, &mut w, 3, 1300, p, ol, Events { bank: true, ..tel });
    let health = Events {
        health: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 4, 1400, p, ol, health);
    let slow = Events {
        slow: true,
        ..PLAIN
    };
    both(&mut k, &mut w, 5, LONG_TICKS, 6, ol, slow);
    assert_eq!(k.load(ol, p).max, 1100);
    assert_eq!(k.load(Regime::Closed, p), Load::ZERO);
    let at = |m: [[u16; PHASES]; 2]| m[closed][p];
    assert_eq!(
        (at(k.tel_max), at(k.bank_max), at(k.health_max)),
        (1200, 1300, 1400)
    );
    assert_eq!(
        (k.tel[closed].n, k.slow[closed].n, w.long[closed]),
        (1, 1, 1)
    );
    assert_eq!(
        kernel_tick(ol, 6, slow) - tick_body(ol, 6),
        SLOW_DUE[closed]
    );
}

/// The drive direction check's stage books apart from the plain SPARE_B
/// row: a SPARE_B tick that left a stage state. A crossing that enters
/// one, an edge on another phase and a SPARE_B a thermometer stage took
/// book plain.
#[test]
fn a_drive_stage_books_apart_from_its_tags() {
    use crate::kernel::phases::loops::faults::stage_due;
    let (track, leg_end, judge) = (1, 2, 3);
    assert!(!stage_due(track) && stage_due(leg_end) && stage_due(judge));
    let tags = |phase, drive| ProbeTags {
        phase,
        booted: true,
        drive,
        ..ProbeTags::default()
    };
    let ran = |b, a| Events::between(b, a).drive;
    assert!(ran(tags(phase::SPARE_B, leg_end), tags(0, judge)));
    assert!(ran(tags(phase::SPARE_B, judge), tags(0, track)));
    assert!(!ran(tags(phase::SPARE_B, track), tags(0, leg_end)));
    assert!(!ran(tags(phase::SPARE_B, judge), tags(0, judge)));
    assert!(!ran(
        tags(phase::CONTROL, judge),
        tags(phase::CONTROL + 1, 4)
    ));
    let mut k = KernelProbe::ZERO;
    let mut w = WindowProbe::ZERO;
    both(&mut k, &mut w, 0, 1000, 0, OPEN, PLAIN);
    let drive = Events {
        drive: true,
        ..PLAIN
    };
    both(
        &mut k,
        &mut w,
        1,
        1300,
        phase::SPARE_B as usize,
        OPEN,
        drive,
    );
    assert_eq!((k.drive.n, k.drive.max), (1, 1300));
    assert_eq!(k.load(OPEN, phase::SPARE_B as usize), Load::ZERO);
}

/// The drive direction check's stage stays inside the thermometer's stage
/// on SPARE_B, alone or under a TEL row and its bank: the phase's worst
/// tick is the thermometer's, as before the check.
#[test]
fn the_drive_stage_keeps_the_spare_b_envelope() {
    let tel = Events { tel: true, ..PLAIN };
    for r in Regime::ALL {
        for c in [PLAIN, tel, Events { bank: true, ..tel }] {
            let drive = kernel_tick(r, phase::SPARE_B as usize, Events { drive: true, ..c });
            let side = kernel_tick(r, phase::SPARE_B as usize, Events { side: true, ..c });
            assert!(drive <= side, "{r:?} {c:?}: {drive} over {side}");
        }
    }
}

/// Lateness counts only what held the tick off past both its grid slot
/// and its predecessor's end.
#[test]
fn entry_lateness_is_measured_from_the_grid_and_the_last_end() {
    let mut w = WindowProbe::ZERO;
    w.tick(0, 900, None);
    w.ended(900);
    w.tick(TICK_PERIOD + 70, 900, Some(OPEN));
    w.ended(TICK_PERIOD + 970);
    assert_eq!(w.late_max, 70);
    let over = 3 * TICK_PERIOD;
    w.tick(2 * TICK_PERIOD + 70, over, Some(OPEN));
    let end = 5 * TICK_PERIOD + 70;
    w.ended(end);
    w.tick(end + ENTRY_EXIT + 10, 900, Some(OPEN));
    assert_eq!(w.late_max, 70, "a tick the last one overran is not late");
}

#[test]
fn bus_frames_fold_at_breaks() {
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(100, 100, 0, Body::BreakWake);
    b.body(200, 200, 0, Body::Deadline);
    b.body(50, 50, 0, Body::TxDone);
    b.body(40, 40, 0, Body::TxDone);
    b.mark_break();
    b.body(90, 90, 0, Body::BreakWake);
    assert_eq!(
        b.host,
        Load {
            n: 1,
            sum: 390,
            max: 390
        },
        "the reply stays in its frame"
    );
    assert_eq!(b.body[Body::TxDone as usize].n, 2);
}

/// A bus body subtracts every kernel vector that ran inside it, each with
/// its probe update and its interrupt entry and exit; the longest net body
/// keeps its wall time and nesting.
#[test]
fn a_bus_body_nets_out_each_kernel_vector_with_its_entry_and_exit() {
    let mut k = KernelProbe::ZERO;
    tick(&mut k, 0, 900, 0, OPEN, PLAIN);
    let (at, kernel, entries) = (TICK_PERIOD - 300, k.busy, k.entries);
    k.book(1500, 1, OPEN, PLAIN, false);
    k.ended(TICK_PERIOD, TICK_PERIOD + 1530);
    k.other(200);
    let wall = 2 * TICK_PERIOD - at;
    let net = wall - k.busy.wrapping_sub(kernel);
    assert_eq!(net, wall - (1500 + 30 + 200 + 2 * ENTRY_EXIT));
    let nested = k.entries.wrapping_sub(entries);
    assert_eq!(nested, 2);
    let mut b = BusProbe::ZERO;
    b.body(net, wall, nested, Body::TxDone);
    b.body(10, wall + 1000, 5, Body::TxDone);
    let i = Body::TxDone as usize;
    assert_eq!(
        (b.body[i].max, b.wall[i], b.nested[i]),
        (net, wall, nested),
        "the wall and nesting of the net maximum"
    );
}

#[test]
fn a_bus_row_is_judged_on_its_net_time() {
    let mut b = BusProbe::ZERO;
    let at = body(Body::Deadline);
    b.body(at, 2 * at, 3, Body::Deadline);
    let s = Scenario {
        tel: false,
        frames: &[],
    };
    let mut row = None;
    rows(&KernelProbe::ZERO, &b, &s, |r| {
        if r.component == Component::Body(Body::Deadline) {
            row = Some(r);
        }
    });
    let r = row.expect("a Deadline row");
    assert_eq!((r.max, r.wall, r.nested), (at, 2 * at, 3));
    assert!(!r.over());
}

/// One polled exchange as the chip books it: the wake, the deadline body
/// that triggers the reply, the reply's two arms.
fn exchange(b: &mut BusProbe) {
    b.mark_break();
    b.body(100, 100, 0, Body::BreakWake);
    b.mark_dispatch();
    b.body(200, 200, 0, Body::Deadline);
    b.body(50, 50, 0, Body::TxDone);
    b.body(50, 50, 0, Body::TxDone);
}

#[test]
fn a_window_closes_its_last_frame_at_the_dump() {
    let mut b = BusProbe::ZERO;
    for _ in 0..5 {
        exchange(&mut b);
    }
    assert_eq!(b.host.n, 4, "a frame closes at the next break");
    let b = b.closed();
    assert_eq!(
        b.host,
        Load {
            n: 5,
            sum: 5 * 400,
            max: 400
        }
    );
    assert_eq!(b.closed().host.n, 5, "closing twice adds nothing");
}

#[test]
fn a_wake_that_dispatches_books_as_frame_service() {
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.mark_dispatch();
    b.body(2500, 2500, 0, Body::BreakWake);
    b.mark_break();
    b.body(700, 700, 0, Body::BreakWake);
    assert_eq!(b.body[Body::Deadline as usize].max, 2500);
    assert_eq!(b.body[Body::BreakWake as usize].max, 700);
    assert_eq!(b.host.max, 2500, "the frame still holds its wake");
}

#[test]
fn a_halted_body_stays_out_of_its_frame() {
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(100, 100, 0, Body::BreakWake);
    b.body(HALT_TICKS + 1, HALT_TICKS + 1, 0, Body::Deadline);
    b.mark_break();
    b.body(100, 100, 0, Body::BreakWake);
    assert_eq!((b.halts, b.host.max), (1, 100));
}

#[test]
fn dumps_decode_field_for_field() {
    let mut k = KernelProbe::ZERO;
    for n in 0..30 {
        tick(&mut k, n, 1234 + n, n as usize % PHASES, OPEN, PLAIN);
    }
    tick(
        &mut k,
        30,
        2345,
        9,
        Regime::Closed,
        Events { tel: true, ..PLAIN },
    );
    tick(&mut k, 31, 2222, 4, Regime::OpenLoop, PLAIN);
    let after = ProbeTags {
        phase: 3,
        config_seq: 7,
        booted: true,
        slow_due: true,
        slow_finish: false,
        therm: 2,
        regime: Regime::OpenLoop,
        drive: 3,
    };
    k.tick(2000, || after, true, 9, false);
    assert_eq!(KernelProbe::from_words(words(&k)), Some(k));
    let mut bad = words(&k).to_vec();
    bad[4] |= 0xff << 16;
    assert_eq!(KernelProbe::from_words(&bad), None, "no such regime");
    let mut w = WindowProbe::ZERO;
    for n in 0..30 {
        w.tick(n * TICK_PERIOD + n, 1000 + 7 * n, Some(Regime::Closed));
        w.ended(n * TICK_PERIOD + 1500);
    }
    assert_eq!(WindowProbe::from_words(words(&w)), Some(w));
    assert_eq!(WindowProbe::from_words(&[0; 3]), None);
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(77, 900, 4, Body::BreakWake);
    b.body(66, 70, 0, Body::TxDone);
    assert_eq!(BusProbe::from_words(words(&b)), Some(b));
    assert_eq!(BusProbe::from_words(&[0; 3]), None);
}

#[test]
fn rows_flag_only_a_component_over_its_budget() {
    let mut k = KernelProbe::ZERO;
    let b = BusProbe::ZERO;
    let at = kernel_tick(OPEN, 1, PLAIN);
    tick(&mut k, 1, at + 1, 1, OPEN, PLAIN);
    tick(&mut k, 2, kernel_tick(OPEN, 2, PLAIN), 2, OPEN, PLAIN);
    tick(&mut k, 3, at + 1, 1, Regime::Closed, PLAIN);
    let s = Scenario {
        tel: false,
        frames: &[],
    };
    let mut over = heapless::Vec::<Component, 4>::new();
    rows(&k, &b, &s, |r| {
        if r.over() {
            let _ = over.push(r.component);
        }
    });
    assert_eq!(over, [Component::Tick(OPEN, 1)], "Closed rows cover it");
}

#[test]
fn a_slower_rate_budgets_its_longer_law_break() {
    assert_eq!(spin_extra(BUDGET_BAUD_HZ), 0);
    assert_eq!(spin_extra(1_000_000), LAW_BREAK_BIT_TIMES * (48 - 16));
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(1, 1, 0, Body::BreakWake);
    let over = body(Body::Deadline) + 1;
    b.body(over, over, 0, Body::Deadline);
    let frames = [(Frame::Read32, 1_000_000)];
    let s = Scenario {
        tel: false,
        frames: &frames,
    };
    let mut over = 0;
    rows(&KernelProbe::ZERO, &b, &s, |r| over += r.over() as u32);
    assert_eq!(over, 0);
}

#[test]
fn a_window_mean_over_its_budget_flags_once_judged() {
    let mut k = KernelProbe::ZERO;
    let mean = tick_body_mean(OPEN, 8);
    for n in 1..MEAN_MIN_BODIES {
        tick(&mut k, n, mean + 1, 8, OPEN, PLAIN);
    }
    let s = Scenario {
        tel: false,
        frames: &[],
    };
    let mut over = 0;
    rows(&k, &BusProbe::ZERO, &s, |r| over += r.over() as u32);
    assert_eq!(over, 0, "too few bodies to judge a mean");
    tick(&mut k, MEAN_MIN_BODIES, mean + 1, 8, OPEN, PLAIN);
    rows(&k, &BusProbe::ZERO, &s, |r| over += r.mean_over() as u32);
    assert_eq!(over, 1);
}

#[test]
fn kernel_budgets_rise_with_the_regime() {
    for p in 0..PHASES {
        let [open, closed, openloop] = Regime::ALL.map(|r| kernel_tick(r, p, PLAIN));
        assert!(
            open <= closed && open <= openloop,
            "phase {p}: {open} {closed} {openloop}"
        );
        for r in Regime::ALL {
            let (mean, max) = (KERNEL_MEAN[r as usize][p], KERNEL[r as usize][p]);
            assert!(mean <= max, "{r:?} phase {p}: {mean} {max}");
        }
    }
}

#[test]
fn every_mean_budget_sits_inside_its_maximum() {
    for b in Body::ALL {
        let (mean, max) = (body_mean(b), body(b));
        assert!(mean <= max, "{b:?}: {mean} {max}");
    }
    for f in Frame::ALL {
        let (mean, max) = (frame_mean(f), frame(f));
        assert!(mean <= max, "{f:?}: {mean} {max}");
    }
    const { assert!(TEL_ROW_MEAN <= TEL_ROW) };
}

#[test]
fn the_row_mean_is_the_tel_ticks_over_the_plain_ticks() {
    let mut k = KernelProbe::ZERO;
    let enc = TEL_ROW_MEAN + TEL_LANE_MEAN;
    let plain = KERNEL_MEAN[OPEN as usize].into_iter().min().unwrap_or(0);
    for i in 1..2 * MEAN_MIN_BODIES {
        let p = i as usize % PHASES;
        tick(&mut k, 2 * i, plain, p, OPEN, PLAIN);
        tick(
            &mut k,
            2 * i + 1,
            plain + enc + 1,
            p,
            OPEN,
            Events { tel: true, ..PLAIN },
        );
    }
    let s = Scenario {
        tel: true,
        frames: &[],
    };
    let mut over = heapless::Vec::<Component, 4>::new();
    rows(&k, &BusProbe::ZERO, &s, |r| {
        if r.over() {
            let _ = over.push(r.component);
        }
    });
    assert_eq!(over, [Component::TelRow(OPEN)]);
}

/// A TEL tick books under its own drive arm, and the row mean is judged
/// against the plain ticks of that arm: a window that drove part of the
/// time does not count the parked ticks' lower mean as the row's.
#[test]
fn tel_ticks_book_by_regime_against_their_regime_s_plain_ticks() {
    let mut k = KernelProbe::ZERO;
    let closed = Regime::Closed;
    let enc = TEL_ROW_MEAN;
    let plain = KERNEL_MEAN[closed as usize].into_iter().min().unwrap_or(0);
    let tel = Events { tel: true, ..PLAIN };
    for i in 1..2 * MEAN_MIN_BODIES {
        let p = i as usize % PHASES;
        tick(&mut k, 3 * i, plain, p, closed, PLAIN);
        tick(&mut k, 3 * i + 1, plain / 2, p, OPEN, PLAIN);
        tick(&mut k, 3 * i + 2, plain + enc, p, closed, tel);
    }
    assert_eq!((k.tel[0].n, k.tel[1].n), (0, 2 * MEAN_MIN_BODIES - 1));
    assert_eq!(k.tel_max[1][1], (plain + enc) as u16);
    let s = Scenario {
        tel: true,
        frames: &[],
    };
    let mut encode = heapless::Vec::<(Component, u32), 2>::new();
    rows(&k, &BusProbe::ZERO, &s, |r| {
        if let Component::TelRow(_) = r.component {
            let _ = encode.push((r.component, r.mean));
        }
    });
    assert_eq!(
        encode,
        [
            (Component::TelRow(OPEN), 0),
            (Component::TelRow(closed), enc)
        ]
    );
}

/// A tick that closed a tick-load window stays off the plain and TEL rows
/// and books under its own, charged the close on top of what it carried.
#[test]
fn a_health_close_books_apart() {
    let mut k = KernelProbe::ZERO;
    let closed = Regime::Closed;
    let health = Events {
        health: true,
        ..PLAIN
    };
    tick(&mut k, 1, 1300, 5, closed, PLAIN);
    tick(&mut k, 2, 1400, 5, closed, health);
    let tel = Events {
        tel: true,
        ..health
    };
    tick(&mut k, 3, 1900, 1, closed, tel);
    assert_eq!(k.load(closed, 5).max, 1300);
    assert_eq!((k.health_max[1][5], k.health_max[1][1]), (1400, 1900));
    assert_eq!((k.tel[1].n, k.tel_max[1][1]), (0, 0));
    let s = Scenario {
        tel: true,
        frames: &[],
    };
    let mut budget = 0;
    rows(&k, &BusProbe::ZERO, &s, |r| {
        if r.component == Component::Health(closed, 1) {
            budget = r.budget;
        }
    });
    let carried = Events { bank: true, ..tel };
    assert_eq!(budget, kernel_tick(closed, 1, carried));
    assert_eq!(
        kernel_tick(closed, 1, carried),
        kernel_tick(
            closed,
            1,
            Events {
                health: false,
                ..carried
            }
        ) + HEALTH_CLOSE
    );
}

#[test]
fn gamma_is_the_heaviest_run_of_k_ticks() {
    let mut w = Windows::ZERO;
    let bodies = (0..50u32).map(|t| if t == 30 { 2000 } else { 100 + t });
    let all: heapless::Vec<u32, 50> = bodies.clone().collect();
    for b in bodies {
        w.add(b);
    }
    for (j, k) in WINDOW_TICKS.into_iter().enumerate() {
        let best = all.windows(k).map(|s| s.iter().sum::<u32>()).max();
        assert_eq!(Some(w.gamma[j]), best, "k = {k}");
    }
}

/// Many laps of the ring, the running total wrapping its 16 bits about
/// every 40 ticks: each gamma(k) is still the heaviest run of k bodies.
#[test]
fn gamma_holds_across_laps_and_a_wrapping_total() {
    let mut w = Windows::ZERO;
    let all: heapless::Vec<u32, 2000> = (0..2000u32).map(|t| 1200 + (t * 7919) % 1300).collect();
    for &b in &all {
        w.add(b);
    }
    for (j, k) in WINDOW_TICKS.into_iter().enumerate() {
        let best = all.windows(k).map(|s| s.iter().sum::<u32>()).max();
        assert_eq!(Some(w.gamma[j]), best, "k = {k}");
    }
}

#[test]
fn a_configuration_take_charges_its_row() {
    let refresh = Events {
        refresh: true,
        ..PLAIN
    };
    let (closed, p) = (Regime::Closed as usize, phase::SPARE_A as usize);
    assert_eq!(
        kernel_tick(Regime::Closed, p, refresh),
        KERNEL[closed][p] + TICK_PUBLISH[closed][p] + LAYOUT + REFRESH
    );
}

/// The chip closes a tick-load window on every twentieth tick, the first
/// at the boot count's first PUBLISH tick: every close lands on PUBLISH,
/// never on a SLOW, thermometer or configuration take's phase.
#[test]
fn the_health_close_lands_only_on_publish() {
    let lands: heapless::Vec<usize, PHASES> =
        (0..PHASES).filter(|&p| health_close_lands(p)).collect();
    assert_eq!(lands, [phase::PUBLISH as usize]);
    let closes = (0..4000u64).filter(|&t| health_close_at(t));
    assert!(
        closes
            .clone()
            .all(|t| t as usize % PHASES == phase::PUBLISH as usize)
    );
    assert_eq!(closes.count(), 4000 / HEALTH_CLOSE_TICKS as usize);
}

/// A SLOW or thermometer stage tick holds the TX lane: its TEL tick
/// carries the poll and a head or a bank, never a lane step. A configuration
/// take does not hold.
#[test]
fn a_stage_tick_holds_the_tel_lane() {
    let closed = Regime::Closed;
    let tel = Events { tel: true, ..PLAIN };
    let bank = Events { bank: true, ..tel };
    let stages = [
        (
            phase::SLOW,
            Events {
                slow: true,
                ..PLAIN
            },
        ),
        (
            phase::SLOW,
            Events {
                finish: true,
                ..PLAIN
            },
        ),
        (
            phase::SLOW,
            Events {
                close: true,
                finish: true,
                ..PLAIN
            },
        ),
        (
            phase::SPARE_A,
            Events {
                install: true,
                ..PLAIN
            },
        ),
        (
            phase::SPARE_B,
            Events {
                side: true,
                ..PLAIN
            },
        ),
    ];
    for (p, stage) in stages {
        let p = p as usize;
        let alone = kernel_tick(closed, p, stage);
        let with = |c: Events| {
            let ev = Events {
                slow: stage.slow,
                finish: stage.finish,
                close: stage.close,
                install: stage.install,
                side: stage.side,
                ..c
            };
            kernel_tick(closed, p, ev) - alone
        };
        assert_eq!(with(tel), TEL_ROW + TEL_POLL + TEL_HEAD, "{stage:?}");
        let held = TEL_ROW + TEL_POLL + TEL_HEAD.max(TEL_BANK);
        assert_eq!(with(bank), held, "{stage:?}");
    }
    let refresh = Events {
        refresh: true,
        ..tel
    };
    let p = phase::SPARE_A as usize;
    let lane = kernel_tick(closed, p, refresh)
        - kernel_tick(
            closed,
            p,
            Events {
                tel: false,
                ..refresh
            },
        );
    assert_eq!(lane, TEL_ROW + tel_lane(false));
}

#[test]
fn a_finish_charges_its_own_row_and_a_close_replaces_it() {
    let closed = Regime::Closed;
    let p = phase::SLOW as usize;
    let finish = Events {
        finish: true,
        ..PLAIN
    };
    let base = kernel_tick(closed, p, PLAIN);
    assert_eq!(kernel_tick(closed, p, finish), base + SLOW_FINISH);
    let close = Events {
        close: true,
        ..finish
    };
    assert_eq!(kernel_tick(closed, p, close), base + SLOW_CLOSE);
}

/// A TEL step books its stamps' time less the stamp pair's own, never
/// below zero, into its step and the running tick's total; a halt between
/// the stamps books nowhere. The record decodes from its words.
#[test]
fn tel_steps_book_less_the_stamp_pair_and_skip_halts() {
    let mut c = TelCost {
        cal: 12,
        ..TelCost::ZERO
    };
    c.add(TelStep::Row, 112);
    c.add(TelStep::Row, 62);
    c.add(TelStep::Poll, 5);
    c.add(TelStep::Poll, HALT_TICKS + 1);
    assert_eq!(
        c.load(TelStep::Row),
        Load {
            n: 2,
            sum: 150,
            max: 100
        }
    );
    assert_eq!(
        c.load(TelStep::Poll),
        Load {
            n: 1,
            sum: 0,
            max: 0
        }
    );
    assert_eq!(c.load(TelStep::Bank), Load::ZERO);
    assert_eq!(c.tick, 150);
    assert_eq!(TelCost::from_words(words(&c)), Some(c));
}

/// The tally books a tick where the kernel record books it as TEL or
/// banking: an unstamped burst's body by phase and its hook time, a
/// stamped burst's steps, a burst's first tick apart, a plain tick's hook
/// time by regime, and nothing the record books elsewhere.
#[test]
fn tel_tally_books_unstamped_bodies_and_stamped_steps() {
    let mut k = KernelProbe::ZERO;
    let mut t = TelTally::ZERO;
    let closed = Regime::Closed;
    let tel = Events { tel: true, ..PLAIN };
    let bank = Events { bank: true, ..tel };
    let ticks = [
        (0, 900, tel, false, None, 1),
        (1, 1500, tel, true, Some(700), 2),
        (2, 1100, tel, false, None, 300),
        (3, 1200, tel, false, None, 310),
        (4, 1700, bank, false, None, 500),
        (5, 1300, Events { slow: true, ..tel }, false, None, 4),
        (
            6,
            1300,
            Events {
                health: true,
                ..bank
            },
            false,
            None,
            8,
        ),
        (7, 1000, PLAIN, false, None, 20),
        (8, 1400, tel, false, Some(300), 16),
        (9, 1600, bank, false, Some(450), 32),
        (10, HALT_TICKS + 1, PLAIN, false, None, 64),
    ];
    for (n, body, ev, first, steps, hook) in ticks {
        let p = n as usize % PHASES;
        tick(&mut k, n, body, p, closed, ev);
        t.tick(body, p, closed, ev, first, steps, hook, n == 0);
    }
    assert_eq!(
        t.first[1],
        Load {
            n: 1,
            sum: 1500,
            max: 1500
        }
    );
    assert_eq!(t.tel_n[1][..4], [0, 0, 1, 1]);
    assert_eq!(t.tel_sum[1], 2300);
    assert_eq!((t.bank_n[1][4], t.bank_sum[1]), (1, 1700));
    assert_eq!(t.bank_n[1].iter().sum::<u16>(), 1);
    assert_eq!((t.tel_steps.n, t.tel_steps.sum), (1, 300));
    assert_eq!((t.bank_steps.n, t.bank_steps.sum), (1, 450));
    assert_eq!(t.tel_n[0], [0; PHASES]);
    assert_eq!((t.hook_tel, t.hook_bank), (610, 500));
    assert_eq!(t.hook_plain, [Tally::ZERO, Tally { n: 1, sum: 20 }]);
    assert_eq!(TelTally::from_words(words(&t)), Some(t));
}

/// The TEL cost holds each unstamped TEL and banking tick against the
/// plain ticks of its own phase: phases that differ by hundreds of ticks,
/// and TEL ticks that land on two of them, still read the row's true
/// excess, where the all-phase mean does not. A banking tick weighs one in
/// a frame, in the cost and in the stamped steps alike.
#[test]
fn tel_cost_matches_each_tel_tick_to_its_phase() {
    let mut k = KernelProbe::ZERO;
    let mut t = TelTally::ZERO;
    let closed = Regime::Closed;
    let (enc, banked) = (120, 400);
    let plain = |p: usize| 1000 + 100 * p as u32;
    let tel = Events { tel: true, ..PLAIN };
    let bank = Events { bank: true, ..tel };
    let mut n = 1;
    let mut run = |k: &mut KernelProbe, body: u32, p: usize, ev: Events, steps: Option<u32>| {
        tick(k, n, body, p, closed, ev);
        t.tick(body, p, closed, ev, false, steps, 0, false);
        n += 1;
    };
    for p in 0..PHASES {
        for _ in 0..=p {
            run(&mut k, plain(p), p, PLAIN, None);
        }
    }
    for _ in 0..2 * MEAN_MIN_BODIES {
        run(&mut k, plain(0) + enc, 0, tel, None);
        run(&mut k, plain(9) + enc, 9, tel, None);
        run(&mut k, plain(4) + banked, 4, bank, None);
        run(&mut k, plain(2) + enc + 50, 2, tel, Some(100));
        run(&mut k, plain(3) + banked, 3, bank, Some(300));
    }
    let r = TelReport::of(&k, &t);
    assert_eq!((r.tel, r.tel_n), (Some(enc as f32), 4 * MEAN_MIN_BODIES));
    assert_eq!(
        (r.bank, r.bank_n),
        (Some(banked as f32), 2 * MEAN_MIN_BODIES)
    );
    let w = 1.0 / crate::tel::STREAM_SAMPLES_MAX as f32;
    let mix = |a: f32, b: f32| (1.0 - w) * a + w * b;
    let near = |x: Option<f32>, want: f32| x.is_some_and(|x| (x - want).abs() < 1e-3);
    assert!(near(r.per_tick, mix(enc as f32, banked as f32)), "{r:?}");
    assert!(near(r.steps, mix(100.0, 300.0)), "{r:?}");
    assert_eq!(r.steps_n, 4 * MEAN_MIN_BODIES);
    assert_eq!(TelReport::tel_row(&k, &t, closed), Some(enc as f32));
    assert_eq!(TelReport::tel_row(&k, &t, OPEN), None);
    let s = Scenario {
        tel: true,
        frames: &[],
    };
    let mut pooled = 0;
    rows(&k, &BusProbe::ZERO, &s, |r| {
        if r.component == Component::TelRow(closed) {
            pooled = r.mean;
        }
    });
    assert_ne!(pooled, enc, "the all-phase mean misreads the row");
}

/// A TEL tick on a phase with no plain tick has nothing to match.
#[test]
fn tel_cost_needs_a_plain_tick_on_every_tel_phase() {
    let mut k = KernelProbe::ZERO;
    let mut t = TelTally::ZERO;
    let tel = Events { tel: true, ..PLAIN };
    tick(&mut k, 1, 1000, 1, OPEN, PLAIN);
    tick(&mut k, 2, 1200, 2, OPEN, tel);
    t.tick(1200, 2, OPEN, tel, false, None, 0, false);
    let r = TelReport::of(&k, &t);
    assert_eq!((r.tel, r.per_tick), (None, None));
}

/// A telcost window, steady state: every plain tick of a regime runs the
/// hooks' flag tests in `plain_hook`, every TEL tick the same plus the
/// steps, the `glue` no step stamps, and `outside` cycles the TX DMA takes
/// from the rest of the tick; a stamped burst books the steps alone.
fn telcost_window(steps: u32, glue: u32, outside: u32) -> (KernelProbe, TelTally) {
    let mut k = KernelProbe::ZERO;
    let mut t = TelTally::ZERO;
    let tel = Events { tel: true, ..PLAIN };
    let bank = Events { bank: true, ..tel };
    let (bank_steps, plain_hook) = (steps + 200, [24, 31]);
    let mut n = 1;
    let mut run = |k: &mut KernelProbe, body, p, r: Regime, ev, steps, hook| {
        tick(k, n, body, p, r, ev);
        t.tick(body, p, r, ev, false, steps, hook, false);
        n += 1;
    };
    for r in Regime::EVENT_ROWS {
        let hook = plain_hook[r as usize];
        let plain = |p: usize| 1000 + 100 * p as u32 + 300 * r as u32;
        for p in 0..PHASES {
            run(&mut k, plain(p), p, r, PLAIN, None, hook);
        }
        for i in 0..2 * MEAN_MIN_BODIES as usize {
            let p = i % PHASES;
            let (ev, s) = if i % 16 == 15 {
                (bank, bank_steps)
            } else {
                (tel, steps)
            };
            let x = s + glue;
            run(&mut k, plain(p) + x + outside, p, r, ev, None, hook + x);
            run(&mut k, plain(p) + x + 90, p, r, ev, Some(s), hook + x + 90);
        }
    }
    (k, t)
}

/// The glue the hooks run beyond the steps is measured, whatever its size:
/// a codegen shift to 45 HCLK, past the 34 the old fixed slack allowed,
/// still passes, and the report reads it and the DMA's share apart.
#[test]
fn tel_self_check_measures_the_glue() {
    let (k, t) = telcost_window(250, 45, 9);
    let r = TelReport::of(&k, &t);
    let near = |x: Option<f32>, want: f32| x.is_some_and(|x| (x - want).abs() < 0.05);
    let w = 1.0 / crate::tel::STREAM_SAMPLES_MAX as f32;
    let steps = (1.0 - w) * 250.0 + w * 450.0;
    assert!(near(r.steps, steps), "{r:?}");
    assert!(near(r.glue(), 45.0), "{r:?}");
    assert!(near(r.outside(), 9.0), "{r:?}");
    assert!(near(r.per_tick, steps + 54.0), "{r:?}");
    assert_eq!(r.check(), Some(true));
}

/// The check fails a step stamped wider than its hook and hooks wider than
/// the tick beyond the per-tick noise, and holds no verdict without hooks.
#[test]
fn tel_self_check_fails_on_crossed_stamps() {
    let crossed = TelReport::of(&KernelProbe::ZERO, &TelTally::ZERO);
    assert_eq!(crossed.check(), None);
    let mut r = TelReport {
        steps: Some(250.0),
        hooks: Some(240.0),
        per_tick: Some(260.0),
        ..crossed
    };
    assert_eq!(r.check(), Some(false), "steps past the hooks");
    r.hooks = Some(260.0 + TEL_PER_TICK_NOISE - 1.0);
    assert_eq!(r.check(), Some(true), "inside the per-tick noise");
    r.hooks = Some(260.0 + TEL_PER_TICK_NOISE + 1.0);
    assert_eq!(r.check(), Some(false), "hooks past the tick");
}

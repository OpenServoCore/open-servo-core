use super::probe::{
    BusProbe, Component, HALT_TICKS, KernelProbe, LONG_TICKS, Load, Scenario, Windows, rows,
};
use super::*;

const PLAIN: Events = Events {
    refresh: false,
    tel: false,
    bank: false,
    slow: false,
    close: false,
};
const OPEN: Regime = Regime::Open;

/// A kernel tick on the grid: tick `n` entered `n` periods in.
fn tick(k: &mut KernelProbe, n: u32, body: u32, phase: usize, r: Regime, ev: Events) {
    let entry = n * TICK_PERIOD;
    k.tick(entry, body, phase, r, ev, n == 0);
    k.ended(entry + body, entry + body);
}

fn words<T>(t: &T) -> &[u32] {
    // SAFETY: the probe records are repr(C) runs of u32.
    unsafe {
        core::slice::from_raw_parts(t as *const T as *const u32, core::mem::size_of::<T>() / 4)
    }
}

#[test]
fn kernel_ticks_book_by_what_they_carried() {
    let mut k = KernelProbe::ZERO;
    let closed = Regime::Closed;
    tick(&mut k, 0, 1000, 0, OPEN, PLAIN);
    tick(&mut k, 1, 1100, 1, OPEN, PLAIN);
    tick(&mut k, 2, 1500, 2, OPEN, Events { tel: true, ..PLAIN });
    let bank = Events {
        tel: true,
        bank: true,
        ..PLAIN
    };
    tick(&mut k, 3, 1700, 3, OPEN, bank);
    tick(
        &mut k,
        4,
        2000,
        0,
        OPEN,
        Events {
            refresh: true,
            ..PLAIN
        },
    );
    tick(&mut k, 5, HALT_TICKS + 1, 4, OPEN, PLAIN);
    tick(&mut k, 6, LONG_TICKS, 1, closed, PLAIN);
    tick(
        &mut k,
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
        slow: true,
        close: true,
        ..PLAIN
    };
    tick(&mut k, 8, 2200, 6, closed, close);
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
    assert_eq!((k.tel_max[2], k.bank_max[3]), (1500, 1700));
    assert_eq!(
        (k.refresh.max, k.slow[1], k.close),
        (2000, one(1900), one(2200))
    );
    assert_eq!((k.halts, k.long), (1, [3, 3]));
    let bodies = 1000 + 1100 + 1500 + 1700 + 2000 + HALT_TICKS + 1 + LONG_TICKS + 1900 + 2200;
    assert_eq!(k.busy, bodies);
    assert_eq!(k.drove(), Regime::Closed);
}

/// Lateness counts only what held the tick off past both its grid slot
/// and its predecessor's end.
#[test]
fn entry_lateness_is_measured_from_the_grid_and_the_last_end() {
    let mut k = KernelProbe::ZERO;
    k.tick(0, 900, 0, OPEN, PLAIN, true);
    k.ended(900, 900);
    k.tick(TICK_PERIOD + 70, 900, 1, OPEN, PLAIN, false);
    k.ended(TICK_PERIOD + 970, TICK_PERIOD + 970);
    assert_eq!(k.late_max, 70);
    let over = 3 * TICK_PERIOD;
    k.tick(2 * TICK_PERIOD + 70, over, 2, OPEN, PLAIN, false);
    let end = 5 * TICK_PERIOD + 70;
    k.ended(end, end);
    k.tick(end + ENTRY_EXIT + 10, 900, 3, OPEN, PLAIN, false);
    assert_eq!(k.late_max, 70, "a tick the last one overran is not late");
}

#[test]
fn bus_frames_fold_at_breaks_and_at_starts_after_the_reply() {
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(100, Body::BreakWake);
    b.mark_start();
    b.body(200, Body::Deadline);
    b.body(50, Body::TxDone);
    b.mark_start();
    b.body(300, Body::TelStage);
    b.body(40, Body::TxDone);
    b.body(10, Body::TelStage);
    b.mark_start();
    b.body(60, Body::TxDone);
    b.mark_break();
    b.body(90, Body::BreakWake);
    assert_eq!(
        b.host,
        Load {
            n: 1,
            sum: 350,
            max: 350
        },
        "the reply stays in its frame"
    );
    assert_eq!(
        b.tel,
        Load {
            n: 2,
            sum: 410,
            max: 350
        },
        "staged, then chained from a TX done"
    );
    assert_eq!(b.body[Body::TelStage as usize].n, 1);
    assert_eq!(b.body[Body::TelPoll as usize].max, 10);
    assert_eq!(b.body[Body::TxDone as usize].n, 3);
}

/// One polled exchange as the chip books it: the wake, the deadline body
/// that triggers the reply, the reply's two arms.
fn exchange(b: &mut BusProbe) {
    b.mark_break();
    b.body(100, Body::BreakWake);
    b.mark_dispatch();
    b.mark_start();
    b.body(200, Body::Deadline);
    b.body(50, Body::TxDone);
    b.body(50, Body::TxDone);
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
    let mut t = BusProbe::ZERO;
    exchange(&mut t);
    for _ in 0..3 {
        t.mark_start();
        t.body(300, Body::TelStage);
    }
    let t = t.closed();
    assert_eq!((t.host.n, t.tel.n), (1, 3));
}

#[test]
fn a_wake_that_dispatches_books_as_frame_service() {
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.mark_dispatch();
    b.mark_start();
    b.body(2500, Body::BreakWake);
    b.mark_break();
    b.body(700, Body::BreakWake);
    assert_eq!(b.body[Body::Deadline as usize].max, 2500);
    assert_eq!(b.body[Body::BreakWake as usize].max, 700);
    assert_eq!(b.host.max, 2500, "the frame still holds its wake");
}

#[test]
fn a_halted_body_stays_out_of_its_frame() {
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(100, Body::BreakWake);
    b.body(HALT_TICKS + 1, Body::Deadline);
    b.mark_break();
    b.body(100, Body::BreakWake);
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
    assert_eq!(KernelProbe::from_words(words(&k)), Some(k));
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(77, Body::BreakWake);
    b.breaks = 5;
    b.rises = 2;
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
    b.body(1, Body::BreakWake);
    b.body(body(Body::Deadline) + 1, Body::Deadline);
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
    let mean = KERNEL_MEAN[OPEN as usize][8];
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
        let [open, closed] = Regime::ALL.map(|r| kernel_tick(r, p, PLAIN));
        assert!(open <= closed, "phase {p}: {open} {closed}");
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
    for r in Regime::ALL {
        let (mean, max) = (TEL_ENCODE_MEAN[r as usize], TEL_ENCODE[r as usize]);
        assert!(mean <= max, "{r:?}: {mean} {max}");
    }
}

#[test]
fn the_encode_mean_is_the_tel_ticks_over_the_plain_ticks() {
    let mut k = KernelProbe::ZERO;
    let enc = TEL_ENCODE_MEAN[Regime::Open as usize];
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
    assert_eq!(over, [Component::TelEncode]);
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

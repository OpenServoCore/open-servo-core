use super::probe::{
    BusProbe, Component, HALT_TICKS, KernelProbe, LONG_TICKS, Load, Scenario, Windows, rows,
};
use super::*;

const PLAIN: Events = Events {
    refresh: false,
    tel: false,
    bank: false,
    slow: false,
    finish: false,
    close: false,
    install: false,
    side: false,
    health: false,
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
    let refresh = Events {
        refresh: true,
        ..PLAIN
    };
    tick(&mut k, 4, 2000, 8, OPEN, refresh);
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
        finish: true,
        close: true,
        ..PLAIN
    };
    tick(&mut k, 8, 2200, 6, closed, close);
    let install = Events {
        install: true,
        ..PLAIN
    };
    tick(&mut k, 9, 1600, 8, closed, install);
    let side = Events {
        side: true,
        ..PLAIN
    };
    tick(&mut k, 10, 1700, 9, closed, side);
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
        ([one(2000), Load::ZERO], one(1900), one(2200))
    );
    assert_eq!((k.install, k.side), (one(1600), one(1700)));
    assert_eq!(
        k.load(closed, 8),
        Load::ZERO,
        "the stages stay off the plain rows"
    );
    assert_eq!((k.halts, k.long), (1, [3, 5]));
    tick(&mut k, 11, 1800, 9, OPEN, refresh);
    assert_eq!(k.refresh, [one(2000), one(1800)], "booked by stage");
    let finish = Events {
        finish: true,
        ..PLAIN
    };
    tick(&mut k, 12, 1750, 6, closed, finish);
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
    b.body(100, 100, 0, Body::BreakWake);
    b.mark_start();
    b.body(200, 200, 0, Body::Deadline);
    b.body(50, 50, 0, Body::TxDone);
    b.mark_start();
    b.body(300, 300, 0, Body::TelStage);
    b.body(40, 40, 0, Body::TxDone);
    b.body(10, 10, 0, Body::TelStage);
    b.mark_start();
    b.body(60, 60, 0, Body::TxDone);
    b.mark_break();
    b.body(90, 90, 0, Body::BreakWake);
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

/// A bus body subtracts every kernel vector that ran inside it, each with
/// its probe update and its interrupt entry and exit; the longest net body
/// keeps its wall time and nesting.
#[test]
fn a_bus_body_nets_out_each_kernel_vector_with_its_entry_and_exit() {
    let mut k = KernelProbe::ZERO;
    tick(&mut k, 0, 900, 0, OPEN, PLAIN);
    let (at, kernel, entries) = (TICK_PERIOD - 300, k.busy, k.entries);
    k.tick(TICK_PERIOD, 1500, 1, OPEN, PLAIN, false);
    k.ended(TICK_PERIOD + 1500, TICK_PERIOD + 1530);
    k.other(200);
    let wall = 2 * TICK_PERIOD - at;
    let net = wall - k.busy.wrapping_sub(kernel);
    assert_eq!(net, wall - (1500 + 30 + 200 + 2 * ENTRY_EXIT));
    let nested = k.entries.wrapping_sub(entries);
    assert_eq!(nested, 2);
    let mut b = BusProbe::ZERO;
    b.body(net, wall, nested, Body::TelStage);
    b.body(10, wall + 1000, 5, Body::TelStage);
    let i = Body::TelPoll as usize;
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
    b.mark_start();
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
    let mut t = BusProbe::ZERO;
    exchange(&mut t);
    for _ in 0..3 {
        t.mark_start();
        t.body(300, 300, 0, Body::TelStage);
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
    assert_eq!(KernelProbe::from_words(words(&k)), Some(k));
    let mut b = BusProbe::ZERO;
    b.mark_break();
    b.body(77, 900, 4, Body::BreakWake);
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
    let enc = TEL_ENCODE_MEAN[Regime::Open as usize] + TEL_LANE_MEAN;
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
    assert_eq!(over, [Component::TelEncode(OPEN)]);
}

/// A TEL tick books under its own drive arm, and the encode mean is judged
/// against the plain ticks of that arm: a window that drove part of the
/// time does not count the parked ticks' lower mean as encode.
#[test]
fn tel_ticks_book_by_regime_against_their_regime_s_plain_ticks() {
    let mut k = KernelProbe::ZERO;
    let closed = Regime::Closed;
    let enc = TEL_ENCODE_MEAN[closed as usize];
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
        if let Component::TelEncode(_) = r.component {
            let _ = encode.push((r.component, r.mean));
        }
    });
    assert_eq!(
        encode,
        [
            (Component::TelEncode(OPEN), 0),
            (Component::TelEncode(closed), enc)
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

#[test]
fn each_refresh_stage_charges_its_own_row() {
    let refresh = Events {
        refresh: true,
        ..PLAIN
    };
    let closed = Regime::Closed as usize;
    for (p, row) in [
        (phase::THERM_INSTALL as usize, REFRESH_1),
        (phase::THERM_SIDE as usize, REFRESH_2),
    ] {
        assert_eq!(
            kernel_tick(Regime::Closed, p, refresh),
            KERNEL[closed][p] + row
        );
    }
}

/// The chip closes a tick-load window on every sixteenth tick from the
/// boot tick at CONTROL: with ten phases that is every odd phase and never
/// an even one, so never the SLOW stages or the install's phase.
#[test]
fn the_health_close_lands_only_on_odd_phases() {
    let lands: heapless::Vec<usize, PHASES> =
        (0..PHASES).filter(|&p| health_close_lands(p)).collect();
    assert_eq!(lands, [1, 3, 5, 7, 9]);
    for p in [phase::SLOW, phase::THERM_INSTALL] {
        assert!(!health_close_lands(p as usize), "phase {p}");
    }
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

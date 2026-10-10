use super::probe::{BusProbe, Component, HALT_TICKS, KernelProbe, Load, Scenario, rows};
use super::*;

const PLAIN: Events = Events {
    refresh: false,
    tel: false,
    bank: false,
};

fn words<T>(t: &T) -> &[u32] {
    // SAFETY: the probe records are repr(C) runs of u32.
    unsafe {
        core::slice::from_raw_parts(t as *const T as *const u32, core::mem::size_of::<T>() / 4)
    }
}

#[test]
fn kernel_ticks_book_by_what_they_carried() {
    let mut k = KernelProbe::ZERO;
    k.tick(1000, 0, PLAIN, true);
    k.tick(1100, 1, PLAIN, false);
    k.tick(1500, 2, Events { tel: true, ..PLAIN }, false);
    k.tick(
        1700,
        3,
        Events {
            tel: true,
            bank: true,
            ..PLAIN
        },
        false,
    );
    k.tick(
        2000,
        0,
        Events {
            refresh: true,
            ..PLAIN
        },
        false,
    );
    k.tick(HALT_TICKS + 1, 4, PLAIN, false);
    assert_eq!(k.tick[0], Load::ZERO, "the boot tick books nothing");
    assert_eq!(
        k.tick[1],
        Load {
            n: 1,
            sum: 1100,
            max: 1100
        }
    );
    assert_eq!((k.tel_max[2], k.bank_max[3]), (1500, 1700));
    assert_eq!(k.refresh.max, 2000);
    assert_eq!(k.halts, 1);
    assert_eq!(k.busy, 1000 + 1100 + 1500 + 1700 + 2000 + HALT_TICKS + 1);
    assert_eq!(k.hist.iter().sum::<u32>(), 4);
}

#[test]
fn a_period_sums_its_ten_ticks() {
    let mut k = KernelProbe::ZERO;
    for p in 0..2 * PHASES + 1 {
        k.tick(100 + p as u32, p % PHASES, PLAIN, false);
    }
    let first: u32 = (0..PHASES as u32).map(|p| 100 + p).sum();
    let second: u32 = (PHASES as u32..2 * PHASES as u32).map(|p| 100 + p).sum();
    assert_eq!(k.period.n, 2);
    assert_eq!(k.period.max, first.max(second));
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
    k.tick(1234, 7, PLAIN, false);
    k.tick(2345, 9, Events { tel: true, ..PLAIN }, false);
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
    let at = kernel_tick(Regime::Quiet, 1, PLAIN);
    k.tick(at + 1, 1, PLAIN, false);
    k.tick(kernel_tick(Regime::Quiet, 2, PLAIN), 2, PLAIN, false);
    let s = Scenario {
        regime: Regime::Quiet,
        tel: false,
        frames: &[],
    };
    let mut over = heapless::Vec::<Component, 4>::new();
    rows(&k, &b, &s, |r| {
        if r.over() {
            let _ = over.push(r.component);
        }
    });
    assert_eq!(over, [Component::Tick(1)]);
    let s = Scenario {
        regime: Regime::Moving,
        ..s
    };
    let mut over = 0;
    rows(&k, &b, &s, |r| over += r.over() as u32);
    assert_eq!(over, 0, "the moving budget covers the quiet tick");
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
        regime: Regime::Quiet,
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
    let mean = KERNEL_MEAN[Regime::Quiet as usize][8];
    for _ in 1..MEAN_MIN_BODIES {
        k.tick(mean + 1, 8, PLAIN, false);
    }
    let s = Scenario {
        regime: Regime::Quiet,
        tel: false,
        frames: &[],
    };
    let mut over = 0;
    rows(&k, &BusProbe::ZERO, &s, |r| over += r.over() as u32);
    assert_eq!(over, 0, "too few bodies to judge a mean");
    k.tick(mean + 1, 8, PLAIN, false);
    rows(&k, &BusProbe::ZERO, &s, |r| over += r.mean_over() as u32);
    assert_eq!(over, 1);
}

#[test]
fn kernel_budgets_rise_with_the_regime() {
    for p in 0..PHASES {
        let [q, h, m] = Regime::ALL.map(|r| kernel_tick(r, p, PLAIN));
        assert!(q <= h && h <= m, "phase {p}: {q} {h} {m}");
        for r in Regime::ALL {
            let (mean, max) = (KERNEL_MEAN[r as usize][p], KERNEL[r as usize][p]);
            assert!(
                mean <= max && 2 * mean >= max,
                "{r:?} phase {p}: {mean} {max}"
            );
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
    let enc = TEL_ENCODE_MEAN[Regime::Quiet as usize];
    for i in 0..2 * MEAN_MIN_BODIES {
        let p = i as usize % PHASES;
        k.tick(800, p, PLAIN, false);
        k.tick(800 + enc + 1, p, Events { tel: true, ..PLAIN }, false);
    }
    let s = Scenario {
        regime: Regime::Quiet,
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

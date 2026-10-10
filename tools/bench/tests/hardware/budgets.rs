//! Kernel and bus budgets on silicon (`osc_servo_core::budget`): each
//! budgeted regime and traffic type runs in its own window, opened by a
//! reboot (the probe starts at zero) and read over the debug link. Fails
//! when any component's maximum, or a mean judged over enough bodies,
//! exceeds its budget, naming the window, the component, the measured
//! ticks and us, and the budget.
//!
//! Needs the bench image (`--features bench`) flashed, `BENCH_ELF` naming
//! that ELF, and `wlink` on PATH with the probe attached; flashes nothing.
//! Opt-in: `cargo test --test hardware -- --ignored budgets --test-threads=1`;
//! `BUDGET_WINDOWS` (comma-separated names, e.g. `tel still,tel stepping`)
//! runs only those windows.
//! Drives the motor: a hold and 600-count steps about the rail middle, each
//! drive ending centred with torque off; the last window ends rebooted.

use std::env;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::thread::sleep;
use std::time::Duration;

use bench::osc::{
    REBOOT_SETTLE_MS, build_instruction, build_ping, build_read, build_reboot, build_write,
    gread_uniform_payload,
};
use bench::probe::{self, Reader, Snapshot};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::Mode;
use osc_servo_core::budget::probe::{Component, Row, Scenario, rows, us};
use osc_servo_core::budget::{Frame, Regime};
use osc_servo_core::regions::config::addr::common::MODEL_NUMBER;
use osc_servo_core::regions::control::addr::lifecycle::{
    GOAL_POSITION, MODE, TEL_COUNT, TEL_MASK, TORQUE_ENABLE,
};
use osc_servo_core::regions::telemetry::addr::estimates::SAMPLE_TICK;
use serial_test::serial;

use crate::support::{Bench, bench};

const STEP_COUNTS: i32 = 600;
const STEP_HOLD: Duration = Duration::from_millis(400);
const WINDOW: Duration = Duration::from_secs(10);
const STEP_WINDOW: Duration = Duration::from_secs(20);
const CENTRE_SETTLE: Duration = Duration::from_millis(500);
/// The bench soaks' six-field mask (202 B frames at 3M).
const SIX_FIELDS: u16 = 0x1cd;
const TEL_ROWS: u16 = 4000;
const TEL_BURSTS: u32 = 10;
/// A burst's wire time with its drops, and margin: a host break inside it
/// would abort it.
const TEL_BURST_WAIT: Duration = Duration::from_millis(600);
const POLLS: u32 = 2000;
const GREADS: u32 = 200;
/// Top of the unicast range: never assigned, so slot 0 stays silent.
const PHANTOM_ID: u8 = 249;
const BROADCAST: u8 = 0xFE;
const SLOW_BAUD: u32 = 1_000_000;

struct Window {
    name: &'static str,
    regime: Regime,
    tel: bool,
    frames: Vec<(Frame, u32)>,
    snap: Snapshot,
}

#[serial]
#[test]
#[ignore = "needs the bench image: BENCH_ELF and wlink"]
fn kernel_and_bus_stay_inside_their_budgets() {
    let path = env::var("BENCH_ELF").expect("BENCH_ELF: the flashed bench image's ELF");
    let elf = std::fs::read(&path).expect("read BENCH_ELF");
    let reader = Reader::from_elf(&elf).expect("budget probe symbols");
    let mut b = bench();
    let fast = b.home_baud();
    let id = b.id();
    let mid = b.goal_mid();
    let mut w = Vec::new();
    let mut failed = Vec::new();

    w.extend(window(
        &mut b,
        &reader,
        "quiet",
        Regime::Quiet,
        false,
        &[],
        |_| sleep(WINDOW),
    ));
    let writes = [(Frame::Write, fast)];
    w.extend(window(
        &mut b,
        &reader,
        "hold",
        Regime::Hold,
        false,
        &writes,
        |b| {
            driven(b, |b| {
                hold_at(b, mid);
                sleep(WINDOW);
            })
        },
    ));
    w.extend(window(
        &mut b,
        &reader,
        "stepping",
        Regime::Moving,
        false,
        &writes,
        |b| {
            driven(b, |b| {
                hold_at(b, mid);
                for k in 0..(STEP_WINDOW.as_millis() / STEP_HOLD.as_millis()) {
                    let goal = mid
                        + if k % 2 == 0 {
                            STEP_COUNTS
                        } else {
                            -STEP_COUNTS
                        };
                    write(b, GOAL_POSITION, &goal.to_le_bytes());
                    sleep(STEP_HOLD);
                }
            })
        },
    ));
    w.extend(window(
        &mut b,
        &reader,
        "tel still",
        Regime::Quiet,
        true,
        &writes,
        |b| {
            write(b, TEL_MASK, &SIX_FIELDS.to_le_bytes());
            for _ in 0..TEL_BURSTS {
                tel_burst(b);
            }
        },
    ));
    w.extend(window(
        &mut b,
        &reader,
        "tel stepping",
        Regime::Moving,
        true,
        &writes,
        |b| {
            driven(b, |b| {
                hold_at(b, mid);
                write(b, TEL_MASK, &SIX_FIELDS.to_le_bytes());
                for k in 0..TEL_BURSTS {
                    let goal = mid
                        + if k % 2 == 0 {
                            STEP_COUNTS
                        } else {
                            -STEP_COUNTS
                        };
                    write(b, GOAL_POSITION, &goal.to_le_bytes());
                    tel_burst(b);
                }
            })
        },
    ));

    let ping = build_ping(id);
    let read32 = build_read(id, 0, 32);
    let read240 = build_read(id, 0, 240);
    let goal = build_write(id, GOAL_POSITION, &mid.to_le_bytes());
    let polled: [(&'static str, Frame, &[u8]); 4] = [
        ("ping", Frame::Ping, &ping),
        ("read 32", Frame::Read32, &read32),
        ("read 240", Frame::Read240, &read240),
        ("write", Frame::Write, &goal),
    ];
    for (name, frame, wire) in polled {
        w.extend(window(
            &mut b,
            &reader,
            name,
            Regime::Quiet,
            false,
            &[(frame, fast)],
            |b| poll(b, wire, &mut failed, name),
        ));
    }
    let gread = build_instruction(
        BROADCAST,
        Opcode::Gread,
        0,
        &gread_uniform_payload(MODEL_NUMBER, 2, &[PHANTOM_ID, id]),
    );
    let slot = [(Frame::GreadSlot, fast)];
    w.extend(window(
        &mut b,
        &reader,
        "gread slot",
        Regime::Quiet,
        false,
        &slot,
        |b| {
            for _ in 0..GREADS {
                match b.xfer(&gread) {
                    Ok(ex) if ex.status.result == Some(ResultCode::PredecessorSilent) => {}
                    other => failed.push(format!("gread slot: {other:?}")),
                }
            }
        },
    ));
    let slow: [(&'static str, Frame, &[u8]); 3] = [
        ("ping 1M", Frame::Ping, &ping),
        ("read 32 1M", Frame::Read32, &read32),
        ("write 1M", Frame::Write, &goal),
    ];
    for (name, frame, wire) in slow {
        let frames = [(Frame::Write, fast), (frame, SLOW_BAUD)];
        w.extend(window(
            &mut b,
            &reader,
            name,
            Regime::Quiet,
            false,
            &frames,
            |b| {
                b.switch_baud(SLOW_BAUD);
                poll(b, wire, &mut failed, name)
            },
        ));
    }
    reboot(&mut b);

    let over = report(&w);
    assert!(failed.is_empty(), "failed exchanges: {failed:#?}");
    assert!(over.is_empty(), "over budget:\n{}", over.join("\n"));
}

/// Reboot (zeroing the probe), run `run`, read the probe; `None` when
/// `BUDGET_WINDOWS` leaves the window out.
fn window(
    b: &mut Bench,
    reader: &Reader,
    name: &'static str,
    regime: Regime,
    tel: bool,
    frames: &[(Frame, u32)],
    run: impl FnOnce(&mut Bench),
) -> Option<Window> {
    if let Ok(only) = env::var("BUDGET_WINDOWS")
        && !only.split(',').any(|w| w.trim() == name)
    {
        return None;
    }
    reboot(b);
    run(b);
    let snap = reader.read().expect("read the budget probe");
    running(b);
    Some(Window {
        name,
        regime,
        tel,
        frames: frames.to_vec(),
        snap,
    })
}

fn reboot(b: &mut Bench) {
    let id = b.id();
    b.status_ok(&build_reboot(id));
    sleep(Duration::from_millis(3 * REBOOT_SETTLE_MS));
    let home = b.home_baud();
    b.follow_baud(home);
}

/// A dump can leave the hart halted: `sample_tick` must move, resumed once
/// if it does not.
fn running(b: &mut Bench) {
    for attempt in 0..2 {
        if ticking(b) {
            return;
        }
        assert_eq!(attempt, 0, "the servo stays halted after a dump");
        probe::resume().expect("wlink resume");
        sleep(Duration::from_millis(REBOOT_SETTLE_MS));
    }
}

fn ticking(b: &mut Bench) -> bool {
    let id = b.id();
    let mut tick = || {
        let ex = b.xfer(&build_read(id, SAMPLE_TICK, 4)).ok()?;
        let p: [u8; 4] = ex.status.payload[..].try_into().ok()?;
        Some(u32::from_le_bytes(p))
    };
    let first = tick();
    sleep(Duration::from_millis(10));
    matches!((first, tick()), (Some(a), Some(c)) if a != c)
}

fn write(b: &mut Bench, addr: u16, data: &[u8]) {
    let id = b.id();
    b.status_ok(&build_write(id, addr, data));
}

fn hold_at(b: &mut Bench, goal: i32) {
    write(b, MODE, &[Mode::Position as u8]);
    write(b, GOAL_POSITION, &goal.to_le_bytes());
    write(b, TORQUE_ENABLE, &[1]);
}

/// Run a drive, then centre and drop torque whatever happened in it.
fn driven(b: &mut Bench, drive: impl FnOnce(&mut Bench)) {
    let r = catch_unwind(AssertUnwindSafe(|| drive(b)));
    let id = b.id();
    let mid = b.goal_mid();
    let _ = b.xfer(&build_write(id, GOAL_POSITION, &mid.to_le_bytes()));
    sleep(CENTRE_SETTLE);
    let _ = b.xfer(&build_write(id, TORQUE_ENABLE, &[0]));
    if let Err(e) = r {
        resume_unwind(e);
    }
}

/// One six-field burst; the host stays silent until it is over.
fn tel_burst(b: &mut Bench) {
    write(b, TEL_COUNT, &TEL_ROWS.to_le_bytes());
    sleep(TEL_BURST_WAIT);
    b.drain_stamps();
}

fn poll(b: &mut Bench, wire: &[u8], failed: &mut Vec<String>, name: &str) {
    match b.measure(wire, POLLS) {
        Ok(r) if r.fail == 0 => {}
        Ok(r) => failed.push(format!("{name}: {} of {POLLS}", r.fail)),
        Err(e) => failed.push(format!("{name}: {e}")),
    }
}

/// Print every window's components against their budgets; the lines over.
fn report(windows: &[Window]) -> Vec<String> {
    let mut over = Vec::new();
    for w in windows {
        let s = Scenario {
            regime: w.regime,
            tel: w.tel,
            frames: &w.frames,
        };
        let k = &w.snap.kernel;
        println!(
            "== {} ({:?}{}): kernel halts {}, bus halts {}, period max {:.1} us, hist {:?}",
            w.name,
            w.regime,
            if w.tel { ", TEL" } else { "" },
            k.halts,
            w.snap.bus.halts,
            us(k.period.max),
            k.hist
        );
        rows(k, &w.snap.bus, &s, |r| {
            if r.n == 0 && r.max == 0 {
                return;
            }
            println!("  {}", line(&r));
            if r.over() {
                over.push(format!("{}: {}", w.name, line(&r)));
            }
        });
    }
    over
}

fn line(r: &Row) -> String {
    let name = match r.component {
        Component::Tick(p) => format!("tick phase {p}"),
        Component::TelTick(p) => format!("TEL tick phase {p}"),
        Component::BankTick(p) => format!("bank tick phase {p}"),
        c => format!("{c:?}"),
    };
    let mut s = format!(
        "{name:<20} n {:>7}  max {:>5} ticks = {:>6.1} us / budget {:>5} = {:>6.1} us",
        r.n,
        r.max,
        us(r.max),
        r.budget,
        us(r.budget)
    );
    if r.mean_budget != 0 {
        s += &format!(
            "  mean {:>6.1} us / {:>6.1} us",
            us(r.mean),
            us(r.mean_budget)
        );
    }
    if r.over() {
        s += "  OVER";
    }
    s
}

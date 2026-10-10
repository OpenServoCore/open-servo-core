//! Kernel and bus budgets on silicon (`osc_servo_core::budget`): each
//! budgeted regime and traffic type runs in its own window, opened by a
//! reboot (the probe starts at zero) and read over the debug link. Fails
//! when any component's maximum, or a mean judged over enough bodies,
//! exceeds its budget, naming the window, the component, the measured
//! ticks and us, and the budget; or when a window's traffic went wrong.
//!
//! Needs the bench image (`--features bench`) flashed, `BENCH_ELF` naming
//! that ELF, and `wlink` on PATH with the probe attached; flashes nothing.
//! Opt-in: `cargo test --test hardware -- --ignored budgets --test-threads=1
//! --nocapture`; `BUDGET_WINDOWS` (comma-separated names, e.g. `tel still,tel
//! stepping`) runs only those windows; `BUDGET_OUT` names a directory that
//! receives each window's report and raw records as the window ends.
//! Drives the motor: holds and 600-count steps about the rail middle, each
//! drive ending centred with torque off; the last window ends rebooted.

use std::any::Any;
use std::env;
use std::fmt::Write as _;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

use bench::osc::{
    REBOOT_SETTLE_MS, build_instruction, build_ping, build_read, build_reboot, build_write,
    gread_uniform_payload,
};
use bench::probe::{self, Reader, Snapshot};
use osc_client::Outcome;
use osc_protocol::wire::{Opcode, ResultCode, STREAM_SAMPLES_MAX};
use osc_servo_core::Mode;
use osc_servo_core::budget::probe::{Component, Load, Row, Scenario, rows, us};
use osc_servo_core::budget::{Frame, Regime};
use osc_servo_core::regions::config::addr::common::MODEL_NUMBER;
use osc_servo_core::regions::control::addr::lifecycle::{
    GOAL_POSITION, MODE, TEL_MASK, TORQUE_ENABLE,
};
use osc_servo_core::regions::telemetry::addr::estimates::{SAMPLE_TICK, THETA_HAT_Q16};
use osc_servo_core::regions::telemetry::addr::health::TEL_DROP_COUNT;
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
/// A burst's wire time with its drops, and margin.
const TEL_WINDOW: Duration = Duration::from_millis(600);
const POLLS: u32 = 2000;
const GREADS: u32 = 200;
/// Top of the unicast range: never assigned, so slot 0 stays silent.
const PHANTOM_ID: u8 = 249;
const BROADCAST: u8 = 0xFE;
const SLOW_BAUD: u32 = 1_000_000;
/// `theta_hat_q16` holds pot counts in Q16.
const THETA_FRAC_BITS: u32 = 16;

struct Window {
    name: &'static str,
    regime: Regime,
    tel: bool,
    frames: Vec<(Frame, u32)>,
    snap: Snapshot,
    /// Instruction frames the host sent after the window's reboot.
    sent: u64,
    /// `tel_drop_count` and `stack_free_min` at the window's end.
    tel_drops: u16,
    stack_free_min: u16,
}

/// The windows run so far and what went wrong in them.
struct Fill {
    reader: Reader,
    out: Option<PathBuf>,
    failed: Vec<String>,
    over: Vec<String>,
}

#[serial]
#[test]
#[ignore = "needs the bench image: BENCH_ELF and wlink"]
fn kernel_and_bus_stay_inside_their_budgets() {
    let path = env::var("BENCH_ELF").expect("BENCH_ELF: the flashed bench image's ELF");
    let elf = std::fs::read(&path).expect("read BENCH_ELF");
    let out = env::var_os("BUDGET_OUT").map(PathBuf::from);
    if let Some(dir) = &out {
        std::fs::create_dir_all(dir).expect("create BUDGET_OUT");
    }
    let mut f = Fill {
        reader: Reader::from_elf(&elf).expect("budget probe symbols"),
        out,
        failed: Vec::new(),
        over: Vec::new(),
    };
    let mut b = bench();
    let fast = b.home_baud();
    let id = b.id();
    let mid = b.goal_mid();

    f.window(&mut b, "quiet", Regime::Quiet, false, &[], |_, _| {
        sleep(WINDOW)
    });
    let writes = [(Frame::Write, fast)];
    f.window(&mut b, "hold", Regime::Hold, false, &writes, |b, _| {
        driven(b, |b| {
            hold_at(b, mid);
            sleep(WINDOW);
        })
    });
    // The servo holds where the reboot left it: no approach move.
    f.window(
        &mut b,
        "hold still",
        Regime::Hold,
        false,
        &writes,
        |b, _| {
            let here = present(b);
            driven(b, |b| {
                hold_at(b, here);
                sleep(WINDOW);
            })
        },
    );
    f.window(
        &mut b,
        "stepping",
        Regime::Moving,
        false,
        &writes,
        |b, _| {
            driven(b, |b| {
                hold_at(b, mid);
                for k in 0..(STEP_WINDOW.as_millis() / STEP_HOLD.as_millis()) {
                    write(b, GOAL_POSITION, &step_goal(mid, k as u32).to_le_bytes());
                    sleep(STEP_HOLD);
                }
            })
        },
    );
    f.window(
        &mut b,
        "tel still",
        Regime::Quiet,
        true,
        &writes,
        |b, failed| {
            write(b, TEL_MASK, &SIX_FIELDS.to_le_bytes());
            for _ in 0..TEL_BURSTS {
                tel_burst(b, failed, "tel still");
            }
        },
    );
    f.window(
        &mut b,
        "tel stepping",
        Regime::Moving,
        true,
        &writes,
        |b, failed| {
            driven(b, |b| {
                hold_at(b, mid);
                write(b, TEL_MASK, &SIX_FIELDS.to_le_bytes());
                for k in 0..TEL_BURSTS {
                    write(b, GOAL_POSITION, &step_goal(mid, k).to_le_bytes());
                    tel_burst(b, failed, "tel stepping");
                }
            })
        },
    );

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
        f.window(
            &mut b,
            name,
            Regime::Quiet,
            false,
            &[(frame, fast)],
            |b, failed| poll(b, wire, failed, name),
        );
    }
    let gread = build_instruction(
        BROADCAST,
        Opcode::Gread,
        0,
        &gread_uniform_payload(MODEL_NUMBER, 2, &[PHANTOM_ID, id]),
    );
    let slot = [(Frame::GreadSlot, fast)];
    f.window(
        &mut b,
        "gread slot",
        Regime::Quiet,
        false,
        &slot,
        |b, failed| {
            for _ in 0..GREADS {
                match b.xfer(&gread) {
                    Ok(ex) if ex.status.result == Some(ResultCode::PredecessorSilent) => {}
                    other => failed.push(format!("gread slot: {other:?}")),
                }
            }
        },
    );
    let slow: [(&'static str, Frame, &[u8]); 3] = [
        ("ping 1M", Frame::Ping, &ping),
        ("read 32 1M", Frame::Read32, &read32),
        ("write 1M", Frame::Write, &goal),
    ];
    for (name, frame, wire) in slow {
        let frames = [(Frame::Write, fast), (frame, SLOW_BAUD)];
        f.window(&mut b, name, Regime::Quiet, false, &frames, |b, failed| {
            b.switch_baud(SLOW_BAUD);
            poll(b, wire, failed, name)
        });
    }
    reboot(&mut b);

    assert!(f.failed.is_empty(), "failed windows: {:#?}", f.failed);
    assert!(f.over.is_empty(), "over budget:\n{}", f.over.join("\n"));
}

impl Fill {
    /// Reboot (zeroing the probe), run `run`, read the probe and the health
    /// block, and report the window at once. A panic inside `run` fails the
    /// window and the run goes on to the next one. Skipped when
    /// `BUDGET_WINDOWS` leaves the window out.
    fn window(
        &mut self,
        b: &mut Bench,
        name: &'static str,
        regime: Regime,
        tel: bool,
        frames: &[(Frame, u32)],
        run: impl FnOnce(&mut Bench, &mut Vec<String>),
    ) {
        if let Ok(only) = env::var("BUDGET_WINDOWS")
            && !only.split(',').any(|w| w.trim() == name)
        {
            return;
        }
        reboot(b);
        let from = b.frames_sent();
        if let Err(e) = catch_unwind(AssertUnwindSafe(|| run(b, &mut self.failed))) {
            self.failed.push(format!("{name}: {}", panic_text(&*e)));
        }
        let sent = b.frames_sent() - from;
        let snap = self.reader.read().expect("read the budget probe");
        running(b);
        let [d0, d1, s0, s1] = read_bytes::<4>(b, TEL_DROP_COUNT);
        let w = Window {
            name,
            regime,
            tel,
            frames: frames.to_vec(),
            snap,
            sent,
            tel_drops: u16::from_le_bytes([d0, d1]),
            stack_free_min: u16::from_le_bytes([s0, s1]),
        };
        if u64::from(w.snap.bus.host.n) != w.sent {
            self.failed.push(format!(
                "{name}: {} host frames booked, {} sent",
                w.snap.bus.host.n, w.sent
            ));
        }
        self.report(&w);
    }

    /// Print the window's components against their budgets and its raw
    /// records, save them under `BUDGET_OUT`, and keep the lines over.
    fn report(&mut self, w: &Window) {
        let s = Scenario {
            regime: w.regime,
            tel: w.tel,
            frames: &w.frames,
        };
        let (k, bus) = (&w.snap.kernel, &w.snap.bus);
        let mut text = String::new();
        let _ = writeln!(
            text,
            "== {} ({:?}{}): kernel halts {}, bus halts {}, frames sent {}, \
             tel_drop {}, stack_free_min {} B",
            w.name,
            w.regime,
            if w.tel { ", TEL" } else { "" },
            k.halts,
            bus.halts,
            w.sent,
            w.tel_drops,
            w.stack_free_min
        );
        let _ = writeln!(text, "  {}", load_line("period", &k.period));
        let _ = writeln!(text, "  hist {:?}", k.hist);
        rows(k, bus, &s, |r| {
            if r.n == 0 && r.max == 0 {
                return;
            }
            let l = line(&r);
            let _ = writeln!(text, "  {l}");
            if r.over() {
                self.over.push(format!("{}: {l}", w.name));
            }
        });
        let _ = writeln!(text, "  kernel words {:08x?}", w.snap.kernel_words);
        let _ = writeln!(text, "  bus words {:08x?}", w.snap.bus_words);
        print!("{text}");
        if let Some(dir) = &self.out {
            let file = dir.join(format!("{}.txt", w.name.replace(' ', "-")));
            if let Err(e) = std::fs::write(&file, &text) {
                println!("  (not saved to {}: {e})", file.display());
            }
        }
    }
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

fn read_bytes<const N: usize>(b: &mut Bench, addr: u16) -> [u8; N] {
    let id = b.id();
    let st = b.status_ok(&build_read(id, addr, N as u16));
    st.payload[..]
        .try_into()
        .unwrap_or_else(|_| panic!("read of {addr:#06x}: {N}-byte payload"))
}

/// The goal that holds the shaft where it is.
fn present(b: &mut Bench) -> i32 {
    let theta = i32::from_le_bytes(read_bytes::<4>(b, THETA_HAT_Q16));
    b.goal_clamp(theta >> THETA_FRAC_BITS)
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

/// Step `k` of a square wave about `mid`.
fn step_goal(mid: i32, k: u32) -> i32 {
    if k.is_multiple_of(2) {
        mid + STEP_COUNTS
    } else {
        mid - STEP_COUNTS
    }
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

/// One six-field burst, whole: acked, every frame in, before the host
/// speaks again.
fn tel_burst(b: &mut Bench, failed: &mut Vec<String>, name: &str) {
    let frames = usize::from(TEL_ROWS) / STREAM_SAMPLES_MAX;
    match b.tel_stream(TEL_ROWS, TEL_WINDOW) {
        Ok(r) => {
            let ack = r.ack.as_ref().and_then(|a| a.result);
            if ack != Some(ResultCode::Ok)
                || r.outcome != Outcome::Complete
                || r.frames.len() != frames
            {
                failed.push(format!(
                    "{name}: TEL burst ack {ack:?}, {:?}, {} of {frames} frames, {} laps",
                    r.outcome,
                    r.frames.len(),
                    r.laps
                ));
            }
        }
        Err(e) => failed.push(format!("{name}: TEL burst: {e}")),
    }
}

fn poll(b: &mut Bench, wire: &[u8], failed: &mut Vec<String>, name: &str) {
    match b.measure(wire, POLLS) {
        Ok(r) if r.fail == 0 => {}
        Ok(r) => failed.push(format!("{name}: {} of {POLLS}", r.fail)),
        Err(e) => failed.push(format!("{name}: {e}")),
    }
}

fn panic_text(e: &(dyn Any + Send)) -> String {
    e.downcast_ref::<String>()
        .cloned()
        .or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()))
        .unwrap_or_else(|| "panic".into())
}

/// `t` ticks as ticks and us.
fn ticks(t: u32) -> String {
    format!("{t:>5} ticks = {:>6.1} us", us(t))
}

fn load_line(name: &str, l: &Load) -> String {
    format!(
        "{name:<20} n {:>7}  max {}  mean {}",
        l.n,
        ticks(l.max),
        ticks(l.mean())
    )
}

fn line(r: &Row) -> String {
    let name = match r.component {
        Component::Tick(p) => format!("tick phase {p}"),
        Component::TelTick(p) => format!("TEL tick phase {p}"),
        Component::BankTick(p) => format!("bank tick phase {p}"),
        c => format!("{c:?}"),
    };
    let mut s = format!(
        "{name:<20} n {:>7}  max {} / budget {}",
        r.n,
        ticks(r.max),
        ticks(r.budget)
    );
    if r.n > 0 {
        s += &format!("  mean {}", ticks(r.mean));
        if r.mean_budget != 0 {
            s += &format!(" / {}", ticks(r.mean_budget));
        }
    }
    if r.over() {
        s += "  OVER";
    }
    s
}

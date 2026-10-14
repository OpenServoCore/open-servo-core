//! Kernel and bus budgets on silicon (`osc_servo_core::budget`): each
//! budgeted regime and traffic type runs in its own window, opened by a
//! reboot (the probe starts at zero) and read over the debug link. Fails
//! when any component's maximum, or a mean judged over enough bodies,
//! exceeds its budget, naming the window, the component, the measured
//! ticks and us, and the budget; or when a window's traffic went wrong; or
//! when its health counters, read over the bus after the dumps, show a CRC
//! fail, a framing drop, a TEL drop or lost ticks beyond the dumps' own.
//!
//! Needs the bench image (`--features bench`) flashed, `BENCH_ELF` naming
//! that ELF, and `wlink` on PATH with the probe attached; flashes nothing.
//! Opt-in: `cargo test --test hardware -- --ignored budgets --test-threads=1
//! --nocapture`; `BUDGET_WINDOWS` (comma-separated names, e.g. `tel still,tel
//! stepping`) runs only those windows; `BUDGET_OUT` names a directory that
//! receives each window's report and raw records as the window ends.
//! Drives the motor: holds and 600-count steps about the rail middle, each
//! drive ending centred with torque off; the last window ends rebooted.
//! `seated` runs only when `BUDGET_WINDOWS` names it: it needs the horn
//! clamped at mid travel, and holds a goal past the clamp so the
//! thermometer tracks a seat.
//! Kernel ticks book by drive arm. A window image (`--features
//! bench-window`, `scripts/build-image.sh --window`) also prints gamma(k),
//! the long ticks by arm and the entry lateness; the standard bench image
//! skips those lines and says so. A bus body reports its net
//! time (the kernel vectors inside it subtracted), which the table judges,
//! and for its longest one the wall time and the kernel vectors nested.
//! A report-only window is never held to the table.
//! A telcost image (`--features bench-telcost`, no bus record) reports the
//! TEL windows' cost per tick and per TEL step instead, held only to its
//! own self-check; its windows run no budget, bus or host-frame check. It
//! stamps every other burst: the unstamped ones give the cost per tick and
//! the time inside the TEL hooks, the stamped ones the steps. The
//! self-check holds the steps inside the hooks and the hooks inside the
//! tick; the glue between them is measured, not bounded. A ping ahead of
//! the read hands the bus the TX lane, so no TEL tick lands between the
//! records' dumps.

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
use osc_protocol::table::{CRC_FAIL_COUNT, FRAMING_DROP_COUNT};
use osc_protocol::tel::{Chunk, Chunks, StreamFrame};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::Mode;
use osc_servo_core::budget::probe::{
    Component, LONG_TICKS, Row, Scenario, TEL_PER_TICK_NOISE, TelReport, TelStep, WindowProbe,
    rows, us,
};
use osc_servo_core::budget::{Frame, Regime, TICKS_PER_US, WINDOW_TICKS};
use osc_servo_core::estimator::thermal::flag;
use osc_servo_core::kernel::HEALTH_LOST_MAX;
use osc_servo_core::regions::config::addr::common::MODEL_NUMBER;
use osc_servo_core::regions::control::addr::lifecycle::{GOAL_POSITION, MODE, TORQUE_ENABLE};
use osc_servo_core::regions::control::addr::tel::{CHUNK0_ADDR, TEL_COUNT};
use osc_servo_core::regions::telemetry::addr::estimates::THETA_HAT_Q16;
use osc_servo_core::regions::telemetry::addr::tick::{CURRENT_RAW, POS, SAMPLE_TICK};
use osc_servo_core::regions::telemetry_ext::addr::health::{
    STACK_FREE_MIN, TEL_DROP_COUNT, TICK_LOST_COUNT, TICK_OVER_COUNT,
};
use osc_servo_core::regions::telemetry_ext::addr::therm::THERM_FLAGS;
use serial_test::serial;

use crate::support::{Bench, bench};

const STEP_COUNTS: i32 = 600;
const STEP_HOLD: Duration = Duration::from_millis(400);
const WINDOW: Duration = Duration::from_secs(10);
const STEP_WINDOW: Duration = Duration::from_secs(20);
const CENTRE_SETTLE: Duration = Duration::from_millis(500);
/// The bench soaks' six raw fields, `pos` and
/// `current_raw..=current_trough`: 12 B rows.
const SIX_FIELDS: Chunks = Chunks::two(Chunk::new(POS, 2), Chunk::new(CURRENT_RAW, 10));
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
/// The seated window's goal past the clamp, and the most the clamped shaft
/// may move.
const SEAT_PUSH: i32 = 40;
const SEAT_MOVE_MAX: i32 = 10;
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum Gate {
    Budget,
    ReportOnly,
}

struct Window {
    name: &'static str,
    gate: Gate,
    tel: bool,
    frames: Vec<(Frame, u32)>,
    snap: Snapshot,
    /// Instruction frames the host sent after the window's reboot.
    sent: u64,
    counters: Counters,
    stack_free_min: u16,
}

/// The health counters since the window's reboot, read over the bus after
/// the probe's dumps so the bus record never books them.
struct Counters {
    crc_fail: u32,
    framing_drop: u32,
    tick_over: u16,
    tick_lost: u16,
    tel_drop: u16,
}

impl std::fmt::Display for Counters {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "crc_fail {}, framing_drop {}, tick_over {}, tick_lost {}, tel_drop {}",
            self.crc_fail, self.framing_drop, self.tick_over, self.tick_lost, self.tel_drop
        )
    }
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

    f.window(&mut b, "quiet", Gate::Budget, false, &[], |_, _| {
        sleep(WINDOW)
    });
    let writes = [(Frame::Write, fast)];
    f.window(&mut b, "hold", Gate::Budget, false, &writes, |b, _| {
        driven(b, |b| {
            hold_at(b, mid);
            sleep(WINDOW);
        })
    });
    // The servo holds where the reboot left it: no approach move.
    f.window(
        &mut b,
        "hold still",
        Gate::Budget,
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
    // The known hold twitch: a goal from raw `pos`, off the observer's
    // units, can leave the hold cycling Closed and parked.
    f.window(
        &mut b,
        "hold raw pos",
        Gate::ReportOnly,
        false,
        &writes,
        |b, _| {
            let raw = u16::from_le_bytes(read_bytes::<2>(b, POS));
            let goal = b.goal_clamp(i32::from(raw));
            driven(b, |b| {
                hold_at(b, goal);
                sleep(WINDOW);
            })
        },
    );
    f.window(&mut b, "stepping", Gate::Budget, false, &writes, |b, _| {
        driven(b, |b| {
            hold_at(b, mid);
            for k in 0..(STEP_WINDOW.as_millis() / STEP_HOLD.as_millis()) {
                write(b, GOAL_POSITION, &step_goal(mid, k as u32).to_le_bytes());
                sleep(STEP_HOLD);
            }
        })
    });
    f.window(
        &mut b,
        "tel still",
        Gate::Budget,
        true,
        &writes,
        |b, failed| {
            for _ in 0..TEL_BURSTS {
                tel_burst(b, failed, "tel still");
            }
        },
    );
    f.window(
        &mut b,
        "tel stepping",
        Gate::Budget,
        true,
        &writes,
        |b, failed| {
            driven(b, |b| {
                hold_at(b, mid);
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
    // The TEL arm block with count 0 (nothing streams) and the count alone:
    // the chunks validate as plain bytes.
    let mut block = SIX_FIELDS.to_le_bytes().to_vec();
    block.extend_from_slice(&0u16.to_le_bytes());
    let arm10 = build_write(id, CHUNK0_ADDR, &block);
    let arm2 = build_write(id, TEL_COUNT, &0u16.to_le_bytes());
    let polled: [(&'static str, Frame, &[u8]); 6] = [
        ("ping", Frame::Ping, &ping),
        ("read 32", Frame::Read32, &read32),
        ("read 240", Frame::Read240, &read240),
        ("write", Frame::Write, &goal),
        ("arm10", Frame::Write, &arm10),
        ("arm2", Frame::Write, &arm2),
    ];
    for (name, frame, wire) in polled {
        f.window(
            &mut b,
            name,
            Gate::Budget,
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
        Gate::Budget,
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
        f.window(&mut b, name, Gate::Budget, false, &frames, |b, failed| {
            b.switch_baud(SLOW_BAUD);
            poll(b, wire, failed, name)
        });
    }
    if named("seated") {
        f.window(&mut b, "seated", Gate::Budget, false, &writes, seated);
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
        gate: Gate,
        tel: bool,
        frames: &[(Frame, u32)],
        run: impl FnOnce(&mut Bench, &mut Vec<String>),
    ) {
        if env::var_os("BUDGET_WINDOWS").is_some() && !named(name) {
            return;
        }
        reboot(b);
        let from = b.frames_sent();
        if let Err(e) = catch_unwind(AssertUnwindSafe(|| run(b, &mut self.failed))) {
            self.failed.push(format!("{name}: {}", panic_text(&*e)));
        }
        let sent = b.frames_sent() - from;
        if self.reader.telcost() {
            let ping = build_ping(b.id());
            if let Err(e) = b.xfer(&ping) {
                self.failed.push(format!("{name}: lane handback ping: {e}"));
            }
        }
        let snap = self.reader.read().expect("read the budget probe");
        running(b);
        let counters = counters(b);
        // tick_over is reported, not judged: a debug halt inside the tick
        // interrupt counts one, and the kernel rows already judge every tick
        // body. Each dump's halt counts exactly HEALTH_LOST_MAX lost ticks.
        let lost = u32::from(HEALTH_LOST_MAX) * u32::from(snap.dumps);
        if counters.crc_fail != 0
            || counters.framing_drop != 0
            || counters.tel_drop != 0
            || u32::from(counters.tick_lost) != lost
        {
            self.failed.push(format!(
                "{name}: {counters}; want crc_fail, framing_drop and tel_drop 0, \
                 tick_lost {lost} ({} dumps x {HEALTH_LOST_MAX})",
                snap.dumps
            ));
        }
        let w = Window {
            name,
            gate,
            tel,
            frames: frames.to_vec(),
            snap,
            sent,
            counters,
            stack_free_min: u16::from_le_bytes(read_bytes(b, STACK_FREE_MIN)),
        };
        if let Some(bus) = &w.snap.bus
            && u64::from(bus.host.n) != w.sent
        {
            self.failed.push(format!(
                "{name}: {} host frames booked, {} sent",
                bus.host.n, w.sent
            ));
        }
        if w.snap.tel.is_some() {
            self.report_tel(&w);
        } else {
            self.report(&w);
        }
    }

    /// Print the window's components against their budgets and its raw
    /// records, save them under `BUDGET_OUT`, and keep the lines over.
    fn report(&mut self, w: &Window) {
        let s = Scenario {
            tel: w.tel,
            frames: &w.frames,
        };
        let k = &w.snap.kernel;
        let Some(bus) = &w.snap.bus else {
            return;
        };
        let mut text = String::new();
        let _ = writeln!(
            text,
            "== {} ({:?}{}): kernel halts {}, bus halts {}, frames sent {}, \
             {}, stack_free_min {} B",
            w.name,
            w.gate,
            if w.tel { ", TEL" } else { "" },
            k.halts,
            bus.halts,
            w.sent,
            w.counters,
            w.stack_free_min
        );
        window_lines(&mut text, w.snap.windows.as_ref());
        rows(k, bus, &s, |r| {
            if r.n == 0 && r.max == 0 {
                return;
            }
            let l = line(&r);
            let _ = writeln!(text, "  {l}");
            if r.over() && w.gate == Gate::Budget {
                self.over.push(format!("{}: {l}", w.name));
            }
        });
        let _ = writeln!(text, "  kernel words {:08x?}", w.snap.kernel_words);
        let _ = writeln!(text, "  bus words {:08x?}", w.snap.bus_words);
        if w.snap.windows.is_some() {
            let _ = writeln!(text, "  window words {:08x?}", w.snap.window_words);
        }
        self.emit(w, &text);
    }

    /// A telcost window: the TEL steps, the TEL ticks over the plain ticks
    /// of their phase, the mean TEL cost a tick and its self-check.
    fn report_tel(&mut self, w: &Window) {
        let (k, Some(tel)) = (&w.snap.kernel, &w.snap.tel) else {
            return;
        };
        let (t, c) = (&tel.tally, &tel.cost);
        let mut text = String::new();
        let _ = writeln!(
            text,
            "== {} (telcost{}): kernel halts {}, frames sent {}, {}, \
             stack_free_min {} B\n  budget, bus and host-frame checks: skipped (telcost image)",
            w.name,
            if w.tel { ", TEL" } else { "" },
            k.halts,
            w.sent,
            w.counters,
            w.stack_free_min
        );
        let _ = writeln!(text, "  stamp pair {}", ticks(c.cal.into()));
        for s in TelStep::ALL {
            let l = c.load(s);
            let _ = writeln!(
                text,
                "  TEL step {:<8} n {:>7}  mean {}  max {}",
                format!("{s:?}"),
                l.n,
                ticks(l.mean()),
                ticks(l.max)
            );
        }
        for r in Regime::ALL {
            let first = t.first[r as usize];
            if let Some(m) = TelReport::tel_row(k, t, r) {
                let _ = writeln!(text, "  {r:?} TEL row, phase-matched  mean {}", fticks(m));
            }
            if first.n != 0 {
                let _ = writeln!(
                    text,
                    "  {r:?} first TEL tick     n {:>7}  mean {}  max {}",
                    first.n,
                    ticks(first.mean()),
                    ticks(first.max)
                );
            }
        }
        let rep = TelReport::of(k, t);
        let _ = writeln!(
            text,
            "  TEL ticks n {} over plain {}, bank ticks n {} over plain {}",
            rep.tel_n,
            opt_ticks(rep.tel),
            rep.bank_n,
            opt_ticks(rep.bank)
        );
        let _ = writeln!(
            text,
            "  TEL per tick {} unstamped, steps {} over {} stamped ticks (bank 1/{})",
            opt_ticks(rep.per_tick),
            opt_ticks(rep.steps),
            rep.steps_n,
            osc_servo_core::tel::STREAM_SAMPLES_MAX
        );
        let _ = writeln!(
            text,
            "  TEL hooks per tick {}: glue {} unstamped, outside the hooks {}",
            opt_ticks(rep.hooks),
            opt_ticks(rep.glue()),
            opt_ticks(rep.outside())
        );
        if w.tel {
            let pass = rep.check() == Some(true);
            let l = format!(
                "self-check steps {} <= hooks {} <= TEL per tick {} + {TEL_PER_TICK_NOISE}",
                opt_ticks(rep.steps),
                opt_ticks(rep.hooks),
                opt_ticks(rep.per_tick)
            );
            let _ = writeln!(text, "  {l}: {}", if pass { "PASS" } else { "FAIL" });
            if !pass {
                self.failed.push(format!("{}: {l}", w.name));
            }
        }
        window_lines(&mut text, w.snap.windows.as_ref());
        let _ = writeln!(text, "  kernel words {:08x?}", w.snap.kernel_words);
        let _ = writeln!(text, "  TEL tally words {:08x?}", tel.tally_words);
        let _ = writeln!(text, "  TEL step words {:08x?}", tel.cost_words);
        self.emit(w, &text);
    }

    /// Print a window's report and save it under `BUDGET_OUT`.
    fn emit(&self, w: &Window, text: &str) {
        print!("{text}");
        if let Some(dir) = &self.out {
            let file = dir.join(format!("{}.txt", w.name.replace(' ', "-")));
            if let Err(e) = std::fs::write(&file, text) {
                println!("  (not saved to {}: {e})", file.display());
            }
        }
    }
}

/// gamma(k), the long ticks by arm and the entry lateness, from a window
/// image's record.
fn window_lines(text: &mut String, w: Option<&WindowProbe>) {
    let Some(w) = w else {
        let _ = writeln!(
            text,
            "  gamma(k), long ticks, entry lateness: skipped (no window record; \
             build with scripts/build-image.sh --window)"
        );
        return;
    };
    for (kt, g) in WINDOW_TICKS.into_iter().zip(w.windows.gamma) {
        let _ = writeln!(text, "  gamma({kt:>2})            {}", ticks(g));
    }
    let [open, closed] = w.long;
    let share = 100.0 * closed as f32 / (open + closed).max(1) as f32;
    let _ = writeln!(
        text,
        "  ticks >= {:.0} us: Open {open}, Closed {closed} ({share:.1}% Closed)",
        us(LONG_TICKS)
    );
    let _ = writeln!(text, "  entry lateness max  {}", ticks(w.late_max));
}

/// `BUDGET_WINDOWS` names the window.
fn named(name: &str) -> bool {
    env::var("BUDGET_WINDOWS").is_ok_and(|only| only.split(',').any(|w| w.trim() == name))
}

/// Hold a goal past the clamp: the drive pushes at the current limit on a
/// still shaft, which the thermometer gates as a seat and tracks once its
/// reference window closes. Fails when the thermometer has no model, the
/// shaft moved (no clamp), or no seat was tracked.
fn seated(b: &mut Bench, failed: &mut Vec<String>) {
    let flags = |b: &mut Bench| read_bytes::<1>(b, THERM_FLAGS)[0];
    if flags(b) & flag::UNSET != 0 {
        failed.push("seated: the thermometer has no model".into());
        return;
    }
    let here = present(b);
    driven(b, |b| {
        hold_at(b, here + SEAT_PUSH);
        sleep(WINDOW);
        let moved = present(b) - here;
        if moved.abs() > SEAT_MOVE_MAX {
            failed.push(format!("seated: the shaft moved {moved} counts"));
        }
        if flags(b) & flag::TRACK == 0 {
            failed.push("seated: no seat tracked".into());
        }
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

fn counters(b: &mut Bench) -> Counters {
    Counters {
        crc_fail: u32::from_le_bytes(read_bytes(b, CRC_FAIL_COUNT)),
        framing_drop: u32::from_le_bytes(read_bytes(b, FRAMING_DROP_COUNT)),
        tick_over: u16::from_le_bytes(read_bytes(b, TICK_OVER_COUNT)),
        tick_lost: u16::from_le_bytes(read_bytes(b, TICK_LOST_COUNT)),
        tel_drop: u16::from_le_bytes(read_bytes(b, TEL_DROP_COUNT)),
    }
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
    b.require_forward_drive();
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
    match b.tel_stream(SIX_FIELDS, TEL_ROWS, TEL_WINDOW) {
        Ok(r) => {
            let ack = r.ack.as_ref().and_then(|a| a.result);
            let rows: usize = r
                .frames
                .iter()
                .filter_map(|f| StreamFrame::parse(SIX_FIELDS.row_bytes(), &f.payload))
                .map(|f| f.len())
                .sum();
            if ack != Some(ResultCode::Ok)
                || r.outcome != Outcome::Complete
                || rows != usize::from(TEL_ROWS)
            {
                failed.push(format!(
                    "{name}: TEL burst ack {ack:?}, {:?}, {rows} of {TEL_ROWS} rows in {} frames, {} laps",
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

/// A mean `t` ticks as ticks and us.
fn fticks(t: f32) -> String {
    format!("{t:>7.1} ticks = {:>6.2} us", t / TICKS_PER_US as f32)
}

fn opt_ticks(t: Option<f32>) -> String {
    t.map_or_else(
        || "none (no tick, or a TEL phase with no plain tick)".into(),
        fticks,
    )
}

fn line(r: &Row) -> String {
    let name = match r.component {
        Component::Tick(r, p) => format!("{r:?} tick phase {p}"),
        Component::TelTick(r, p) => format!("{r:?} TEL tick phase {p}"),
        Component::BankTick(r, p) => format!("{r:?} bank tick phase {p}"),
        Component::Health(r, p) => format!("{r:?} close tick phase {p}"),
        c => format!("{c:?}"),
    };
    let mut s = format!(
        "{name:<25} n {:>7}  max {} / budget {}",
        r.n,
        ticks(r.max),
        ticks(r.budget)
    );
    if r.wall != 0 {
        s += &format!("  wall {}  nested {}", ticks(r.wall), r.nested);
    }
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

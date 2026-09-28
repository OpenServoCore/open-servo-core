//! The kernel's control trace pinned against a golden recording. One
//! deterministic script drives the integer plant through torque-off rest, a
//! Position hold and step, Velocity both ways, Current both ways, OpenLoop
//! both ways plus a braked zero, an over-current latch and its ack. Every
//! fast tick folds the motor command and the raw TELEMETRY region into a
//! CRC; every medium tick lands a decoded row of the published estimates
//! beside that CRC. A change to anything the kernel computes fails at the
//! first diverging row with both rows in the message, so a band that must
//! leave control bit-identical proves it here without touching the golden.
//!
//! Re-record only for an intended control change:
//! `KERNEL_TRACE_RECORD=1 cargo test -p osc-integration --test kernel_trace`

use std::collections::BTreeSet;

use control_table::RegisterFile;
use osc_integration::plant::{BIAS, Plant, duty_of, kernel, last_cmd, lut_live, seed};
use osc_protocol::crc::osc_crc_continue;
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::kernel::faults::{BIT_OVER_CURRENT, CODE_NONE, CODE_OVER_CURRENT};
use osc_servo_core::pot_lut::{GRID_SHIFT, KNOTS, interp_q4};
use osc_servo_core::regions::{TELEMETRY_BASE_ADDR, TELEMETRY_REGION_SIZE};
use osc_servo_core::{ControlTable, DecayMode, Mode, MotorCmd, RegionStorage, Shared};

mod support;

const GOLDEN: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/kernel_trace.golden");
const RECORD_VAR: &str = "KERNEL_TRACE_RECORD";

const TICKS: u32 = 24_500;
const START_POS: u16 = 1500;

const CMD_DISABLED: u8 = 0;
const CMD_COAST: u8 = 1;
const CMD_BRAKE: u8 = 2;
const CMD_DRIVE_SLOW: u8 = 3;
const CMD_DRIVE_FAST: u8 = 4;

fn cmd_code(cmd: MotorCmd) -> u8 {
    match cmd {
        MotorCmd::Disabled => CMD_DISABLED,
        MotorCmd::Coast => CMD_COAST,
        MotorCmd::Brake => CMD_BRAKE,
        MotorCmd::Drive {
            decay: DecayMode::Slow,
            ..
        } => CMD_DRIVE_SLOW,
        MotorCmd::Drive {
            decay: DecayMode::Fast,
            ..
        } => CMD_DRIVE_FAST,
    }
}

type Script = fn(u32, &Shared, &mut Plant);

/// The golden script, keyed by fast tick. Torque comes on at rest with
/// the goal already at the pot, so the hold parks; the step, the
/// closed-loop modes and the open-loop sweep follow; a shorted winding
/// latches over-current, the torque edge acks it.
fn script(t: u32, sh: &Shared, plant: &mut Plant) {
    let set = |f: fn(&mut ControlTable)| sh.table.with_mut(f);
    match t {
        2_000 => set(|t| {
            t.control.lifecycle.goal_position = START_POS as i32;
            t.control.lifecycle.torque_enable = true;
        }),
        4_000 => set(|t| t.control.lifecycle.goal_position = START_POS as i32 + 400),
        12_000 => set(|t| {
            t.control.lifecycle.mode = Mode::Velocity;
            t.control.lifecycle.goal_velocity = 800;
        }),
        15_000 => set(|t| t.control.lifecycle.goal_velocity = -800),
        18_000 => set(|t| {
            t.control.lifecycle.mode = Mode::Current;
            t.control.lifecycle.goal_current = 150;
        }),
        18_800 => set(|t| t.control.lifecycle.goal_current = -150),
        19_600 => set(|t| {
            t.control.lifecycle.mode = Mode::OpenLoop;
            t.control.lifecycle.goal_duty = 5000;
        }),
        20_600 => set(|t| t.control.lifecycle.goal_duty = -5000),
        21_600 => set(|t| t.control.lifecycle.goal_duty = 0),
        22_000 => {
            set(|t| t.control.lifecycle.goal_duty = 8000);
            plant.current_offset = 3000;
        }
        22_200 => plant.current_offset = 0,
        23_000 => set(|t| t.control.lifecycle.torque_enable = false),
        23_200 => set(|t| t.control.lifecycle.torque_enable = true),
        24_000 => set(|t| t.control.lifecycle.torque_enable = false),
        _ => {}
    }
}

/// One medium tick of the trace: the CRC of the fast ticks since the
/// previous row, then the state the kernel published at this one.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
struct Row {
    fast_crc: u16,
    cmd: u8,
    mode_active: u8,
    fault_code: u8,
    omega_hat_src: u8,
    fault_flags: u8,
    pos: u16,
    current_bias_counts: u16,
    theta_hat_q16: i32,
    omega_hat_cps: i32,
    tau_d_counts: i16,
    i_lim_counts: u16,
    t_winding_cc: i16,
    vbus_counts: u16,
    duty_applied_q15: i16,
    omega_bemf_cps: i16,
    r_hat_q12: u16,
    i_hat_counts: i16,
}

const ROW_LEN: usize = 36;

impl Row {
    fn publish(fast_crc: u16, cmd: u8, t: &ControlTable) -> Self {
        let e = &t.telemetry.estimates;
        Self {
            fast_crc,
            cmd,
            mode_active: t.telemetry.mode.mode_active,
            fault_code: t.telemetry.mode.fault_code,
            omega_hat_src: t.telemetry.mode.omega_hat_src,
            fault_flags: t.telemetry.common.fault_flags,
            pos: t.telemetry.sensors.pos,
            current_bias_counts: t.telemetry.sensors.current_bias_counts,
            theta_hat_q16: e.theta_hat_q16,
            omega_hat_cps: e.omega_hat_cps,
            tau_d_counts: e.tau_d_counts,
            i_lim_counts: e.i_lim_counts,
            t_winding_cc: e.t_winding_cc,
            vbus_counts: e.vbus_counts,
            duty_applied_q15: e.duty_applied_q15,
            omega_bemf_cps: e.omega_bemf_cps,
            r_hat_q12: e.r_hat_q12,
            i_hat_counts: e.i_hat_counts,
        }
    }

    fn encode(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.fast_crc.to_le_bytes());
        out.extend_from_slice(&[
            self.cmd,
            self.mode_active,
            self.fault_code,
            self.omega_hat_src,
            self.fault_flags,
            0,
        ]);
        out.extend_from_slice(&self.pos.to_le_bytes());
        out.extend_from_slice(&self.current_bias_counts.to_le_bytes());
        out.extend_from_slice(&self.theta_hat_q16.to_le_bytes());
        out.extend_from_slice(&self.omega_hat_cps.to_le_bytes());
        out.extend_from_slice(&self.tau_d_counts.to_le_bytes());
        out.extend_from_slice(&self.i_lim_counts.to_le_bytes());
        out.extend_from_slice(&self.t_winding_cc.to_le_bytes());
        out.extend_from_slice(&self.vbus_counts.to_le_bytes());
        out.extend_from_slice(&self.duty_applied_q15.to_le_bytes());
        out.extend_from_slice(&self.omega_bemf_cps.to_le_bytes());
        out.extend_from_slice(&self.r_hat_q12.to_le_bytes());
        out.extend_from_slice(&self.i_hat_counts.to_le_bytes());
    }

    fn decode(b: &[u8; ROW_LEN]) -> Self {
        let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let i16_at = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        let i32_at = |i: usize| i32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        Self {
            fast_crc: u16_at(0),
            cmd: b[2],
            mode_active: b[3],
            fault_code: b[4],
            omega_hat_src: b[5],
            fault_flags: b[6],
            pos: u16_at(8),
            current_bias_counts: u16_at(10),
            theta_hat_q16: i32_at(12),
            omega_hat_cps: i32_at(16),
            tau_d_counts: i16_at(20),
            i_lim_counts: u16_at(22),
            t_winding_cc: i16_at(24),
            vbus_counts: u16_at(26),
            duty_applied_q15: i16_at(28),
            omega_bemf_cps: i16_at(30),
            r_hat_q12: u16_at(32),
            i_hat_counts: i16_at(34),
        }
    }
}

/// `script` against the rig for `ticks`, with `lut` LIVE in the kernel's
/// array (the plant itself stays linear: the table only re-maps what the
/// kernel believes the pot reads).
fn run(lut: Option<&[i16; KNOTS]>, ticks: u32, script: Script) -> Vec<Row> {
    let sh = Shared::new();
    seed(&sh);
    if let Some(k) = lut {
        lut_live(&sh, k);
    }
    let mut k = kernel();
    let mut plant = Plant::new(START_POS);
    let mut duty = 0i16;
    let mut rng: u32 = 0xdead_beef;
    let mut crc = 0u16;
    let mut rows = Vec::with_capacity(ticks as usize / DECIM_MED as usize + 1);
    for t in 0..ticks {
        script(t, &sh, &mut plant);
        let mut f = plant.step(duty);
        // +-3 counts of pot noise so the observer corrections are never
        // trivially zero, even parked
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let n = ((rng >> 24) as i32 % 7) - 3;
        f.pos = (f.pos as i32 + n).clamp(0, 4095) as u16;
        k.on_tick(f, &sh);
        let cmd = last_cmd(&k);
        duty = duty_of(cmd);
        let code = cmd_code(cmd);
        crc = osc_crc_continue(crc, &[code]);
        crc = osc_crc_continue(crc, &duty.to_le_bytes());
        let telemetry = RegisterFile::read(&sh.table, TELEMETRY_BASE_ADDR, TELEMETRY_REGION_SIZE)
            .expect("whole TELEMETRY region");
        crc = osc_crc_continue(crc, telemetry);
        if t % DECIM_MED as u32 == 0 {
            rows.push(sh.table.with(|t| Row::publish(crc, code, t)));
            crc = 0;
        }
    }
    rows
}

fn row_at(rows: &[Row], tick: u32) -> Row {
    rows[tick as usize / DECIM_MED as usize]
}

/// The script reached every phase it claims to.
fn assert_script_phases(rows: &[Row]) {
    let modes: BTreeSet<u8> = rows.iter().map(|r| r.mode_active).collect();
    assert_eq!(
        modes,
        BTreeSet::from([
            Mode::OpenLoop as u8,
            Mode::Current as u8,
            Mode::Velocity as u8,
            Mode::Position as u8
        ])
    );
    let parked = row_at(rows, 3_990);
    assert_eq!(parked.cmd, CMD_COAST, "hold parked: {parked:?}");
    let stepped = row_at(rows, 11_990);
    assert_eq!(stepped.cmd, CMD_COAST, "step re-parked: {stepped:?}");
    assert!(
        ((stepped.theta_hat_q16 >> 16) - (START_POS as i32 + 400)).abs() <= 16,
        "step landed: {stepped:?}"
    );
    for t in [14_990, 17_990, 19_590, 21_590] {
        let r = row_at(rows, t);
        assert_eq!(r.fault_flags, 0, "tick {t}: {r:?}");
        assert_eq!(r.cmd, CMD_DRIVE_SLOW, "tick {t}: {r:?}");
    }
    let braked = row_at(rows, 21_990);
    assert_eq!(braked.cmd, CMD_BRAKE, "zero duty brakes: {braked:?}");
    let latched = row_at(rows, 22_990);
    assert_eq!(latched.fault_flags, BIT_OVER_CURRENT, "{latched:?}");
    assert_eq!(latched.fault_code, CODE_OVER_CURRENT);
    assert_eq!(latched.cmd, CMD_DISABLED);
    let acked = row_at(rows, 23_990);
    assert_eq!(acked.fault_flags, 0, "ack cleared the latch: {acked:?}");
    assert_eq!(acked.fault_code, CODE_NONE);
    assert_eq!(acked.cmd, CMD_DRIVE_SLOW);
    assert_eq!(row_at(rows, TICKS - 10).cmd, CMD_DISABLED);
}

fn assert_matches_golden(rows: &[Row]) {
    let golden = std::fs::read(GOLDEN)
        .unwrap_or_else(|e| panic!("{GOLDEN}: {e}; record it with {RECORD_VAR}=1"));
    let (whole, tail) = golden.as_chunks::<ROW_LEN>();
    assert!(
        tail.is_empty(),
        "golden is not whole rows; re-record with {RECORD_VAR}=1"
    );
    let golden: Vec<Row> = whole.iter().map(Row::decode).collect();
    for (i, (l, g)) in rows.iter().zip(golden.iter()).enumerate() {
        assert_eq!(
            l,
            g,
            "kernel trace diverges from the golden at row {i} (tick {}, {} ms): \
             left = live, right = golden. Re-record with {RECORD_VAR}=1 only for \
             an intended control change.",
            i * DECIM_MED as usize,
            i * DECIM_MED as usize / 20
        );
    }
    assert_eq!(rows.len(), golden.len(), "row count");
}

#[test_log::test]
fn kernel_trace_matches_golden() {
    let rows = run(None, TICKS, script);
    assert_script_phases(&rows);
    if std::env::var_os(RECORD_VAR).is_some() {
        let mut live = Vec::with_capacity(rows.len() * ROW_LEN);
        for r in &rows {
            r.encode(&mut live);
        }
        std::fs::write(GOLDEN, &live).expect("write golden");
        return;
    }
    assert_matches_golden(&rows);
}

/// An all-zero table LIVE is the identity: the same trace, bit for bit.
#[test_log::test]
fn kernel_trace_without_lut_matches_golden() {
    let rows = run(Some(&[0; KNOTS]), TICKS, script);
    assert_script_phases(&rows);
    assert_matches_golden(&rows);
}

/// One position step and a long hold: torque on at rest, the step at
/// 2000, then 1.4 s to settle (the rig's velocity loop creeps back from an
/// overshoot below the plant's stiction, slower than the golden script's
/// window).
fn step_script(t: u32, sh: &Shared, _plant: &mut Plant) {
    let set = |f: fn(&mut ControlTable)| sh.table.with_mut(f);
    match t {
        1_000 => set(|t| {
            t.control.lifecycle.goal_position = START_POS as i32;
            t.control.lifecycle.torque_enable = true;
        }),
        2_000 => set(|t| t.control.lifecycle.goal_position = STEP_GOAL),
        _ => {}
    }
}

const STEP_TICKS: u32 = 30_000;
const STEP_GOAL: i32 = START_POS as i32 + 400;

/// The mg90-a table LIVE: the observer tracks the linearized pot the whole
/// way, and the position loop settles at the linearized goal, so the raw
/// pot parks where the table maps the goal from (raw 1881 reads 1900 on
/// mg90-a), not on the goal count as the identity run does.
#[test_log::test]
fn kernel_trace_with_mg90_a_lut_tracks_the_linearized_pot() {
    let k = support::mg90_a();
    let rows = run(Some(&k), STEP_TICKS, step_script);
    for (i, r) in rows.iter().enumerate() {
        let lin = (interp_q4(r.pos, &k) as i32) << (16 - GRID_SHIFT);
        // the observer lags a moving pot; +-3 counts of noise parked
        let tol = if r.omega_hat_cps.unsigned_abs() >> 16 > 100 {
            64
        } else {
            6
        };
        assert!(
            (r.theta_hat_q16 - lin).abs() <= tol << 16,
            "row {i}: theta_hat off the linearized pot: {r:?}"
        );
    }
    let parked = row_at(&rows, STEP_TICKS - 10);
    assert_eq!(parked.cmd, CMD_COAST, "hold parked: {parked:?}");
    assert!(
        ((parked.theta_hat_q16 >> 16) - STEP_GOAL).abs() <= 8,
        "settled at the linearized goal: {parked:?}"
    );
    assert!(
        STEP_GOAL - parked.pos as i32 >= 10,
        "the raw pot parks short of the goal count: {parked:?}"
    );
    assert_eq!(interp_q4(1881, &k) >> GRID_SHIFT, STEP_GOAL as u16);

    let identity = row_at(&run(None, STEP_TICKS, step_script), STEP_TICKS - 10);
    assert_eq!(identity.cmd, CMD_COAST, "{identity:?}");
    assert!(
        (STEP_GOAL - identity.pos as i32).abs() <= 8,
        "identity parks on the goal count: {identity:?}"
    );
}

#[test]
fn row_codec_round_trips() {
    let r = Row {
        fast_crc: 0xBB3D,
        cmd: CMD_DRIVE_FAST,
        mode_active: Mode::Velocity as u8,
        fault_code: 3,
        omega_hat_src: 1,
        fault_flags: 0x21,
        pos: 4095,
        current_bias_counts: BIAS,
        theta_hat_q16: -1 << 20,
        omega_hat_cps: 1234 << 16,
        tau_d_counts: -77,
        i_lim_counts: 1200,
        t_winding_cc: -150,
        vbus_counts: 3000,
        duty_applied_q15: -8000,
        omega_bemf_cps: -2,
        r_hat_q12: 8192,
        i_hat_counts: 301,
    };
    let mut b = Vec::new();
    r.encode(&mut b);
    assert_eq!(b.len(), ROW_LEN);
    assert_eq!(Row::decode(b.as_slice().try_into().unwrap()), r);
}

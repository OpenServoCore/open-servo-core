//! The thermometer's published trace pinned against a recording. One
//! script holds a seat on a fixed frame in OpenLoop: a first reference
//! closes and tracks, a second seat re-bases, the hold derates on the ramp
//! throughout, then five seats after long idles feed the cold check until
//! it flags the stored cold R (10% low). A row lands at each PUBLISH that
//! moves `r_hat_q12` or `therm_flags`; every 4096 periods a row carries the
//! CRC of each period's temperature, limit and NTC. A change to what the
//! thermometer publishes, or when, fails at the first diverging row.
//!
//! Re-record only for an intended thermometer change:
//! `THERMAL_TRACE_RECORD=1 cargo test -p osc-integration --test thermal_trace`

use osc_integration::plant::{BIAS, kernel, seed, vbus_raw};
use osc_protocol::crc::osc_crc_continue;
use osc_servo_core::kernel::{DECIM_MED, DECIM_SLOW, phase};
use osc_servo_core::{ControlTable, Kernel, Mode, RegionStorage, SensorFrame, Shared};

const GOLDEN: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/thermal_trace.golden");
const RECORD_VAR: &str = "THERMAL_TRACE_RECORD";

/// Fast ticks per SLOW tick.
const SLOW: u32 = DECIM_MED as u32 * DECIM_SLOW as u32;
/// A seat closes its reference 192 gated SLOW ticks in; the first waits
/// for the observer to settle on the seat.
const HOLD_SLOW: u32 = 400;
/// Past the thermometer's 11 min without current that makes the next seat
/// a cold reading.
const IDLE_SLOW: u32 = 41_300;
const COLD_SEATS: u16 = 5;
const CRC_PERIODS: u32 = 4096;
/// The rig hold's own R, 722 x 4096 / 400 = 7393, stored 10% low as the
/// cold R.
const R_COLD_Q12: u16 = 6654;

/// A held drive: the crest `current` over the trough's bias, both taps on.
fn frame(pos: u16, current: u16) -> SensorFrame {
    SensorFrame {
        pos,
        current,
        current_trough: BIAS,
        vmotor_a: 3000,
        vmotor_a_trough: 3000,
        vmotor_b: 40,
        vmotor_b_trough: 40,
        vcal: 1200,
        vbus_raw: vbus_raw(3000),
        ntc_raw: 2048,
        tick: 0,
    }
}

/// The bench MG90 model on the rig (the core kernel tests' thermometer),
/// the derate ramp from 35 C, OpenLoop at a held duty.
fn seed_thermometer(sh: &Shared) {
    sh.table.with_mut(|t| {
        t.calib.thermal.th_alpha_q24 = 2097;
        t.calib.thermal.th_g_q016 = 100;
        t.calib.thermal.th_mu_q016 = 5369;
        t.calib.winding.r0_q12 = R_COLD_Q12;
        t.calib.thermal.ntc_raw_ref = 2048;
        t.calib.thermal.ntc_t_ref_cc = 2500;
        t.calib.thermal.ntc_k1_q88 = 563;
        t.calib.thermal.ntc_k2_q24 = 5800;
        t.config.thermal.derate_start_cc = 3500;
        t.config.thermal.cutoff_cc = 6500;
        t.config.thermal.recover_cc = 4000;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
}

struct Trace {
    rows: Vec<String>,
    period: u32,
    crc: u16,
    last: Option<(u16, u8)>,
}

impl Trace {
    fn run<I: osc_servo_core::ControlIo>(
        &mut self,
        k: &mut Kernel<I>,
        sh: &Shared,
        f: SensorFrame,
        ticks: u32,
    ) {
        for _ in 0..ticks {
            let published = k.probe_tags().phase == phase::PUBLISH;
            k.on_tick(f, sh);
            if published {
                self.publish(sh);
            }
        }
    }

    fn publish(&mut self, sh: &Shared) {
        let (t, r_hat, i_lim, t_ntc, flags) = sh.table.with(|t: &ControlTable| {
            let e = &t.telemetry.estimates;
            (
                e.t_winding_cc,
                e.r_hat_q12,
                e.i_lim_counts,
                t.telemetry.therm.t_ntc_cc,
                t.telemetry.therm.therm_flags,
            )
        });
        let p = self.period;
        if self.last != Some((r_hat, flags)) {
            self.rows
                .push(format!("E {p} r_hat {r_hat} flags {flags} t {t}"));
            self.last = Some((r_hat, flags));
        }
        let mut b = [0u8; 6];
        b[..2].copy_from_slice(&t.to_le_bytes());
        b[2..4].copy_from_slice(&i_lim.to_le_bytes());
        b[4..].copy_from_slice(&t_ntc.to_le_bytes());
        self.crc = osc_crc_continue(self.crc, &b);
        self.period += 1;
        if self.period.is_multiple_of(CRC_PERIODS) {
            self.rows
                .push(format!("C {} {:04x}", self.period, self.crc));
        }
    }
}

#[test]
fn thermal_trace_matches_the_recording() {
    let sh = Shared::new();
    seed(&sh);
    seed_thermometer(&sh);
    let mut k = kernel();
    let mut tr = Trace {
        rows: Vec::new(),
        period: 0,
        crc: 0,
        last: None,
    };
    let enable = |on: bool| {
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = on)
    };
    tr.run(&mut k, &sh, frame(2000, BIAS + 400), HOLD_SLOW * SLOW);
    tr.run(&mut k, &sh, frame(2100, BIAS + 396), HOLD_SLOW * SLOW);
    for seat in 0..COLD_SEATS {
        enable(false);
        tr.run(&mut k, &sh, frame(2100 + 50 * seat, BIAS), IDLE_SLOW * SLOW);
        enable(true);
        tr.run(
            &mut k,
            &sh,
            frame(2150 + 50 * seat, BIAS + 400),
            HOLD_SLOW * SLOW,
        );
    }
    let got = tr.rows.join("\n") + "\n";
    if std::env::var_os(RECORD_VAR).is_some() {
        std::fs::write(GOLDEN, &got).expect("write the golden");
        return;
    }
    let want = std::fs::read_to_string(GOLDEN).expect("read the golden");
    for (n, (g, w)) in got.lines().zip(want.lines()).enumerate() {
        assert_eq!(g, w, "row {n} diverges");
    }
    assert_eq!(got.lines().count(), want.lines().count(), "row count");
}

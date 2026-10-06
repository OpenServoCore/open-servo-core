//! What both capture tools do before anything moves, and again before every
//! capture and after every reconnect: read the servo, refuse a capture it
//! cannot run unattended, and prove the shaft free. The duty the jam check
//! moved the shaft at seeds every seek and the park, each raised only off a
//! stop and never past the duty whose stall draws the current limit. A
//! blocked shaft ends the run where it stopped, torque off.

use std::path::Path;

use anyhow::{Context, Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_ident::exp::centre::Centre;
use osc_ident::exp::rl::Scales;
use osc_ident::exp::{Guarded, RigParams};
use osc_ident::limits::{CLASS_R_MIN, DutyPlan, ServoLimits, pct_floor, q15_floor};
use osc_ident::regs::calib;
use osc_ident::run::{self as order, Run};
use osc_ident::runway;
use osc_ident::units::SenseParams;
use serde_json::{Value, json};

use super::envelope::Envelope;
use super::verdict::Abort;
use super::{Rule, Supply};
use crate::rig::battery;
use crate::rig::centre::centre_on;
use crate::rig::limits::{self, Stall};
use crate::rig::pump::read_snapshot;
use crate::rig::snapshot::read_u16;
use crate::sweep::{self, Decay, pct_q15};

/// The exit code of a run a blocked shaft ended.
pub(crate) const BLOCKED_EXIT: i32 = 4;

/// How far this rail may sit from the one an envelope was measured on, a
/// fraction of it: steady speed follows the rail.
const RAIL_TOL: f64 = 0.05;

/// `limit_flags` bit 3: the stall permit lease is live.
const LIMIT_PERMIT: u8 = 1 << 3;

/// What the front read off the servo.
#[derive(Clone, Debug)]
pub(crate) struct Front {
    pub(crate) lim: ServoLimits,
    pub(crate) stall: Stall,
    pub(crate) sc: Scales,
    pub(crate) data_flags: u8,
    pub(crate) limit_flags: u8,
    pub(crate) fault_flags: u8,
    pub(crate) fault_code: u8,
    pub(crate) rail_mv: u32,
    pub(crate) pack_mv: Option<u32>,
    pub(crate) tick_hz: u16,
}

/// What the jam check proved: the shaft moves, at `moved`, and the plan
/// every seek and the park drive by.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct Proved {
    pub(crate) moved: f64,
    pub(crate) plan: DutyPlan,
}

impl Proved {
    pub(crate) fn seek_pct(&self) -> u8 {
        pct_floor(self.plan.seek)
    }

    /// The most a seek raises to, off a stop only: the stall-safe cap.
    pub(crate) fn cap_pct(&self) -> u8 {
        pct_floor(self.plan.stop_cap)
    }

    pub(crate) fn seek_q15(&self) -> i16 {
        pct_q15(self.seek_pct())
    }
}

/// Read the servo and refuse, in plain words, anything a capture on
/// `supply` cannot run under. The pack gate, read at rest, refuses first.
pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id, supply: Supply) -> Result<Front> {
    let data = crate::state::check(c, id)?;
    let lim = limits::read(c, id)?;
    let stall = Stall::read(c, id)?;
    let tel = read_snapshot(c, id)?;
    let (rail_mv, pack_mv) = battery::read(c, id)?;
    let tick_hz = read_u16(c, id, calib::TICK_HZ)?;
    let sense = SenseParams {
        shunt_r_mohm: read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz,
    };
    let sc = Scales::from_sense(
        &sense,
        read_u16(c, id, calib::VBUS_DIV_TOP_OHM)?,
        read_u16(c, id, calib::VBUS_DIV_BOT_OHM)?,
    )
    .context("CalibSense scales degenerate (shunt/gain/dividers/vdd)")?;
    let front = Front {
        lim,
        stall,
        sc,
        data_flags: data.flags,
        limit_flags: tel.limit_flags,
        fault_flags: tel.fault_flags,
        fault_code: tel.fault_code,
        rail_mv,
        pack_mv,
        tick_hz,
    };
    front.check(supply)?;
    Ok(front)
}

impl Front {
    fn check(&self, supply: Supply) -> Result<()> {
        self.lim.guard()?;
        if self.lim.r_q12 == 0 {
            bail!(
                "this servo has no winding resistance stored: run `osc ident run` and `osc ident \
                 write` first"
            );
        }
        if let Some(why) = self.inert() {
            bail!(
                "the servo's stall settings cannot fold a stall ({why}): a capture runs \
                 unattended, set them first"
            );
        }
        if self.limit_flags & LIMIT_PERMIT != 0 {
            bail!("the stall permit is live: another tool is driving this servo");
        }
        if self.fault_flags != 0 {
            bail!(
                "a fault is latched on the servo (flags {:#04x}, code {}): another tool is \
                 driving it, or its last drive faulted",
                self.fault_flags,
                self.fault_code
            );
        }
        let on = runway::Supply::of_rail(self.rail_mv as f64);
        if on != Some(supply.runway()) {
            let is = on.map_or("neither USB nor a 2S pack".to_string(), |s| {
                format!("a {s} supply")
            });
            bail!(
                "--supply {}, but the rail reads {} mV, {is}",
                supply.as_str(),
                self.rail_mv
            );
        }
        Ok(())
    }

    /// Why a held stall would stay at the current limit: the stall timer
    /// folds to a yield that is no lower, or lets go of the fold before a
    /// stall held at the yield reads under the release. A timer that
    /// latches a fault instead always acts.
    fn inert(&self) -> Option<String> {
        if !self.stall.folds {
            return None;
        }
        let ma = self.lim.ma();
        let (y, l, r) = (self.lim.stall_yield, self.lim.i_lim, self.stall.release);
        if y >= l {
            return Some(format!(
                "stall yield {} is not under the current limit {}",
                ma.of(y as f64),
                ma.of(l as f64)
            ));
        }
        if r >= y {
            return Some(format!(
                "stall release {} is not under the stall yield {}",
                ma.of(r as f64),
                ma.of(y as f64)
            ));
        }
        None
    }

    /// Refuse an envelope, `path`, that does not describe this servo under
    /// its limit on this rail.
    pub(crate) fn check_envelope(&self, env: &Envelope, path: &Path) -> Result<()> {
        let (p, again) = (path.display(), "run `osc capture pilot` again");
        if env.rule == Rule::Free {
            bail!("{p} was measured before the servo limited open-loop current: {again}");
        }
        let ma = self.lim.ma();
        let limit = ma.of(self.lim.i_lim as f64);
        match env.current_limit_counts {
            Some(l) if l == self.lim.i_lim => {}
            Some(l) => bail!(
                "{p} was measured under a current limit of {}, the servo holds {limit}: {again}",
                ma.of(l as f64)
            ),
            None => bail!("{p} names no current limit: {again}"),
        }
        match env.rail_mv {
            Some(r) if (self.rail_mv as f64 - r as f64).abs() <= RAIL_TOL * r as f64 => Ok(()),
            Some(r) => bail!(
                "{p} was measured on a {r} mV rail and this one reads {} mV, more than {:.0}% \
                 away: {again}",
                self.rail_mv,
                RAIL_TOL * 100.0
            ),
            None => bail!("{p} names no rail: {again}"),
        }
    }

    /// The rig the jam check runs in: the guard inside the soft limits, the
    /// abort a quarter over the current limit, the stops `osc cal` found.
    pub(crate) fn params(&self) -> RigParams {
        let p = RigParams::new(self.lim.guard().ok(), self.lim.abort_default())
            .with_floor(self.lim.window_floor_q15);
        if self.lim.raw.0 < self.lim.raw.1 {
            p.with_stops(self.lim.raw)
        } else {
            p
        }
    }

    /// The jam check as `osc cal` runs it, pumped by `pump`: out and back at
    /// mid travel from the class-safe duty, raised while the shaft does not
    /// move. The duty it moved at plans the drives by the winding R the
    /// servo carries.
    pub(crate) fn jam_check(
        &self,
        pump: impl FnOnce(&mut Guarded<Centre>) -> Result<()>,
    ) -> Result<Proved> {
        let run = Run::new(self.lim, &self.sc);
        let (from, cap) = (run.bootstrap(), run.nudge_cap());
        println!(
            "[jam check] out and back at mid travel from {}, raised up to {} while the shaft \
             does not move",
            pct(from),
            pct(cap)
        );
        let cfg = order::centre_cfg(from, cap, true);
        let Some(moved) = centre_on(pump, cfg, self.params(), "the jam check")?.moved_at() else {
            bail!("the jam check never saw the shaft move, so nothing drives it");
        };
        let proved = Proved {
            moved,
            plan: self.lim.stall_plan(self.sc.r_vpc(CLASS_R_MIN), Some(moved)),
        };
        println!(
            "  the shaft moves at {}: seeks and the park drive at {}%, raised off a stop up to \
             {}%, where a stall draws the current limit of {}",
            pct(moved),
            proved.seek_pct(),
            proved.cap_pct(),
            self.lim.ma().of(self.lim.i_lim as f64)
        );
        Ok(proved)
    }

    /// What a recording under `decay` is held against: the abort a quarter
    /// over the limit, and what sizes a reversal's residual.
    pub(crate) fn abort(&self, decay: Decay) -> Abort {
        let lim = &self.lim;
        Abort {
            i_abort: lim.abort_default() as f64,
            floor_q15: lim.window_floor_q15,
            rail: lim.vbus as f64,
            r_vpc: lim.r_q12 as f64 / 4096.0,
            fast: decay == Decay::Fast,
            tick_hz: self.tick_hz as f64,
            ma: lim.ma(),
        }
    }

    /// meta.json's `drive` block: the rule, what the front read and what
    /// the jam check proved.
    pub(crate) fn drive(&self, proved: &Proved) -> Value {
        let lim = &self.lim;
        let mut d = sweep::drive(lim, &self.stall, proved.seek_q15());
        let more = json!({
            "moved_q15": q15_floor(proved.moved),
            "stop_cap_q15": pct_q15(proved.cap_pct()),
            "soft": [lim.soft.0, lim.soft.1],
            "phys": [lim.phys.0, lim.phys.1],
            "data_flags": self.data_flags,
            "limit_flags": self.limit_flags,
            "fault_flags": self.fault_flags,
            "rail_mv": self.rail_mv,
            "pack_mv": self.pack_mv,
        });
        if let Value::Object(m) = more {
            d.extend(m);
        }
        Value::Object(d)
    }
}

/// End the run on a blocked shaft: nothing may drive it again, not even to
/// park it, and the drive that found it left torque off and the permit
/// clear.
pub(crate) fn exit_blocked(e: &anyhow::Error) -> ! {
    eprintln!("Error: {e:?}");
    std::process::exit(BLOCKED_EXIT)
}

/// A duty fraction as percent, to the tenth.
fn pct(duty: f64) -> String {
    format!("{:.1}%", duty * 100.0)
}

/// The bench MG90 on 2S as the front reads it: limit 280 counts, stall
/// yield 168 released under 84, R 7270 stored, the rail at 7.9 V.
#[cfg(test)]
pub(super) fn bench() -> Front {
    let sc = osc_ident::exp::testkit::board_d_scales();
    Front {
        lim: ServoLimits {
            i_lim: 280,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (232, 3849),
            raw: (232, 3849),
            r_q12: 7270,
            vbus: 3204,
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: sc.amps_per_count,
            drive_polarity: true,
        },
        stall: Stall {
            folds: true,
            time_ms: 500,
            release: 84,
        },
        sc,
        data_flags: 0,
        limit_flags: 0,
        fault_flags: 0,
        fault_code: 0,
        rail_mv: 7900,
        pack_mv: Some(8150),
        tick_hz: 20_000,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::servo::bench::{set, table as servo, torque};
    use osc_client::fake::FakePipe;
    use osc_ident::regs::config;

    fn refusal(c: &mut Client<FakePipe>, id: Id, supply: Supply) -> String {
        let e = read(c, id, supply).unwrap_err().to_string();
        assert_eq!(torque(c, id), 0, "{e}: refused before anything moved");
        e
    }

    #[test]
    fn a_servo_without_r_is_sent_to_ident() {
        let (mut c, id) = servo(1961);
        let f = read(&mut c, id, Supply::TwoS).unwrap();
        assert_eq!((f.lim.i_lim, f.lim.r_q12, f.rail_mv), (280, 7270, 7899));
        set(&mut c, id, calib::R_Q12, 0);
        assert_eq!(
            refusal(&mut c, id, Supply::TwoS),
            "this servo has no winding resistance stored: run `osc ident run` and `osc ident \
             write` first"
        );
    }

    /// The bench servo's saved stall settings, yield 545 and release 273
    /// over its 280 limit: a stall held there never backs off. A release
    /// over the yield lets every fold go at once. A stall that latches a
    /// fault acts whatever the yield.
    #[test]
    fn an_inert_stall_response_is_refused() {
        let (mut c, id) = servo(1961);
        set(&mut c, id, config::STALL_YIELD_COUNTS, 545);
        set(&mut c, id, config::STALL_RELEASE_COUNTS, 273);
        assert_eq!(
            refusal(&mut c, id, Supply::TwoS),
            "the servo's stall settings cannot fold a stall (stall yield 545 counts (492 mA) is \
             not under the current limit 280 counts (253 mA)): a capture runs unattended, set \
             them first"
        );
        set(&mut c, id, config::STALL_YIELD_COUNTS, 168);
        set(&mut c, id, config::STALL_RELEASE_COUNTS, 200);
        assert_eq!(
            refusal(&mut c, id, Supply::TwoS),
            "the servo's stall settings cannot fold a stall (stall release 200 counts (180 mA) \
             is not under the stall yield 168 counts (152 mA)): a capture runs unattended, set \
             them first"
        );
        set(&mut c, id, config::STALL_YIELD_COUNTS, 545);
        set(&mut c, id, config::STALL_RESPONSE, 0);
        let f = read(&mut c, id, Supply::TwoS).unwrap();
        assert!(!f.stall.folds);
    }

    /// The rail says which supply feeds the servo; a live permit or a
    /// latched fault means something else holds it.
    #[test]
    fn a_servo_another_tool_holds_or_another_supply_feeds_is_refused() {
        let (mut c, id) = servo(1961);
        assert_eq!(
            refusal(&mut c, id, Supply::Usb),
            "--supply usb, but the rail reads 7899 mV, a 2S supply"
        );
        let (mut c, id) = servo(1090);
        assert_eq!(
            refusal(&mut c, id, Supply::TwoS),
            "--supply 2s, but the rail reads 4390 mV, a USB supply"
        );
        read(&mut c, id, Supply::Usb).unwrap();

        let held = Front {
            limit_flags: LIMIT_PERMIT,
            ..bench()
        };
        assert_eq!(
            held.check(Supply::TwoS).unwrap_err().to_string(),
            "the stall permit is live: another tool is driving this servo"
        );
        let faulted = Front {
            fault_flags: 0x20,
            fault_code: 6,
            ..bench()
        };
        assert_eq!(
            faulted.check(Supply::TwoS).unwrap_err().to_string(),
            "a fault is latched on the servo (flags 0x20, code 6): another tool is driving it, \
             or its last drive faulted"
        );
        let nowhere = Front {
            rail_mv: 12_000,
            ..bench()
        };
        assert_eq!(
            nowhere.check(Supply::TwoS).unwrap_err().to_string(),
            "--supply 2s, but the rail reads 12000 mV, neither USB nor a 2S pack"
        );
        bench().check(Supply::TwoS).unwrap();
    }

    /// An envelope names the rule, the limit and the rail the pilot drove
    /// under; one without them was measured before the servo limited
    /// open-loop current.
    #[test]
    fn an_envelope_from_before_the_limiter_is_refused_at_the_front() {
        let f = bench();
        let path = Path::new("/d/mg90-a__2s__limit/envelope.toml");
        let again = |why: &str| format!("{} {why}: run `osc capture pilot` again", path.display());
        let env = super::super::envelope::mg90();
        f.check_envelope(&env, path).unwrap();

        let free = Envelope {
            rule: Rule::Free,
            current_limit_counts: None,
            rail_mv: None,
            ..super::super::envelope::mg90()
        };
        assert_eq!(
            f.check_envelope(&free, path).unwrap_err().to_string(),
            again("was measured before the servo limited open-loop current")
        );
        let other = Envelope {
            current_limit_counts: Some(335),
            ..super::super::envelope::mg90()
        };
        assert_eq!(
            f.check_envelope(&other, path).unwrap_err().to_string(),
            again(
                "was measured under a current limit of 335 counts (300 mA), the servo holds \
                 280 counts (251 mA)"
            )
        );
        // 5% of the envelope's 7899 mV is 395
        for (rail, ok) in [(7505, true), (8293, true), (7504, false), (8294, false)] {
            let r = Front {
                rail_mv: rail,
                ..bench()
            }
            .check_envelope(&env, path);
            assert_eq!(r.is_ok(), ok, "{rail} mV");
        }
        assert_eq!(
            Front {
                rail_mv: 7000,
                ..bench()
            }
            .check_envelope(&env, path)
            .unwrap_err()
            .to_string(),
            again("was measured on a 7899 mV rail and this one reads 7000 mV, more than 5% away")
        );
        let bare = Envelope {
            rail_mv: None,
            ..super::super::envelope::mg90()
        };
        assert_eq!(
            f.check_envelope(&bare, path).unwrap_err().to_string(),
            again("names no rail")
        );
    }
}

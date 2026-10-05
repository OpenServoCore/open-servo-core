//! Read the servo's limits for a drive to plan against (osc-ident
//! `limits`), refuse a flat pack and a servo that publishes no window
//! floor, and say out loud when its stall settings leave the current limit
//! as the only protection. Every drive tool reads them before it drives.

use anyhow::{Context, Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::pipe::Pipe;
use osc_ident::limits::{CLASS_R_MIN, DutyPlan, ServoLimits};
use osc_ident::regs::{calib, config, telemetry};
use osc_ident::units::{self, SenseParams};

use super::battery;
use super::pump::{read_i32, read_snapshot};
use super::snapshot::read_u16;

pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<ServoLimits> {
    battery::before_drive(c, id)?;
    let sense = SenseParams {
        shunt_r_mohm: read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: 0,
        vmotor_div_bot: 0,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz: 0,
    };
    let tel = read_snapshot(c, id)?;
    let lim = ServoLimits {
        i_lim: read_u16(c, id, config::CURRENT_LIMIT_COUNTS)?,
        stall_yield: read_u16(c, id, config::STALL_YIELD_COUNTS)?,
        tau_trip: read_u16(c, id, config::STALL_TAU_TRIP_COUNTS)?,
        soft: (
            read_i32(c, id, config::POS_MIN_SOFT_COUNTS)?,
            read_i32(c, id, config::POS_MAX_SOFT_COUNTS)?,
        ),
        phys: (
            read_i32(c, id, config::POS_MIN_PHYS_COUNTS)?,
            read_i32(c, id, config::POS_MAX_PHYS_COUNTS)?,
        ),
        raw: (
            read_u16(c, id, calib::RAW_MIN)?,
            read_u16(c, id, calib::RAW_MAX)?,
        ),
        r_q12: read_u16(c, id, calib::R_Q12)?,
        vbus: tel.vbus_counts,
        window_floor_q15: tel.window_floor_q15,
        window_v_floor_q15: read_u16(c, id, telemetry::WINDOW_V_FLOOR_Q15)?,
        amps_per_count: units::amps_per_count(&sense),
    };
    lim.check_floor()?;
    for w in lim.warnings(read_u16(c, id, config::RTHERM_I_MIN_COUNTS)?) {
        eprintln!("warning: {w}");
    }
    Ok(lim)
}

/// The stall settings `ServoLimits` leaves out: what the stall timer does
/// once it trips, and when a fold lets go.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Stall {
    /// A stall folds the limit to the yield; else it latches a fault.
    pub(crate) folds: bool,
    pub(crate) time_ms: u16,
    pub(crate) release: u16,
}

impl Stall {
    pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Self> {
        let response = c
            .read(id, config::STALL_RESPONSE.addr, 1)
            .context("field read")?[0];
        Ok(Self {
            folds: response != 0,
            time_ms: read_u16(c, id, config::STALL_TIME_MS)?,
            release: read_u16(c, id, config::STALL_RELEASE_COUNTS)?,
        })
    }

    /// The name `stall_response` reads under in the descriptor.
    pub(crate) fn response(&self) -> &'static str {
        if self.folds { "yield" } else { "fault" }
    }
}

/// The stall-safe plan for a drive outside a run: by the winding R the
/// servo carries, else by the class's lowest, which errs safe.
pub(crate) fn plan(c: &mut Client<NusbPipe>, id: Id, lim: &ServoLimits) -> Result<DutyPlan> {
    let sense = SenseParams {
        shunt_r_mohm: read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz: 0,
    };
    let (a, v) = (
        units::amps_per_count(&sense),
        units::volts_per_count(&sense),
    );
    if a <= 0.0 || v <= 0.0 {
        bail!("CalibSense scales degenerate (shunt/gain/dividers/vdd)");
    }
    Ok(lim.stall_plan(CLASS_R_MIN * a / v, None))
}

#[cfg(test)]
mod tests {
    use super::super::servo::bench;

    /// The limits carry both floors the servo publishes, 64 and 160 ticks
    /// on osc-dev-v006, and a drive that fits the terminal differential
    /// plans from the higher; firmware that publishes no terminal floor
    /// (reads 0) leaves the current floor.
    #[test]
    fn limits_carry_both_window_floors() {
        let (mut c, id) = bench::table(1961);
        let lim = super::read(&mut c, id).unwrap();
        assert_eq!((lim.window_v_floor_q15, lim.vdiff_floor()), (0, 4356));
        c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.limits.window_floor_q15 = 1734;
            t.telemetry.limits_ext.window_v_floor_q15 = 4356;
        });
        let lim = super::read(&mut c, id).unwrap();
        assert_eq!((lim.window_floor(), lim.vdiff_floor()), (1734, 4356));
    }
}

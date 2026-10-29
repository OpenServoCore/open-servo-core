//! Read the servo's limits for a drive to plan against (osc-ident
//! `limits`), warn about a flat pack, refuse a servo that publishes no
//! window floor, and say out loud when its stall settings leave the current
//! limit as the only protection. Every drive tool reads them before it
//! drives.

use anyhow::{Context, Result};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_ident::limits::ServoLimits;
use osc_ident::regs::{calib, config, telemetry};
use osc_ident::units::{self, SenseParams};
use osc_protocol::table::{CAP_EXPERIMENT, CAPABILITY_FLAGS};

use super::battery;
use super::pump::{read_i32, read_snapshot};
use super::snapshot::{read_bool, read_u16, read_u32};

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
        i_cont: read_u16(c, id, config::I_CONT_COUNTS)?,
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
        vbus: vbus_counts(c, id, tel.vbus_counts)?,
        window_floor_q15: tel.window_floor_q15,
        window_v_floor_q15: read_u16(c, id, telemetry::WINDOW_V_FLOOR_Q15)?,
        amps_per_count: units::amps_per_count(&sense),
        drive_polarity: read_bool(c, id, config::DRIVE_POLARITY)?,
    };
    lim.check_floor()?;
    for w in lim.warnings() {
        eprintln!("warning: {w}");
    }
    Ok(lim)
}

/// The rail in vmotor-tap counts, the unit `ServoLimits::vbus` plans in.
/// The experiment image runs no estimators (kernel sec 2.7) and publishes
/// `vbus_counts` as 0, so for it the rail tap is rescaled here the way the
/// production estimator scales it, from one sample at rest.
fn vbus_counts<P: Pipe>(c: &mut Client<P>, id: Id, published: u16) -> Result<u16> {
    let raw = c.read(id, CAPABILITY_FLAGS, 4).context("capability read")?;
    let caps = u32::from_le_bytes(
        raw.get(..4)
            .and_then(|b| b.try_into().ok())
            .unwrap_or([0; 4]),
    );
    if caps & CAP_EXPERIMENT == 0 {
        return Ok(published);
    }
    Ok(vcounts_of_rail_tap(
        read_u16(c, id, telemetry::VBUS_RAW)?,
        (
            read_u16(c, id, calib::VBUS_DIV_TOP_OHM)?,
            read_u16(c, id, calib::VBUS_DIV_BOT_OHM)?,
        ),
        (
            read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
            read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        ),
    ))
}

/// `vbus_raw` in vmotor-tap counts, by the production estimator's scale
/// `(vbus_top + vbus_bot) * vmotor_bot / (vbus_bot * (vmotor_top +
/// vmotor_bot))`; 0 when a divider bottom is unset.
fn vcounts_of_rail_tap(raw: u16, (vt, vb): (u16, u16), (mt, mb): (u16, u16)) -> u16 {
    let num = raw as u64 * (vt as u64 + vb as u64) * mb as u64;
    num.checked_div(vb as u64 * (mt as u64 + mb as u64))
        .map_or(0, |v| v.min(u16::MAX as u64) as u16)
}

/// The stall settings `ServoLimits` leaves out: what the stall timer does
/// once it trips, and when a fold lets go.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Stall {
    /// A stall folds the limit to the continuous current; else it latches
    /// a fault.
    pub(crate) folds: bool,
    /// Kernel ticks.
    pub(crate) time_ticks: u32,
    pub(crate) release: u16,
}

impl Stall {
    pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Self> {
        let response = c
            .read(id, config::STALL_RESPONSE.addr, 1)
            .context("field read")?[0];
        Ok(Self {
            folds: response != 0,
            time_ticks: read_u32(c, id, config::STALL_TIME_TICKS)?,
            release: read_u16(c, id, config::STALL_RELEASE_COUNTS)?,
        })
    }

    /// The name `stall_response` reads under in the descriptor.
    pub(crate) fn response(&self) -> &'static str {
        if self.folds { "yield" } else { "fault" }
    }
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
            t.telemetry_ext.limits_ext.window_v_floor_q15 = 4356;
        });
        let lim = super::read(&mut c, id).unwrap();
        assert_eq!((lim.window_floor(), lim.vdiff_floor()), (1734, 4356));
    }

    /// The experiment image publishes `vbus_counts` as 0: the limits carry
    /// the rail tap rescaled into vmotor-tap counts instead, the same rail
    /// the battery gate reads from `vbus_raw`, to a count. Any other image
    /// keeps the published estimate.
    #[test]
    fn experiment_image_rail_comes_from_the_rail_tap() {
        use osc_client::fake::TelSample;
        use osc_ident::units::{SenseParams, volts_per_count};
        let (mut c, id) = bench::table(1961);
        assert_eq!(super::read(&mut c, id).unwrap().vbus, 3204);
        let mut s = TelSample::default();
        (s.tick.pos, s.tick.vbus_raw) = (2029, 1961);
        c.pipe_mut().set_track(0, vec![s]);
        c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.config.common.capability_flags |= osc_protocol::table::CAP_EXPERIMENT;
        });
        let vbus = super::read(&mut c, id).unwrap().vbus;
        let (rail_mv, _) = super::battery::read(&mut c, id).unwrap();
        let sense = SenseParams {
            shunt_r_mohm: 0,
            gain_milli: 0,
            vmotor_div_top: super::read_u16(&mut c, id, super::calib::VMOTOR_DIV_TOP).unwrap(),
            vmotor_div_bot: super::read_u16(&mut c, id, super::calib::VMOTOR_DIV_BOT).unwrap(),
            vdd_mv: super::read_u16(&mut c, id, super::calib::VDD_MV).unwrap(),
            tick_hz: 0,
        };
        let mv_per_count = volts_per_count(&sense) * 1000.0;
        assert!(vbus > 0);
        assert!((vbus as f64 * mv_per_count - rail_mv as f64).abs() <= mv_per_count);
    }

    #[test]
    fn rail_tap_rescale_is_zero_on_an_unset_divider() {
        assert_eq!(
            super::vcounts_of_rail_tap(1961, (15_000, 0), (6400, 1600)),
            0
        );
        assert_eq!(
            super::vcounts_of_rail_tap(1961, (15_000, 10_000), (0, 0)),
            0
        );
    }
}

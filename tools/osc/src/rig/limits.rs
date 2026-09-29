//! Read the servo's limits for a drive to plan against (osc-ident
//! `limits`), and say out loud when its stall settings leave the current
//! limit as the only protection.

use anyhow::Result;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::limits::ServoLimits;
use osc_ident::regs::{calib, config};
use osc_ident::units::{self, SenseParams};

use super::pump::{read_i32, read_snapshot};
use super::snapshot::read_u16;

pub(crate) fn read(c: &mut Client<NusbPipe>, id: Id) -> Result<ServoLimits> {
    let sense = SenseParams {
        shunt_r_mohm: read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: 0,
        vmotor_div_bot: 0,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz: 0,
    };
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
        vbus: read_snapshot(c, id)?.vbus_counts,
        i_floor_ticks: read_u16(c, id, calib::I_WINDOW_MIN_TICKS)?,
        amps_per_count: units::amps_per_count(&sense),
    };
    for w in lim.warnings() {
        eprintln!("warning: {w}");
    }
    Ok(lim)
}

//! Battery gate: a capture never starts on a pack near empty. The pack is
//! read at rest as the rail through the board's vbus divider plus its
//! rail_drop_mv, all from the servo's own calib table.

use anyhow::Result;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::regs::{calib, telemetry};

use super::procs::SupplyCfg;
use crate::rig::snapshot::read_u16;

/// Codes of the 12-bit vbus ADC.
const ADC_CODES: u64 = 4096;

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Verdict {
    Ok,
    Warn(String),
    Refuse(String),
}

/// Rail at the divider top, mV; 0 when the divider bottom is unset.
pub(crate) fn rail_mv(vbus_raw: u16, vdd_mv: u16, top_ohm: u16, bot_ohm: u16) -> u32 {
    let num = vbus_raw as u64 * vdd_mv as u64 * (top_ohm as u64 + bot_ohm as u64);
    num.checked_div(ADC_CODES * bot_ohm as u64)
        .map_or(0, |mv| mv.min(u32::MAX as u64) as u32)
}

/// Pack mV from the rail; None when the rail reads 0 (no reading, or no
/// divider calibration), which the gate refuses.
pub(crate) fn pack_mv(rail_mv: u32, rail_drop_mv: u16) -> Option<u32> {
    (rail_mv > 0).then(|| rail_mv + rail_drop_mv as u32)
}

/// A supply with no gate configured (usb) always passes.
pub(crate) fn gate(cfg: Option<&SupplyCfg>, pack_mv: Option<u32>) -> Verdict {
    let Some(s) = cfg else {
        return Verdict::Ok;
    };
    let Some(pack) = pack_mv else {
        return Verdict::Refuse("rail unreadable: vbus_raw or the vbus divider reads 0".into());
    };
    let cells = s.cells;
    if pack < cells * s.cell_floor_mv {
        Verdict::Refuse(format!(
            "pack {pack} mV is under {cells} x {} mV: charge the pack",
            s.cell_floor_mv
        ))
    } else if pack < cells * s.cell_warn_mv {
        Verdict::Warn(format!(
            "pack {pack} mV is under {cells} x {} mV",
            s.cell_warn_mv
        ))
    } else {
        Verdict::Ok
    }
}

pub(crate) fn read_pack_mv(c: &mut Client<NusbPipe>, id: Id) -> Result<Option<u32>> {
    let rail = rail_mv(
        read_u16(c, id, telemetry::VBUS_RAW)?,
        read_u16(c, id, calib::VDD_MV)?,
        read_u16(c, id, calib::VBUS_DIV_TOP_OHM)?,
        read_u16(c, id, calib::VBUS_DIV_BOT_OHM)?,
    );
    Ok(pack_mv(rail, read_u16(c, id, calib::RAIL_DROP_MV)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    const TWO_S: SupplyCfg = SupplyCfg {
        cells: 2,
        cell_floor_mv: 3500,
        cell_warn_mv: 3700,
    };

    /// The bench board: 15k/10k divider, 3.3 V reference, 250 mV diode drop.
    fn bench(raw: u16) -> Option<u32> {
        pack_mv(rail_mv(raw, 3300, 15_000, 10_000), 250)
    }

    #[test]
    fn rail_scales_through_the_divider() {
        assert_eq!(rail_mv(3351, 3300, 15_000, 10_000), 6749);
        assert_eq!(rail_mv(3352, 3300, 15_000, 10_000), 6751);
        assert_eq!(rail_mv(3555, 3300, 15_000, 10_000), 7160);
        assert_eq!(rail_mv(4095, 3300, 15_000, 0), 0);
    }

    #[test]
    fn gate_thresholds_on_the_bench_numbers() {
        let at = |raw| gate(Some(&TWO_S), bench(raw));
        assert!(matches!(at(3351), Verdict::Refuse(m) if m.contains("6999 mV")));
        assert!(matches!(at(3352), Verdict::Warn(m) if m.contains("7001 mV")));
        assert_eq!(at(3555), Verdict::Ok);
        assert!(matches!(at(0), Verdict::Refuse(m) if m.contains("rail unreadable")));
    }

    #[test]
    fn usb_is_never_gated() {
        for pack in [None, Some(0), Some(4500), Some(8400)] {
            assert_eq!(gate(None, pack), Verdict::Ok);
        }
    }
}

//! Battery gate: nothing drives on a pack near empty. The pack is read at
//! rest as the rail through the board's vbus divider plus its
//! rail_drop_mv, all from the servo's own calib table. The capture session
//! gates the supply it was told; a drive tool reads the rail and takes a
//! rail in USB's range for USB, which has no gate, and anything else for a
//! 2S pack.

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_ident::regs::{calib, telemetry};
use osc_ident::runway::Supply;

use super::snapshot::read_u16;

/// Codes of the 12-bit vbus ADC.
const ADC_CODES: u64 = 4096;

/// A pack's gate: refuse under `cells` x `cell_floor_mv`, warn under
/// `cells` x `cell_warn_mv`.
#[derive(serde::Deserialize, Debug, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub(crate) struct SupplyCfg {
    pub(crate) cells: u32,
    pub(crate) cell_floor_mv: u32,
    pub(crate) cell_warn_mv: u32,
}

/// A 2S pack, as the capture session's procedure gates it: 3.5 V a cell
/// at rest, a rail of 6.75 V behind the feed diode.
pub(crate) const TWO_S: SupplyCfg = SupplyCfg {
    cells: 2,
    cell_floor_mv: 3500,
    cell_warn_mv: 3700,
};

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

/// Millivolts as volts to the hundredth, rounded down: a pack a millivolt
/// under its floor never reads as the floor.
fn volts(mv: u32) -> String {
    format!("{}.{:02} V", mv / 1000, mv % 1000 / 10)
}

/// A supply with no gate configured (usb) always passes.
pub(crate) fn gate(cfg: Option<&SupplyCfg>, pack_mv: Option<u32>) -> Verdict {
    let Some(s) = cfg else {
        return Verdict::Ok;
    };
    let Some(pack) = pack_mv else {
        return Verdict::Refuse(
            "the supply voltage is unreadable: vbus_raw or the vbus divider in the calib table \
             reads 0"
                .into(),
        );
    };
    let (floor, warn) = (s.cells * s.cell_floor_mv, s.cells * s.cell_warn_mv);
    if pack < floor {
        Verdict::Refuse(format!(
            "the pack reads {} at rest, under its floor of {} ({} a cell): charge it before \
             anything drives",
            volts(pack),
            volts(floor),
            volts(s.cell_floor_mv)
        ))
    } else if pack < warn {
        Verdict::Warn(format!(
            "the pack reads {} at rest, near its floor: under {} ({} a cell)",
            volts(pack),
            volts(warn),
            volts(s.cell_warn_mv)
        ))
    } else {
        Verdict::Ok
    }
}

/// The gate a drive tool passes: none for a rail in USB's range, a 2S
/// pack's for any other.
pub(crate) fn drive_gate(rail_mv: u32, pack_mv: Option<u32>) -> Verdict {
    let usb = Supply::of_rail(rail_mv as f64) == Some(Supply::Usb);
    gate((!usb).then_some(&TWO_S), pack_mv)
}

/// The rail and the pack, mV, as read now.
pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<(u32, Option<u32>)> {
    let rail = rail_mv(
        read_u16(c, id, telemetry::VBUS_RAW)?,
        read_u16(c, id, calib::VDD_MV)?,
        read_u16(c, id, calib::VBUS_DIV_TOP_OHM)?,
        read_u16(c, id, calib::VBUS_DIV_BOT_OHM)?,
    );
    Ok((rail, pack_mv(rail, read_u16(c, id, calib::RAIL_DROP_MV)?)))
}

pub(crate) fn read_pack_mv<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Option<u32>> {
    Ok(read(c, id)?.1)
}

/// Refuse to drive on a flat pack; read at rest, before anything turns
/// torque on.
pub(crate) fn before_drive<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<()> {
    let (rail, pack) = read(c, id)?;
    match drive_gate(rail, pack) {
        Verdict::Ok => Ok(()),
        Verdict::Warn(m) => {
            eprintln!("warning: {m}");
            Ok(())
        }
        Verdict::Refuse(m) => bail!("{m}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_client::BaudRate;
    use osc_client::fake::{FakePipe, TelSample};
    use osc_ident::regs::control;

    /// The bench board: 15k/10k divider, 3.3 V reference, 250 mV diode drop.
    fn bench(raw: u16) -> (u32, Option<u32>) {
        let rail = rail_mv(raw, 3300, 15_000, 10_000);
        (rail, pack_mv(rail, 250))
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
        let at = |raw| gate(Some(&TWO_S), bench(raw).1);
        assert_eq!(
            at(3351),
            Verdict::Refuse(
                "the pack reads 6.99 V at rest, under its floor of 7.00 V (3.50 V a cell): \
                 charge it before anything drives"
                    .into()
            )
        );
        assert_eq!(
            at(3352),
            Verdict::Warn(
                "the pack reads 7.00 V at rest, near its floor: under 7.40 V (3.70 V a cell)"
                    .into()
            )
        );
        assert_eq!(at(3555), Verdict::Ok);
        assert!(matches!(at(0), Verdict::Refuse(m) if m.contains("unreadable")));
    }

    #[test]
    fn usb_is_never_gated() {
        for pack in [None, Some(0), Some(4500), Some(8400)] {
            assert_eq!(gate(None, pack), Verdict::Ok);
        }
    }

    /// A drive tool on USB, 4.39 V at rest, whatever the pack floor: no
    /// gate. The same board on a rail the 2S gate refuses and on one it
    /// passes.
    #[test]
    fn usb_has_no_battery_gate() {
        let usb = bench(2180);
        assert_eq!(usb.0, 4390);
        assert_eq!(drive_gate(usb.0, usb.1), Verdict::Ok);
        assert!(matches!(
            drive_gate(bench(3351).0, bench(3351).1),
            Verdict::Refuse(_)
        ));
        assert_eq!(drive_gate(bench(3555).0, bench(3555).1), Verdict::Ok);
        // an unreadable rail is no USB supply
        assert!(matches!(drive_gate(0, None), Verdict::Refuse(_)));

        let (mut c, id) = fake(2180);
        before_drive(&mut c, id).expect("USB is not gated");
    }

    /// The bench servo on a 2S pack at 3.5 V a cell less a millivolt: the
    /// gate every drive tool runs when it reads the servo's limits refuses
    /// it in plain words, and the servo was never told to turn torque on.
    #[test]
    fn a_flat_pack_is_refused_before_torque_on() {
        let (mut c, id) = fake(3351);
        let err = crate::rig::limits::read(&mut c, id).unwrap_err();
        assert_eq!(
            err.to_string(),
            "the pack reads 6.99 V at rest, under its floor of 7.00 V (3.50 V a cell): charge \
             it before anything drives"
        );
        let torque = c.read(id, control::TORQUE_ENABLE.addr, 1).unwrap();
        assert_eq!(torque, [0]);
        let (mut c, id) = fake(3555);
        before_drive(&mut c, id).expect("a charged pack drives");
    }

    /// The in-process servo on the bench board's sense chain, its rail
    /// ADC reading `vbus_raw`.
    fn fake(vbus_raw: u16) -> (Client<FakePipe>, Id) {
        let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        pipe.set_track(
            0,
            vec![TelSample {
                pos: 2029,
                current: 0,
                current_trough: 0,
                duty_q15: 0,
                vdiff: 0,
                vbus: 0,
                current_raw: 0,
                vmotor_a: 0,
                vmotor_b: 0,
                vbus_raw,
                ntc_raw: 0,
                pos_lin_q4: 0,
                window_valid: false,
                fault: false,
            }],
        );
        (Client::connect(pipe).unwrap(), Id::new(1))
    }
}

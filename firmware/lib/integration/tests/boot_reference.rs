//! The chip's 1.2 V reference is read once at boot, not in the scan: the
//! crest scan lands the rail right behind the pot, `vref_raw` sits outside
//! the tick block, and no tick writes it.

use osc_integration::mg90::{self, Mg90};
use osc_integration::mg90_d;
use osc_integration::plant::FakeIo;
use osc_servo_core::kernel::{DECIM_MED, DECIM_SLOW};
use osc_servo_core::{ControlTable, Kernel, RegionStorage, Shared};

/// `(addr, width)` of the field the descriptor names `name`.
fn field(name: &str) -> (usize, usize) {
    let d = ControlTable::FIELDS
        .iter()
        .find(|d| d.name == name)
        .unwrap_or_else(|| panic!("no field {name}"));
    (d.addr as usize, d.width as usize)
}

fn tick(k: &mut Kernel<FakeIo>, sh: &Shared, p: &mut Mg90) {
    let f = p.step(k.io.motor.last);
    k.on_scan(f, sh);
}

/// The scan is `[high, low, shunt, pos, rail, NTC]` and the DMA lands it
/// in slot order, so the three published slots are adjacent; the boot
/// reading has no slot among them and no place in the block a tick owns.
#[test]
fn the_scan_lands_the_rail_behind_the_pot_and_no_reference() {
    let (pos, _) = field("pos");
    let (vbus, _) = field("vbus_raw");
    let (ntc, _) = field("ntc_raw");
    let (vref, width) = field("vref_raw");
    let (stamp, stamp_width) = field("sample_tick");
    assert_eq!(width, 2);
    assert_eq!(
        [vbus, ntc],
        [pos + 2, pos + 4],
        "the crest scan's slots behind pos {pos:#x}"
    );
    assert!(
        !(pos..stamp + stamp_width).contains(&vref),
        "vref_raw at {vref:#x} lies in the tick block {pos:#x}..{:#x}",
        stamp + stamp_width
    );
}

/// A boot reading stays as boot left it through every medium phase and
/// both SLOW stages, torque off and driven.
#[test]
fn no_tick_writes_the_boot_reference_reading() {
    const BOOT: u16 = 1492;
    let (addr, _) = field("vref_raw");
    let sh = mg90_d::rig(|t| t.control.lifecycle.torque_enable = false);
    let read = |sh: &Shared| {
        sh.table.with(|t| {
            // SAFETY: a table address is its byte offset, inside the table.
            unsafe {
                (&raw const *t)
                    .cast::<u8>()
                    .add(addr)
                    .cast::<u16>()
                    .read_unaligned()
            }
        })
    };
    sh.table.with_mut(|t| {
        // SAFETY: as `read`.
        unsafe {
            (&raw mut *t)
                .cast::<u8>()
                .add(addr)
                .cast::<u16>()
                .write_unaligned(BOOT)
        }
    });
    let mut p = mg90_d::plant(2048.0, 1);
    let mut k = mg90::kernel();
    let ticks = 2 * DECIM_SLOW as u32 * DECIM_MED as u32 + 2 * DECIM_MED as u32;
    for n in 0..ticks {
        tick(&mut k, &sh, &mut p);
        assert_eq!(read(&sh), BOOT, "torque off, tick {n}");
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_position = 2248;
        t.control.lifecycle.torque_enable = true;
    });
    for n in 0..ticks {
        tick(&mut k, &sh, &mut p);
        assert_eq!(read(&sh), BOOT, "driven, tick {n}");
    }
    assert_eq!(sh.table.with(|t| t.telemetry.common.fault_flags), 0);
}

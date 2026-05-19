use core::cell::SyncUnsafeCell;
use osc_core::{Kernel, Shared};

use crate::board::Ch32Board;

/// Board sensors sampled once per scan, in `Sensors` field order: pos, ntc,
/// vbus, vmotor.0, vmotor.1, enc.0, enc.1.
pub const ADC_SENSOR_COUNT: usize = 7;

/// Scan layout: `[shunt, pos, ntc, vbus, vmotor.0, vmotor.1, enc.0, enc.1,
/// vref]` = 9 slots. Shunt is a single sample at the scan head — `peak`
/// gives ON-window current, `trough` gives freewheel current. Vref tail is
/// chip-lib-owned and used to scale Vdd-relative raw counts.
pub const ADC_SCAN_LEN: usize = 1 + ADC_SENSOR_COUNT + 1;

/// DMA buffer holds two scans per PWM period (peak + trough sampling under
/// center-aligned PWM with RCR=0).
pub const ADC_DMA_BUF_LEN: usize = ADC_SCAN_LEN * 2;

pub static ADC_DMA_BUF: SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]> =
    SyncUnsafeCell::new([0; ADC_DMA_BUF_LEN]);

pub static SHARED: Shared = Shared::const_new();

/// Installed via `install_kernel` after `Ch32Board::new` returns. The DMA1
/// TC ISR reads this; `None` until install, so early interrupts are ignored.
pub(crate) static KERNEL: SyncUnsafeCell<Option<Kernel<Ch32Board>>> = SyncUnsafeCell::new(None);

/// Move the board into a `Kernel` and install it. Called once from `main`
/// after `Ch32Board::new` finishes hardware bring-up.
pub fn install_kernel(board: Ch32Board) {
    unsafe {
        *KERNEL.get() = Some(Kernel::new(board));
    }
}

pub fn read_sample_tick() -> u32 {
    unsafe { (*SHARED.table.telemetry.get()).intermediaries.sample_tick }
}

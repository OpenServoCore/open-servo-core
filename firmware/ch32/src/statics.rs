use core::cell::SyncUnsafeCell;
use osc_core::{Kernel, Shared};

use crate::board::Ch32Board;

/// In `Sensors` field order: pos, ntc, vbus, vmotor.0, vmotor.1, enc.0, enc.1.
pub const ADC_SENSOR_COUNT: usize = 7;

/// `[shunt, pos, ntc, vbus, vmotor.0, vmotor.1, enc.0, enc.1, vref]` = 9 slots.
pub const ADC_SCAN_LEN: usize = 1 + ADC_SENSOR_COUNT + 1;

/// Two scans per PWM period (peak + trough under center-aligned PWM, RCR=0).
pub const ADC_DMA_BUF_LEN: usize = ADC_SCAN_LEN * 2;

pub static ADC_DMA_BUF: SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]> =
    SyncUnsafeCell::new([0; ADC_DMA_BUF_LEN]);

pub static SHARED: Shared = Shared::const_new();

/// `None` until `install_kernel` runs — DMA1 TC ISR ignores ticks while None.
pub(crate) static KERNEL: SyncUnsafeCell<Option<Kernel<Ch32Board>>> = SyncUnsafeCell::new(None);

pub fn install_kernel(board: Ch32Board) {
    unsafe {
        *KERNEL.get() = Some(Kernel::new(board));
    }
}

pub fn read_sample_tick() -> u32 {
    unsafe { (*SHARED.table.telemetry.get()).intermediaries.sample_tick }
}

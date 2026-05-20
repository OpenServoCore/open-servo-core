use core::cell::SyncUnsafeCell;
use osc_core::{Kernel, Shared};

use crate::board::Ch32Board;

/// In `Sensors` field order: pos, ntc, vbus, vmotor.0, vmotor.1.
pub const ADC_SENSOR_COUNT: usize = 5;

/// Scan = `[IN9/OpaOut, IN7/PD4/pos, IN2/PC4/ntc,
///          IN5/PD5/vmA, IN6/PD6/vmB, IN10/Vcal]`. IN1 (PA1/vbus) excluded.
pub const ADC_SCAN_LEN: usize = 6;

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
    crate::log::info!("kernel installed; DMA TC ISR live");
}

/// `read_volatile` is load-bearing: a plain read gets hoisted out of spin loops
/// because LLVM can't see the DMA TC ISR writing this asynchronously.
pub fn read_sample_tick() -> u32 {
    unsafe {
        let p = &raw const (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        p.read_volatile()
    }
}

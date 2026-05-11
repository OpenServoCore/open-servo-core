use core::cell::SyncUnsafeCell;
use osc_core::{Kernel, Shared};

use crate::board::Ch32Board;

/// 13 channels × 2 halves (peak + trough) per PWM period.
pub const ADC_DMA_BUF_LEN: usize = 26;

pub static ADC_DMA_BUF: SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]> =
    SyncUnsafeCell::new([0; ADC_DMA_BUF_LEN]);

pub static SHARED: Shared = Shared::const_new();

/// Installed via `install_kernel` after `Ch32Board::new` returns. The DMA1
/// TC ISR reads this; `None` until install, so early interrupts are ignored.
pub(crate) static KERNEL: SyncUnsafeCell<Option<Kernel<Ch32Board>>> =
    SyncUnsafeCell::new(None);

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

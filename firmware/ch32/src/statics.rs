use core::cell::SyncUnsafeCell;
use osc_core::Shared;

/// 13 channels × 2 halves (peak + trough) per PWM period.
pub const ADC_DMA_BUF_LEN: usize = 26;

pub static ADC_DMA_BUF: SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]> =
    SyncUnsafeCell::new([0; ADC_DMA_BUF_LEN]);

pub static SHARED: Shared = Shared::const_new();

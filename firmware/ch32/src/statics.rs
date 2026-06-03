use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use osc_core::{Kernel, Services, Shared};

use crate::board::Ch32KernelIo;
use crate::hal::pfic;
use crate::services::Ch32ServicesIo;

/// In `AdcPins` field order: pos, ntc, vbus, vmotor.0, vmotor.1.
pub const ADC_SENSOR_COUNT: usize = 5;

/// Scan = `[IN9/OpaOut, IN7/PD4/pos, IN2/PC4/ntc,
///          IN5/PD5/vmA, IN6/PD6/vmB, IN10/Vcal]`. IN1 (PA1/vbus) excluded.
pub const ADC_SCAN_LEN: usize = 6;

/// Two scans per PWM period (peak + trough under center-aligned PWM, RCR=0).
pub const ADC_DMA_BUF_LEN: usize = ADC_SCAN_LEN * 2;

pub static ADC_DMA_BUF: SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]> =
    SyncUnsafeCell::new([0; ADC_DMA_BUF_LEN]);

pub static SHARED: Shared = Shared::new();

/// Initialised by `install`; DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Kernel<Ch32KernelIo>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// Initialised by `install`; sole `&mut` writer is the main loop.
pub(crate) static SERVICES: SyncUnsafeCell<MaybeUninit<Services<Ch32ServicesIo>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

pub fn install(io: Ch32KernelIo) {
    unsafe {
        (*KERNEL.get()).write(Kernel::new(io));
        (*SERVICES.get()).write(Services::new(Ch32ServicesIo::new()));
    }
    crate::log::info!("kernel + services installed");
}

pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    pfic::enable_systick();
    crate::log::info!("ISRs live");
}

/// `read_volatile` is load-bearing: a plain read gets hoisted out of spin loops
/// because LLVM can't see the DMA TC ISR writing this asynchronously.
pub fn read_sample_tick() -> u32 {
    unsafe {
        let p = &raw const (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        p.read_volatile()
    }
}

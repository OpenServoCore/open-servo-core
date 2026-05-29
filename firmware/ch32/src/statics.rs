use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::Ordering;
use heapless::Vec;
use osc_core::{Kernel, Services, Shared};
use portable_atomic::{AtomicBool, AtomicU16, AtomicU32};

use crate::board::{Ch32KernelIo, TxEn};
use crate::hal::{Pin, pfic, systick};
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

pub const DXL_RX_BUF_LEN: usize = 512;

pub static DXL_RX_BUF: SyncUnsafeCell<[u8; DXL_RX_BUF_LEN]> =
    SyncUnsafeCell::new([0; DXL_RX_BUF_LEN]);

/// USART1 IDLE handler stores the DMA write index; `Ch32Bus::received` reads it.
pub static DXL_RX_WRITE_POS: AtomicU16 = AtomicU16::new(0);

pub const DXL_TX_BUF_LEN: usize = 256;

pub static DXL_TX_BUF: SyncUnsafeCell<Vec<u8, DXL_TX_BUF_LEN>> = SyncUnsafeCell::new(Vec::new());

/// Written once during `bring_up_dxl` before USART1 IRQ is unmasked; read-only thereafter.
pub static DXL_TX_EN: SyncUnsafeCell<Option<TxEn>> = SyncUnsafeCell::new(None);

/// Scope debug pin — bench instrumentation for the chain-CRC ISRs. Yanked
/// out of `kernel.io.dbg` so the dxl_fast hot path can reach it without
/// going through KERNEL. Seeded once at bring-up; ISR readers only.
pub static DXL_DBG_PIN: SyncUnsafeCell<Option<Pin>> = SyncUnsafeCell::new(None);

/// Set by `Ch32Device::reboot`; USART1 TC ISR fires the soft reset after TX drains.
pub static DXL_REBOOT_PENDING: AtomicBool = AtomicBool::new(false);

/// Pending USART1 BRR change requested by a control-table BAUD write. 0 = none.
/// TC ISR consumes after clearing DXL_TX_BUF so the Status reply ships at the
/// old wire rate (host can still decode), then the next byte is at the new rate.
pub static DXL_BAUD_PENDING_BRR: AtomicU32 = AtomicU32::new(0);

/// HCLK ticks for 9 bit-times on the wire (brr × 9); on_usart1_idle
/// backdates the request end tick by this to recover the moment the
/// last byte's stop bit completed (IDLE counts the stop bit as bit 1
/// of the 10 it needs, so it asserts 9 bit-times later).
pub static DXL_CHAR_TIME_TICKS: AtomicU32 = AtomicU32::new(0);

pub static DXL_BYTE_TIME_TICKS: AtomicU32 = AtomicU32::new(0);

/// `(1 << 16) / byte_time_us` — precomputed reciprocal so the snoop ISR
/// converts µs → bytes with a multiply + shift instead of a runtime divide
/// (RV32EC + zmmul has no hardware divide).
pub static DXL_BYTES_PER_US_Q16: AtomicU32 = AtomicU32::new(0);

/// Called at bring-up and after a BAUD_RATE write — never on the snoop hot
/// path, so the runtime division is fine here.
pub fn store_baud_derived(brr: u32) {
    DXL_CHAR_TIME_TICKS.store(brr.wrapping_mul(9), Ordering::Relaxed);
    let byte_time_ticks = brr.wrapping_mul(10);
    DXL_BYTE_TIME_TICKS.store(byte_time_ticks, Ordering::Relaxed);
    // Round-to-nearest on the reciprocal so the ISR's product floors at
    // the true completed-byte count; plain truncation drifts ~1 byte
    // low on long snoops.
    let bytes_per_us_q16 = ((systick::TICKS_PER_US << 16) + byte_time_ticks / 2)
        .checked_div(byte_time_ticks)
        .unwrap_or(0);
    DXL_BYTES_PER_US_Q16.store(bytes_per_us_q16, Ordering::Relaxed);
}

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
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL5, pfic::Priority::High);
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL5);
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

use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use heapless::Vec;
use osc_core::{Kernel, Shared};
use portable_atomic::AtomicU16;

use crate::board::{Ch32Board, TxEn};
use crate::hal::pfic;
use crate::ring_reader::RingReader;

/// In `Sensors` field order: pos, ntc, vbus, vmotor.0, vmotor.1.
pub const ADC_SENSOR_COUNT: usize = 5;

/// Scan = `[IN9/OpaOut, IN7/PD4/pos, IN2/PC4/ntc,
///          IN5/PD5/vmA, IN6/PD6/vmB, IN10/Vcal]`. IN1 (PA1/vbus) excluded.
pub const ADC_SCAN_LEN: usize = 6;

/// Two scans per PWM period (peak + trough under center-aligned PWM, RCR=0).
pub const ADC_DMA_BUF_LEN: usize = ADC_SCAN_LEN * 2;

pub static ADC_DMA_BUF: SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]> =
    SyncUnsafeCell::new([0; ADC_DMA_BUF_LEN]);

pub const DXL_RX_BUF_LEN: usize = 256;

pub static DXL_RX_BUF: SyncUnsafeCell<[u8; DXL_RX_BUF_LEN]> =
    SyncUnsafeCell::new([0; DXL_RX_BUF_LEN]);

/// USART1 IDLE handler stores the DMA write index; `dxl_handler::poll` reads it.
pub static DXL_RX_WRITE_POS: AtomicU16 = AtomicU16::new(0);

pub const DXL_TX_BUF_LEN: usize = 256;

pub static DXL_TX_BUF: SyncUnsafeCell<Vec<u8, DXL_TX_BUF_LEN>> = SyncUnsafeCell::new(Vec::new());

/// Written once during `bring_up_dxl` before USART1 IRQ is unmasked; read-only thereafter.
pub static DXL_TX_EN: SyncUnsafeCell<Option<TxEn>> = SyncUnsafeCell::new(None);

/// Sole writer + reader: `dxl_handler::poll` on the main loop. Sized to
/// DXL_RX_BUF so a full ring of unread bytes can land contiguously.
pub static DXL_RX_READER: SyncUnsafeCell<RingReader<DXL_RX_BUF_LEN>> =
    SyncUnsafeCell::new(RingReader::new());

pub static SHARED: Shared = Shared::const_new();

/// Initialised by `install_kernel`; DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Kernel<Ch32Board>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

pub fn install_kernel(board: Ch32Board) {
    unsafe {
        (*KERNEL.get()).write(Kernel::new(board));
    }
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
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

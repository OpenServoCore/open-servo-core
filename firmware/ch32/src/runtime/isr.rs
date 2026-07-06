use ch32_metapac::{DMA1, USART1};
use osc_core::{ControlIo, ConversionVariables, RegionStorageRaw, Sensors};

use crate::hal::{pfic, systick, usart};
use crate::runtime::Drivers;
use crate::runtime::statics::{KERNEL, SESSION, SHARED};

/// Configures PFIC priorities and unmasks the transport + ADC IRQs. Called
/// once during bringup, after the drivers and statics are installed.
///
/// The transport vectors (USART1 for break/TC, SysTick for the framer
/// deadlines) share PFIC HIGH so all `&mut` access into the `ServoBus`
/// composite serializes. The ADC DMA channel sits at LOW. DMA1_CH5 (RX ring)
/// runs silent circular — no HT/TC IRQ — and CH4/CH3 raise none either.
/// Bound on the FE→DMA settle spin: the RXNE→ring DMA write is ~10 AHB
/// cycles; 64 spins is orders of magnitude past it while staying sub-µs.
const FE_DMA_SETTLE_SPINS: u32 = 64;

pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable_systick();
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    crate::log::info!("ISRs live");
}

/// ADC DMA TC handler body — wire into the vector table via [`crate::install_isrs!`].
pub fn on_adc_dma_tc() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Volatile pair: load-bearing against optimizer hoisting in the pump.
        let tick = &raw mut (*SHARED.table.region_ptr())
            .telemetry
            .intermediaries
            .sample_tick;
        tick.write_volatile(tick.read_volatile().wrapping_add(1));

        // SAFETY: PFIC unmasks DMA1_CHANNEL1 only after install_kernel writes KERNEL.
        let kernel = (*KERNEL.get()).assume_init_mut();
        let vars = ConversionVariables::snapshot(&SHARED);
        let sample = {
            let (sensors, _motor) = kernel.io.parts();
            sensors.sample(&vars)
        };
        kernel.on_tick(sample, &SHARED);
    }
}

/// USART1 vector — break detection (RX framing error) and TX arm completion.
///
/// SAFETY: the bus driver is installed before this vector unmasks, and USART1
/// shares PFIC HIGH with SysTick, so no concurrent `&mut` into the composite
/// is possible. Statement ordering is load-bearing: the break handoff runs
/// off the RX-error read, then the TC branch does release work first.
/// Bench forensics (temporary): event counters + diag mirror, read via wlink.
#[used]
pub static DBG: [core::sync::atomic::AtomicU32; 6] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 6];

fn dbg_bump(i: usize) {
    // No atomic RMW on rv32ec; single-writer per slot, load+store suffices.
    let o = core::sync::atomic::Ordering::Relaxed;
    DBG[i].store(DBG[i].load(o).wrapping_add(1), o);
}

pub fn on_usart1() {
    crate::log::trace!("usart1 isr");
    dbg_bump(0);
    if DBG[4].load(core::sync::atomic::Ordering::Relaxed) == 0 {
        let cur = 512 - crate::hal::dma::remaining(crate::hal::dma::Channel::CH5);
        DBG[4].store(0x1_0000 | cur as u32, core::sync::atomic::Ordering::Relaxed);
    }
    // (a) RX errors: an FE marks a break (or mid-frame garble) → the framer
    // anchors on the just-ringed 0x00 (F2: the DMA write beats the ISR).
    let errs = usart::rx_errors(USART1);
    if errs.fe || errs.ore || errs.pe || errs.ne {
        if errs.fe {
            // The ERR interrupt beats the DMA drain: at entry the break's 0x00
            // may still sit in DR (RXNE set), so the ring cursor hasn't counted
            // it yet — and the framer's anchor would land one byte early
            // (bench-observed: every header read as [stale, 00, ID, LEN] →
            // BadId). Wait, bounded, for the DMA to take the byte before the
            // driver samples the cursor.
            let mut settle = FE_DMA_SETTLE_SPINS;
            while usart::is_rxne(USART1) && settle > 0 {
                settle -= 1;
                core::hint::spin_loop();
            }
            // SAFETY: see fn doc.
            unsafe { Drivers::bus() }.on_break();
        }
        // SR-then-DR is the only V006 error clear. DMA already drained DR for
        // the ring byte, so this read cannot steal a payload byte (spike-
        // validated: NDTR unchanged across the DR read).
        usart::clear_rx_errors(USART1);
    }

    // (b) TC: an armed TX arm drained (shifter empty). TCIE gates arbitration
    // — the shared vector fans in RX-errors + TC, and a foreign source could
    // enter with a stale reset-value TC. Gate on TCIE so it can't walk into
    // on_tx_complete before the first reply is armed.
    if usart::is_tcie(USART1) && usart::is_tc(USART1) {
        usart::clear_tc(USART1);
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_tx_complete();
    }
}

/// SysTick CMP-match — one or more framer/chain/rescue deadlines are due.
/// CNTIF is cleared first: a final deadline body returns without re-arming
/// and a stale-but-latched CNTIF would re-fire the IRQ the moment we return.
/// The dispatcher is built per-call from the shared table + the session.
///
/// SAFETY: SysTick shares PFIC HIGH with USART1, so no concurrent `&mut` into
/// the composite (or the session) is possible.
pub fn on_systick() {
    crate::log::trace!("systick isr");
    dbg_bump(1);
    systick::clear_match();
    // SAFETY: see fn doc — SESSION is installed before this vector unmasks.
    let session = unsafe { (*SESSION.get()).assume_init_mut() };
    let mut dispatcher = session.dispatcher(&SHARED);
    // SAFETY: see fn doc.
    let bus = unsafe { Drivers::bus() };
    bus.on_deadline(&mut dispatcher);
    let d = bus.diag();
    DBG[2].store(d.crc_fail_count, core::sync::atomic::Ordering::Relaxed);
    DBG[3].store(d.framing_drop_count, core::sync::atomic::Ordering::Relaxed);
}

/// Wires osc-ch32 ISR bodies into the vector table via hand-rolled full-save
/// trampolines, NOT `#[qingke_rt::interrupt]`. The qingke-rt trampoline saves
/// only `ra` and relies on HPE hardware stacking, which on this silicon
/// corrupts the interrupted context's t0/t2 (bench-observed here as a
/// misaligned-load trap in the main loop right after `wfi`; first seen in the
/// break-framing spike). Full software caller-save (ra, t0–t2, a0–a5 — the
/// RV32E set) plus `INTSYSCR = 0` (set in `run!`: no HPE, no nesting) removes
/// the silicon variable entirely. Cost: ISRs no longer preempt each other —
/// fine while the kernel tick is a stub; revisit nesting with the control
/// loop.
#[macro_export]
macro_rules! install_isrs {
    () => {
        ::core::arch::global_asm!(
            r#"
            .macro ISR_TRAMPOLINE name, body
            .section .text.\name
            .global \name
            .align 2
        \name:
            addi sp, sp, -40
            sw ra,  0(sp)
            sw t0,  4(sp)
            sw t1,  8(sp)
            sw t2, 12(sp)
            sw a0, 16(sp)
            sw a1, 20(sp)
            sw a2, 24(sp)
            sw a3, 28(sp)
            sw a4, 32(sp)
            sw a5, 36(sp)
            call \body
            lw ra,  0(sp)
            lw t0,  4(sp)
            lw t1,  8(sp)
            lw t2, 12(sp)
            lw a0, 16(sp)
            lw a1, 20(sp)
            lw a2, 24(sp)
            lw a3, 28(sp)
            lw a4, 32(sp)
            lw a5, 36(sp)
            addi sp, sp, 40
            mret
            .endm

            ISR_TRAMPOLINE DMA1_CHANNEL1, __osc_isr_adc_dma
            ISR_TRAMPOLINE USART1, __osc_isr_usart1
            ISR_TRAMPOLINE SysTick, __osc_isr_systick
            "#
        );

        #[unsafe(no_mangle)]
        extern "C" fn __osc_isr_adc_dma() {
            $crate::runtime::isr::on_adc_dma_tc();
        }

        #[unsafe(no_mangle)]
        extern "C" fn __osc_isr_usart1() {
            $crate::runtime::isr::on_usart1();
        }

        #[unsafe(no_mangle)]
        extern "C" fn __osc_isr_systick() {
            $crate::runtime::isr::on_systick();
        }
    };
}

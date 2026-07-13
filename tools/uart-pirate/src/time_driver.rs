//! SysTick -> `embassy_time_driver::Driver` bridge.
//!
//! Vendored from `ch32-hal/src/embassy/time_driver_systick.rs` with the
//! `rcc::clocks()` lookup replaced by the constant HCLK we already
//! committed to in `rcc::init_144mhz_hse` (144 MHz). SysTick counts at
//! HCLK/8 = 18 MHz; `embassy_time_driver::TICK_HZ` defaults to 1_000_000,
//! so `cnt_per_tick = 18`.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use ch32_metapac::SYSTICK;
use ch32_metapac::systick::vals;
use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use qingke_rt::CoreInterrupt;

const HCLK_HZ: u64 = 144_000_000;

#[qingke_rt::interrupt(core)]
fn SysTick() {
    DRIVER.on_interrupt();
}

pub struct SystickDriver {
    cnt_per_tick: AtomicU32,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    cnt_per_tick: AtomicU32::new(1),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

impl SystickDriver {
    fn init_inner(&'static self, _cs: CriticalSection) {
        let cnt_per_second = HCLK_HZ / 8;
        let cnt_per_tick = cnt_per_second / embassy_time_driver::TICK_HZ;
        self.cnt_per_tick
            .store(cnt_per_tick as u32, Ordering::Relaxed);

        SYSTICK.ctlr().write(|w| {
            w.set_init(true);
            w.set_ste(true);
        });
        SYSTICK.cmph().write_value(0);
        SYSTICK.cmpl().write_value(0);
        SYSTICK.sr().write(|w| w.set_cntif(false));
        SYSTICK.ctlr().modify(|w| {
            w.set_mode(vals::Mode::UPCOUNT);
            w.set_stre(false);
            w.set_stclk(vals::Stclk::HCLK_DIV8);
        });
    }

    fn on_interrupt(&self) {
        SYSTICK.sr().write(|w| w.set_cntif(false));
        critical_section::with(|cs| self.trigger_alarm(cs));
    }

    #[inline]
    fn raw_cnt(&self) -> u64 {
        SYSTICK.cnt().read()
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self
            .queue
            .borrow(cs)
            .borrow_mut()
            .next_expiration(self.raw_cnt());
        while !self.set_alarm(cs, next) {
            next = self
                .queue
                .borrow(cs)
                .borrow_mut()
                .next_expiration(self.raw_cnt());
        }
    }

    fn set_alarm(&self, _cs: CriticalSection, next_alarm_cnt: u64) -> bool {
        if next_alarm_cnt <= self.raw_cnt() {
            return false;
        }
        SYSTICK.cmph().write_value((next_alarm_cnt >> 32) as u32);
        SYSTICK.cmpl().write_value(next_alarm_cnt as u32);
        SYSTICK.ctlr().modify(|w| w.set_stie(true));
        SYSTICK.sr().write(|w| w.set_cntif(false));
        if next_alarm_cnt <= self.raw_cnt() {
            SYSTICK.ctlr().modify(|w| w.set_stie(false));
            SYSTICK.sr().write(|w| w.set_cntif(false));
            return false;
        }
        true
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        let cnt_per_tick = self.cnt_per_tick.load(Ordering::Relaxed) as u64;
        self.raw_cnt() / cnt_per_tick
    }

    fn schedule_wake(&self, ticks: u64, waker: &core::task::Waker) {
        let cnt_per_tick = self.cnt_per_tick.load(Ordering::Relaxed) as u64;
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            if queue.schedule_wake(ticks * cnt_per_tick, waker) {
                let mut next = queue.next_expiration(self.raw_cnt());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.raw_cnt());
                }
            }
        })
    }
}

pub fn init() {
    critical_section::with(|cs| DRIVER.init_inner(cs));
    unsafe {
        // P15 = lowest preempt priority in V4's 4-bit IPRIOR scheme;
        // matches ch32-hal's choice. SysTick gets the lowest urgency
        // because it only services time-queue alarms.
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, 0xF0);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}

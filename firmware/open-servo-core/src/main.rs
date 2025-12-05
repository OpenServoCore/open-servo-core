#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

mod hw;
mod hw_stm32;

use hw::Hw; // Import the Hw trait
use hw_stm32::StmHw; // Import the STM32 implementation

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use pid::Pid;
use stm32f3::stm32f301 as pac;

use pac::interrupt;

#[derive(Copy, Clone, defmt::Format)]
struct SystemState {
    // Desired Servo Position
    pub setpoint: u16,

    // Current Servo Position
    pub position: u16,

    // PWM Duty Cycle
    pub pwm_duty: i32,

    // Current in mA
    pub current: f32,

    // VDDA Voltage
    pub vdda: f32,

    // Chip Temperature in Celsius
    pub temperature: f32,
}

impl SystemState {
    fn new() -> Self {
        SystemState {
            setpoint: 0,
            position: 0,
            pwm_duty: 0,
            current: 0.0,
            vdda: 0.0,
            temperature: 0.0,
        }
    }
}

// PWM parameters - these constants are still used by PID configuration
const PWM_MAX_DUTY: u16 = 1799; // Matches hw_stm32::init::tim::PWM_MAX_DUTY
const PWM_REVERSE: bool = false;

// PID controller parameters
const PID_KP: f32 = 40.0;
const PID_KI: f32 = 0.0;
const PID_KD: f32 = 0.0;
const PID_KP_MAX: f32 = PWM_MAX_DUTY as f32;
const PID_KI_MAX: f32 = PWM_MAX_DUTY as f32 * 0.8;
const PID_KD_MAX: f32 = PWM_MAX_DUTY as f32 * 0.8;
const PID_OUTPUT_MAX: f32 = PWM_MAX_DUTY as f32;

static PID_CONTROLLER: Mutex<RefCell<Option<Pid<f32>>>> = Mutex::new(RefCell::new(Option::None));
static SYSTEM_STATE: Mutex<RefCell<Option<SystemState>>> = Mutex::new(RefCell::new(Option::None));
static HW: Mutex<RefCell<Option<StmHw>>> = Mutex::new(RefCell::new(Option::None));

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    // Use init functions from hw_stm32 module
    hw_stm32::init_rcc(&p);
    hw_stm32::init_tim1_pwm(&p);
    hw_stm32::init_tim2_counter(&p);
    hw_stm32::init_gpio(&p); // This initializes LED on PA3
    hw_stm32::init_dma(&p);
    hw_stm32::init_adc(&p);

    // intialize PID controller
    let mut pid: Pid<f32> = Pid::new(0.0, PID_OUTPUT_MAX);
    pid.p(PID_KP, PID_KP_MAX);
    pid.i(PID_KI, PID_KI_MAX);
    pid.d(PID_KD, PID_KD_MAX);
    free(|cs| PID_CONTROLLER.borrow(cs).replace(Some(pid)));

    // initialize system state
    let system_state = SystemState::new();
    free(|cs| SYSTEM_STATE.borrow(cs).replace(Some(system_state)));

    // initialize hardware abstraction
    let hw = StmHw::new();
    free(|cs| HW.borrow(cs).replace(Some(hw)));

    // enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2);
        pac::NVIC::unmask(pac::Interrupt::DMA1_CH1);
    }

    defmt::info!("Initialized");

    // start ADC conversion
    p.ADC1.cr.modify(|_, w| w.adstart().start_conversion());

    loop {}
}

// interrupt handler for TIM2
#[interrupt]
fn TIM2() {
    defmt::info!("TIM2 interrupt");

    // toggle PA3
    unsafe {
        (*pac::GPIOA::ptr())
            .odr
            .modify(|r, w| w.odr3().bit(!r.odr3().bit()));
    }

    // clone system state to local variable
    let system_state = free(|cs| SYSTEM_STATE.borrow(cs).borrow().unwrap());
    defmt::info!("{}", system_state);

    // clear interrupt flag
    unsafe {
        (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear());
    }
}

#[interrupt]
fn DMA1_CH1() {
    let dma1 = unsafe { &(*pac::DMA1::ptr()) };
    let [is_complete, is_error] = cortex_m::interrupt::free(|_| {
        [
            dma1.isr.read().tcif1().is_complete(),
            dma1.isr.read().teif1().is_error(),
        ]
    });

    // check interrupt flag for transfer complete
    if !is_complete {
        defmt::info!("Transfer complete interrupt flag not set");
        return;
    }

    // check interrupt flag for transfer error
    if is_error {
        defmt::error!("Transfer error interrupt flag set");
        return;
    }

    // Use the Hw trait to read sensor values
    free(|cs| {
        let mut hw_option = HW.borrow(cs).borrow_mut();
        if let Some(hw) = hw_option.as_mut() {
            // Read sensors using the Hw trait
            let position = hw.position();
            let current = hw.phase_current();
            let bus_voltage = hw.bus_voltage();
            let chip_temp = hw.temperature();

            // hardcode setpoint for now
            let setpoint: u16 = 2048;

            // calculate pwm duty cycle using PID controller
            let pid_output = {
                let mut pid_option = PID_CONTROLLER.borrow(cs).borrow_mut();
                let pid = pid_option.as_mut().unwrap();
                pid.setpoint(setpoint).next_control_output(position as f32)
            };

            let mut duty: i32 = pid_output.output as i32;
            if PWM_REVERSE {
                duty = duty * -1;
            }

            // Set PWM using the Hw trait
            hw.set_pwm(duty);

            // Update system state
            let mut system_state_options = SYSTEM_STATE.borrow(cs).borrow_mut();
            let system_state = system_state_options.as_mut().unwrap();

            system_state.setpoint = setpoint;
            system_state.position = position;
            system_state.pwm_duty = duty;
            system_state.vdda = bus_voltage as f32 / 1000.0; // Convert from mV to V
            system_state.current = current as f32;
            if let Some(temp_dk) = chip_temp {
                system_state.temperature = (temp_dk as f32 / 10.0) - 273.15; // Convert from decikelvin to Celsius
            }
        }
    });

    // clear interrupt flag
    unsafe {
        (*pac::DMA1::ptr()).ifcr.write(|w| w.cgif1().set_bit());
    }
}

// Conversion functions moved to hw_stm32::hw_impl
// The Hw trait now handles all sensor conversions internally

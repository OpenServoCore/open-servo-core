//! Two-phase hardware initialization.
//!
//! Phase 1 (configure_*): Set up peripherals but don't start them.
//! Phase 2 (start_*): Enable peripherals in controlled order.
//!
//! This separation ensures:
//! - All configuration is done before any peripheral starts
//! - Interrupt handlers have valid state when enabled
//! - Deterministic startup sequence

pub mod adc;
pub mod gpio;
pub mod nvic;
pub mod rcc;
pub mod tim;
pub mod uart;

pub use adc::{configure_adc, start_adc_dma};
pub use gpio::configure_gpio;
pub use nvic::{configure_nvic, enable_interrupts};
pub use rcc::configure_rcc;
pub use tim::{configure_tim1, configure_tim2, start_tim1, start_tim2};
pub use uart::{configure_usart, start_usart, start_usart_rx_dma};

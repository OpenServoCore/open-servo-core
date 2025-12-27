#![no_std]
//! # open-servo-board-stm32f301
//!
//! New executor-based board crate for STM32F301.
//!
//! Architecture:
//! - Single-writer kernel: only ADC DMA ISR calls kernel.tick() / apply_op()
//! - Two-phase init: configure_* then start_* in controlled order
//! - Free-running TIM2 monotonic (1µs resolution)
//! - Critical-section guarded SPSC queues (no atomics)

pub mod adc_config;
pub mod calibration;
pub mod config;
pub mod init;
pub mod isr;
pub mod monotonic;
pub mod pwm;
pub mod resources;
pub mod sensors;
pub mod sinks;
pub mod uart_bus;

pub use config::kernel_config;

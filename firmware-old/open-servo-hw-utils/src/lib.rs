#![no_std]

pub mod adc_dma;

// When async is enabled, use rtt_async which handles both defmt and shell channels.
// Otherwise fall back to rtt_debug for defmt-only setups.
#[cfg(all(feature = "rtt", not(feature = "async")))]
pub mod rtt_debug;

#[cfg(feature = "async")]
pub mod rtt_async;

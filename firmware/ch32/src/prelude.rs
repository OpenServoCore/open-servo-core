pub use osc_drivers::Level;

#[cfg(not(feature = "half-duplex"))]
pub use crate::cfg::BusWiring;
pub use crate::cfg::{
    AdcPins, AnalogChannel, BoardConfig, BoardWiring, Calibration, CurrentSenseConfig, DigitalPin,
    Divider, DrvEn, NtcCal,
};
pub use crate::hal::{Pin, opa};
pub use crate::{BaudRate, ConfigDefaults};
pub use osc_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;

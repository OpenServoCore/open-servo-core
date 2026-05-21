pub use crate::board::{
    BoardConfig, BoardWiring, Calibration, CurrentSenseConfig, Divider, Duplex, DxlBus,
    MotorConfig, NtcCal, Sensors, TxEn,
};
pub use crate::hal::{Pin, Tim1Mapping, Tim2Mapping, UsartMapping, adc, gpio, opa, timer};
pub use crate::{BaudRate, ConfigDefaults};

#![no_std]
#![feature(sync_unsafe_cell)]

pub mod debug;
pub mod estimator;
pub mod kernel;

/// Firmware version stamped into the identity block (protocol sec 5.4): this
/// crate's Cargo version packed 5.5.6, the number the bootloader reports too.
pub const FIRMWARE_VERSION: u16 = osc_protocol::version::pack_version(
    parse_decimal(env!("CARGO_PKG_VERSION_MAJOR")),
    parse_decimal(env!("CARGO_PKG_VERSION_MINOR")),
    parse_decimal(env!("CARGO_PKG_VERSION_PATCH")),
);

const _: () = assert!(
    parse_decimal(env!("CARGO_PKG_VERSION_MAJOR")) < 32
        && parse_decimal(env!("CARGO_PKG_VERSION_MINOR")) < 32
        && parse_decimal(env!("CARGO_PKG_VERSION_PATCH")) < 64,
    "Cargo version exceeds the 5.5.6 packing"
);

const fn parse_decimal(s: &str) -> u8 {
    let b = s.as_bytes();
    let mut i = 0;
    let mut v: u8 = 0;
    while i < b.len() {
        v = v * 10 + (b[i] - b'0');
        i += 1;
    }
    v
}

pub mod log;
pub mod math;
pub mod persist;
pub mod regions;
pub mod sensor_frame;
pub mod services;
pub mod shared;
pub mod tel;
pub mod traits;

pub use control_table::{
    Error, Region, RegionStorage, RegionStorageRaw, StagedWrites, ValidationKind,
};
pub use kernel::{Kernel, KernelTiming};
pub use persist::{ConfigStore, StoreError};
pub use regions::config::{BaudRate, ConfigDefaults, CurrentDefaults};
pub use regions::{
    BootMode, CalibKinematics, CalibMotor, CalibRegs, CalibSense, CalibSenseExt, CalibWinding,
    ConfigCommon, ConfigFaultCfg, ConfigFusion, ConfigLimits, ConfigLoopCurrent,
    ConfigLoopPosition, ConfigLoopVelocity, ConfigPosLimits, ConfigRegs, ConfigThermal,
    ControlLifecycle, ControlRegs, ControlSystem, ControlTable, ControlTableCell, DecaySelect,
    Mode, PotLutBlock, StallResponse, TelemetryCommon, TelemetryEstimates, TelemetryIdent,
    TelemetryMode, TelemetryRegs, TelemetrySensors,
};
pub use sensor_frame::SensorFrame;
pub use services::bus::{Dispatcher, Session};
pub use shared::Shared;
pub use tel::{TelSample, TelStream};
pub use traits::{
    Capabilities, ControlIo, DecayMode, Dispatch, Dispatched, Motor, MotorCmd, Reply, Request,
    RequestCtx, SendError, Sensors, Status,
};

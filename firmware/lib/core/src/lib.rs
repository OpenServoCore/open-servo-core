#![no_std]
#![feature(sync_unsafe_cell)]

pub mod debug;
pub mod kernel;

/// Firmware version stamped into the identity block (protocol sec 5.4). A
/// table-ABI counter, bumped when a client-visible table ABI change lands;
/// 0 is reserved for an unversioned dev build.
pub const FIRMWARE_VERSION: u8 = 1;

pub mod log;
pub mod persist;
pub mod regions;
pub mod sample;
pub mod services;
pub mod shared;
pub mod traits;

pub use control_table::{
    Error, Region, RegionStorage, RegionStorageRaw, StagedWrites, ValidationKind,
};
pub use kernel::{Kernel, KernelState};
pub use persist::{ConfigStore, StoreError};
pub use regions::config::{BaudRate, ConfigDefaults};
pub use regions::{
    BemfCalibBlock, BootMode, CalibRegs, ConfigCalibration, ConfigCommon, ConfigControlPosition,
    ConfigPosLimits, ConfigRegs, ConfigStall, ConfigThermal, ControlLifecycle, ControlRegs,
    ControlStreaming, ControlSystem, ControlTable, ControlTableCell, Mode, PotLutBlock,
    StallResponse, TelemetryCommon, TelemetryConverted, TelemetryIntermediaries, TelemetryMode,
    TelemetryRaw, TelemetryRegs,
};
pub use sample::{ConversionVariables, RawSamples, Sample};
pub use services::bus::{Dispatcher, Session};
pub use shared::Shared;
pub use traits::{
    Capabilities, ControlIo, DecayMode, Dispatch, Dispatched, Motor, MotorCmd, Reply, Request,
    RequestCtx, SendError, Sensors, Status,
};

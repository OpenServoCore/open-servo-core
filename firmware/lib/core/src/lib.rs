#![no_std]
#![feature(sync_unsafe_cell)]

pub mod debug;
pub mod kernel;
pub mod log;
pub mod page;
pub mod regions;
pub mod sample;
pub mod services;
pub mod shared;
pub mod traits;

pub use control_table::{
    Error, Region, RegionStorage, RegionStorageRaw, StagedWrites, ValidationKind,
};
pub use kernel::{Kernel, KernelState};
pub use page::{PageHeader, PageMagic};
pub use regions::config::{BaudRate, ConfigDefaults};
pub use regions::{
    BemfCalibBlock, BootMode, CalibRegs, ConfigCalibration, ConfigComms, ConfigControlPosition,
    ConfigIdentity, ConfigPosLimits, ConfigRegs, ConfigStall, ConfigThermal, ControlLifecycle,
    ControlRegs, ControlStreaming, ControlSystem, ControlTable, ControlTableCell, Mode,
    PotLutBlock, StallResponse, TelemetryBusLink, TelemetryConverted, TelemetryFault,
    TelemetryIntermediaries, TelemetryRaw, TelemetryRegs,
};
pub use sample::{ConversionVariables, RawSamples, Sample};
pub use services::bus::{Dispatcher, Session};
pub use shared::Shared;
pub use traits::{
    Capabilities, ControlIo, DecayMode, Dispatch, Motor, MotorCmd, Reply, Request, RequestCtx,
    SendError, Sensors, Speculated, Status,
};

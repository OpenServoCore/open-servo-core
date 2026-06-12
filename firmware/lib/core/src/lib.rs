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
    Error, Region, RegionStorage, Router, StagedView, StagedWrites, ValidationKind,
};
pub use kernel::{Kernel, KernelState};
pub use page::{PageHeader, PageMagic};
pub use regions::config::{BaudRate, ConfigDefaults};
pub use regions::{
    BemfCalibBlock, BootMode, CalibRegs, ConfigCalibration, ConfigComms, ConfigControlPosition,
    ConfigIdentity, ConfigPosLimits, ConfigRegs, ConfigStall, ConfigThermal, ControlLifecycle,
    ControlRegs, ControlStreaming, ControlSystem, ControlTable, Mode, PotLutBlock, StallResponse,
    StatusReturnLevel, TelemetryConverted, TelemetryFault, TelemetryIntermediaries, TelemetryRaw,
    TelemetryRegs,
};
pub use sample::{ConversionVariables, RawSamples, Sample};
pub use services::Services;
pub use services::dxl::Dxl;
pub use shared::Shared;
pub use traits::{Capabilities, ControlIo, DecayMode, DxlBus, DxlReply, Motor, MotorCmd, Sensors};

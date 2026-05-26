#![no_std]
#![feature(sync_unsafe_cell)]

pub mod debug;
pub mod kernel;
pub mod kernel_io;
pub mod log;
pub mod page;
pub mod regions;
pub mod ring_reader;
pub mod sample_frame;
pub mod services;
pub mod shared;
pub mod traits;

pub use kernel_io::{Capabilities, ConfigDefaults, DecayMode, KernelIo, Motor, MotorCmd, Sensors};
pub use control_table::{
    Error, Region, RegionStorage, Router, StagedView, StagedWrites, ValidationKind,
};
pub use kernel::{Kernel, KernelState};
pub use page::{PageHeader, PageMagic};
pub use regions::config::BaudRate;
pub use regions::{
    BemfCalibBlock, BootMode, CalibRegs, ConfigCalibration, ConfigComms, ConfigControlPosition,
    ConfigIdentity, ConfigPosLimits, ConfigRegs, ConfigStall, ConfigThermal, ControlLifecycle,
    ControlRegs, ControlStreaming, ControlSystem, ControlTable, Mode, PotLutBlock, StallResponse,
    StatusReturnLevel, TelemetryConverted, TelemetryFault, TelemetryIntermediaries, TelemetryRaw,
    TelemetryRegs,
};
pub use ring_reader::{RingReader, RxSnapshot};
pub use sample_frame::{FrameInputs, RawSamples, SampleFrame};
pub use services::dxl::{DeviceControl, Dxl, DxlBus, ServicesIo};
pub use services::Services;
pub use shared::Shared;

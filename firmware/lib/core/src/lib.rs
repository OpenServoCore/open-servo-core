#![no_std]
#![feature(sync_unsafe_cell)]

pub mod board;
pub mod debug;
pub mod kernel;
pub mod log;
pub mod page;
pub mod regions;
pub mod ring_reader;
pub mod sample_frame;
pub mod services;
pub mod shared;

pub use board::{Board, Capabilities, ConfigDefaults, DecayMode, MotorCmd};
pub use control_table::{
    Access, BOOL_ALLOWED, BlockDesc, BlockValidator, CompareOp, Error, FieldDesc, FieldValidator,
    HasAllowed, Region, RegionDesc, RegionStorage, RegionStorageRaw, RegionValidator, Rhs, Router,
    StagedView, StagedWrites, ValidationKind,
};
pub use kernel::{Kernel, KernelState};
pub use page::{PageHeader, PageMagic};
pub use regions::config::BaudRate;
pub use regions::{
    BemfCalibBlock, CalibRegs, ConfigCalibration, ConfigComms, ConfigControlPosition,
    ConfigIdentity, ConfigPosLimits, ConfigRegs, ConfigStall, ConfigThermal, ControlLifecycle,
    ControlRegs, ControlStreaming, ControlTable, Mode, PotLutBlock, StallResponse,
    TelemetryConverted, TelemetryFault, TelemetryIntermediaries, TelemetryRaw, TelemetryRegs,
};
pub use ring_reader::{RingReader, RxSnapshot};
pub use sample_frame::{FrameInputs, RawSamples, SampleFrame};
pub use services::dxl::{Dxl, DxlIo};
pub use services::{Services, ServicesIo};
pub use shared::Shared;

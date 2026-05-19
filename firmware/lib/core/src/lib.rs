#![no_std]
#![feature(sync_unsafe_cell)]

pub mod board;
pub mod kernel;
pub mod page;
pub mod regions;
pub mod regmap;
pub mod sample_frame;
pub mod services;
pub mod shared;
pub mod stream_coord;

pub use board::{Board, Capabilities, ConfigDefaults, DecayMode, MotorCmd};
pub use kernel::{Kernel, KernelState};
pub use page::{PageHeader, PageMagic};
pub use regions::{
    BemfCalibBlock, CalibRegs, ConfigComms, ConfigControl, ConfigControlPosition, ConfigIdentity,
    ConfigLimits, ConfigPosLimits, ConfigRegs, ConfigSafety, ConfigStall, ConfigThermal,
    ControlLifecycle, ControlRegs, ControlStreaming, ControlTable, PotLutBlock, TelemetryConverted,
    TelemetryFault, TelemetryIntermediaries, TelemetryRaw, TelemetryRegs,
};
pub use regmap::{Access, BlockDesc, RegmapError};
pub use sample_frame::{FrameInputs, RawSamples, SampleFrame};
pub use services::Services;
pub use shared::Shared;
pub use stream_coord::StreamCoord;

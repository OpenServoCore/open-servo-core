#![no_std]
#![feature(sync_unsafe_cell)]

pub mod board;
pub mod debug;
pub mod kernel;
pub mod log;
pub mod page;
pub mod regions;
pub mod regmap;
pub mod ring_reader;
pub mod sample_frame;
pub mod services;
pub mod shared;

pub use board::{Board, Capabilities, ConfigDefaults, DecayMode, MotorCmd};
pub use kernel::{Kernel, KernelState};
pub use page::{PageHeader, PageMagic};
pub use regions::config::BaudRate;
pub use regions::{
    BemfCalibBlock, CalibRegs, ConfigComms, ConfigControl, ConfigControlPosition, ConfigIdentity,
    ConfigLimits, ConfigPosLimits, ConfigRegs, ConfigSafety, ConfigStall, ConfigThermal,
    ControlLifecycle, ControlRegs, ControlStreaming, ControlTable, PotLutBlock, TelemetryConverted,
    TelemetryFault, TelemetryIntermediaries, TelemetryRaw, TelemetryRegs,
};
pub use regmap::{Access, BlockDesc, RegmapError};
pub use ring_reader::{RingReader, RxSnapshot};
pub use sample_frame::{FrameInputs, RawSamples, SampleFrame};
pub use services::dxl::{Dxl, DxlIo};
pub use services::{Services, ServicesIo};
pub use shared::Shared;

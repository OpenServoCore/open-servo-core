//! Common register block (sec 5.4): the model-agnostic register set every
//! node carries at fixed addresses. Everything outside these two blocks is
//! per-model ABI and never named here.

/// CONFIG-COMMON span (sec 5.4): SAVE-persisted; identity RO, comms RW.
pub const CONFIG_COMMON_START: u16 = 0x000;
/// Exclusive end; `CONFIG_COMMON_END..` is model-specific config space.
pub const CONFIG_COMMON_END: u16 = 0x020;

/// u16 RO -- keys the per-model register map.
pub const MODEL_NUMBER: u16 = 0x000;
/// u8 RO.
pub const FIRMWARE_VERSION: u16 = 0x002;
/// u8 RO.
pub const HARDWARE_REVISION: u16 = 0x003;
/// u32 RO -- reserved-slot extension signal; no bits defined yet.
pub const CAPABILITY_FLAGS: u16 = 0x004;
/// u8 RW -- unicast address, 0x01..=0xF9 (sec 3.1).
pub const ID: u16 = 0x010;
/// u8 RW -- sec 2 rate index (`BaudRate` discriminant).
pub const BAUD_RATE_IDX: u16 = 0x011;
/// u16 RW -- sec 7 reclaim window.
pub const RESPONSE_DEADLINE_US: u16 = 0x012;

/// TELEMETRY-COMMON span (sec 5.4): volatile.
pub const TELEMETRY_COMMON_START: u16 = 0x200;
/// Exclusive end; `TELEMETRY_COMMON_END..` is model-specific telemetry space.
pub const TELEMETRY_COMMON_END: u16 = 0x220;

/// u8 RO -- the sec 5.3 alarm register, ALERT's read target.
pub const FAULT_FLAGS: u16 = 0x200;
/// u8 RO -- common node state bits; see `STATUS_FLAG_CONFIG_DIRTY`.
pub const STATUS_FLAGS: u16 = 0x201;
/// i8 RO -- applied clock-trim total in chip trim steps (sec 9.3).
pub const TRIM_STEPS: u16 = 0x202;
/// u32 RW -- sec 5.3 frame-level counter; hosts write 0 to clear.
pub const CRC_FAIL_COUNT: u16 = 0x204;
/// u32 RW -- sec 5.3 frame-level counter; hosts write 0 to clear.
pub const FRAMING_DROP_COUNT: u16 = 0x208;

/// `STATUS_FLAGS` bit 0: modified-since-save (sec 9.4).
pub const STATUS_FLAG_CONFIG_DIRTY: u8 = 1 << 0;

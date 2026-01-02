//! Open Servo RPC Protocol
//!
//! Shared message types for host/device communication over RTT.
//! This crate is `no_std` compatible for use in firmware.

#![no_std]

use heapless::Vec;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

// ============================================================================
// Common Response Types
// ============================================================================

/// Common error type for all RPC operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub enum RpcError {
    /// Address out of range or unknown.
    InvalidAddress,
    /// Register is read-only.
    ReadOnly,
    /// Register is locked (e.g., EEPROM while torque enabled).
    Locked,
    /// Data length mismatch.
    InvalidLength,
    /// Requested stream would exceed buffer capacity.
    TooManyBytes,
}

/// Unified response wrapper - either Ok with data, or Err.
#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
pub enum RpcResp<T> {
    Ok(T),
    Err(RpcError),
}

impl<T> RpcResp<T> {
    /// Create a successful response.
    pub fn ok(value: T) -> Self {
        RpcResp::Ok(value)
    }

    /// Create an error response.
    pub fn err(error: RpcError) -> Self {
        RpcResp::Err(error)
    }

    /// Check if response is Ok.
    pub fn is_ok(&self) -> bool {
        matches!(self, RpcResp::Ok(_))
    }

    /// Check if response is Err.
    pub fn is_err(&self) -> bool {
        matches!(self, RpcResp::Err(_))
    }
}

// ============================================================================
// Registry Operations
// ============================================================================

/// Read register request.
#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
pub struct ReadRegReq {
    /// Register address (0-1023).
    pub addr: u16,
    /// Number of bytes to read (1-4).
    pub len: u8,
}

/// Write register request.
#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
pub struct WriteRegReq {
    /// Register address (0-1023).
    pub addr: u16,
    /// Data to write (up to 4 bytes).
    pub data: Vec<u8, 4>,
}

/// Start register streaming request.
#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
pub struct RegStreamStartReq {
    /// Streaming rate in Hz.
    pub rate_hz: u8,
    /// Addresses to stream.
    pub addresses: Vec<u16, 16>,
}

/// Stop register streaming request.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Schema)]
pub struct RegStreamStopReq;

/// Streaming register data frame (device → host).
#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
pub struct RegStreamFrame {
    /// Sequence number (wraps at 65535).
    pub seq: u16,
    /// Packed register values in address order.
    pub data: Vec<u8, 64>,
}

// ============================================================================
// System Operations
// ============================================================================

/// Device info response.
#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
pub struct DeviceInfo {
    /// Firmware version (major, minor, patch).
    pub version: (u8, u8, u8),
    /// Device ID (Dynamixel ID).
    pub device_id: u8,
    /// Model number.
    pub model_number: u16,
    /// Motor type (0 = BDC, 1 = BLDC).
    pub motor_type: u8,
}

// ============================================================================
// FNV-1a Key Generation (for endpoint dispatch)
// ============================================================================

/// FNV-1a hash for endpoint keys.
pub const fn fnv1a_hash(data: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf29ce484222325;
    let mut i = 0;
    while i < data.len() {
        hash ^= data[i] as u64;
        hash = hash.wrapping_mul(0x100000001b3);
        i += 1;
    }
    hash
}

// Endpoint keys (pre-computed FNV-1a hashes)
pub const KEY_REG_READ: u64 = fnv1a_hash(b"reg/read");
pub const KEY_REG_WRITE: u64 = fnv1a_hash(b"reg/write");
pub const KEY_REG_STREAM_START: u64 = fnv1a_hash(b"reg/stream/start");
pub const KEY_REG_STREAM_STOP: u64 = fnv1a_hash(b"reg/stream/stop");
pub const KEY_REG_DATA: u64 = fnv1a_hash(b"reg/data");
pub const KEY_SYS_PING: u64 = fnv1a_hash(b"sys/ping");
pub const KEY_SYS_INFO: u64 = fnv1a_hash(b"sys/info");

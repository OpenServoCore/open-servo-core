//! Typed views over the common register block (protocol sec 5.4) -- the
//! only registers a model-agnostic client may name. One READ per view;
//! layout comes from the `osc_protocol::table` consts, which the servo's
//! pin test holds to the wire.

use osc_protocol::table;
use osc_protocol::wire::Id;

use crate::client::Client;
use crate::error::{Error, LinkError};
use crate::pipe::Pipe;

/// CONFIG-COMMON identity front (RO).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identity {
    pub model: u16,
    pub fw: u8,
    pub hw: u8,
    pub capabilities: u32,
}

/// TELEMETRY-COMMON front: alarm, dirty, applied trim, transport counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Health {
    /// The sec 5.3 alarm register -- nonzero is what sets ALERT.
    pub fault_flags: u8,
    /// Modified-since-save (sec 9.4).
    pub config_dirty: bool,
    pub trim_steps: i8,
    pub crc_fail_count: u32,
    pub framing_drop_count: u32,
}

pub async fn identity<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Identity, Error> {
    let b = c.read(id, table::MODEL_NUMBER, 8).await?;
    if b.len() < 8 {
        return Err(short("identity"));
    }
    Ok(Identity {
        model: u16::from_le_bytes([b[0], b[1]]),
        fw: b[2],
        hw: b[3],
        capabilities: u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
    })
}

pub async fn health<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Health, Error> {
    let b = c.read(id, table::FAULT_FLAGS, 12).await?;
    if b.len() < 12 {
        return Err(short("health"));
    }
    Ok(Health {
        fault_flags: b[0],
        config_dirty: b[1] & table::STATUS_FLAG_CONFIG_DIRTY != 0,
        trim_steps: b[2] as i8,
        crc_fail_count: u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
        framing_drop_count: u32::from_le_bytes([b[8], b[9], b[10], b[11]]),
    })
}

/// Zero both transport counters -- contiguous by the sec 5.4 layout, so a
/// single 8-byte write clears them atomically.
pub async fn clear_counters<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<(), Error> {
    c.write(id, table::CRC_FAIL_COUNT, &[0u8; 8]).await
}

fn short(what: &str) -> Error {
    Error::Link(LinkError::Desync(format!("short {what} payload")))
}

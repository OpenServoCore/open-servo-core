//! DXL-family drivers. One sibling module per wire — today only
//! [`uart`]; a future CAN-FD variant would land as a sibling `can`
//! module without rearranging the parent.

pub mod uart;

/// DXL 2.0 spec factory-default `RETURN_DELAY_TIME` (register encoding,
/// 2 µs units → 250 µs). Two consumers:
///
/// 1. Chip-app seeds `ConfigDefaults.dxl_return_delay_2us` from this so
///    the canonical value lives next to the protocol driver, not in
///    chip-side boilerplate.
/// 2. Broadcast-Ping deadline math reads this directly instead of the
///    per-instance register value. The per-id `k × frame_bytes` spacer
///    already owns collision avoidance, so a self-rdt term on top would
///    let a low-id, high-rdt servo extend into the next id's slot —
///    using a uniform driver default keeps every on-bus servo aligned
///    regardless of any RDT override (including legacy on-bus chips
///    needing extra turnaround).
pub const DEFAULT_RDT_2US: u8 = 125;

/// [`DEFAULT_RDT_2US`] decoded from register units to the µs value the
/// deadline math consumes.
pub const DEFAULT_RDT_US: u32 = DEFAULT_RDT_2US as u32 * 2;

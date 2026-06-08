//! Chip-side DXL codec alias. Pre-binds the chip's `CrcUmts` impl and the
//! OSC extension markers so call sites stay short — replaces the `Codec`
//! convenience that lived in `dxl-protocol` itself.

use dxl_protocol::{Packet, Slot, SlotPosition, Status, WriteBuf, WriteError};
use osc_core::{OscExt, OscReplyExt};

use super::Ch32DxlCrc;

/// Generic-bound facade — every method just forwards to the corresponding
/// top-level `dxl_protocol` free fn with `<Ch32DxlCrc, OscExt, OscReplyExt>`
/// already plugged in.
pub struct DxlWire;

impl DxlWire {
    #[allow(dead_code)] // master-mode use; not exercised by slave path yet
    pub fn write_packet<W: WriteBuf>(
        out: &mut W,
        packet: &Packet<'_, OscExt>,
    ) -> Result<(), WriteError> {
        dxl_protocol::write_packet::<W, Ch32DxlCrc, OscExt>(out, packet)
    }

    pub fn write_status<W: WriteBuf>(
        out: &mut W,
        status: &Status<'_, OscReplyExt>,
    ) -> Result<(), WriteError> {
        dxl_protocol::write_status::<W, Ch32DxlCrc, OscReplyExt>(out, status)
    }

    pub fn write_slot<W: WriteBuf>(
        out: &mut W,
        slot: &Slot<'_>,
        position: SlotPosition,
    ) -> Result<(), WriteError> {
        dxl_protocol::write_slot::<W, Ch32DxlCrc>(out, slot, position)
    }
}

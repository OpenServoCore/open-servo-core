//! osc-native wire primitives: ID, packed `INST` byte, and frame span math
//! (`docs/osc-native-protocol.md` §3, §5, §9). Layout only — no buffering.

/// Frame ID byte. `0x01..=0xF9` unicast, `0xFE` broadcast; `0x00`/`0xFF` and
/// `0xFA..=0xFD` never address a servo on the wire (§3.1).
#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Id(pub u8);

impl Id {
    pub const BROADCAST: Self = Self(0xFE);

    #[inline]
    pub const fn new(b: u8) -> Self {
        Self(b)
    }

    /// Validated unicast constructor; accepts only `0x01..=0xF9`.
    #[inline]
    pub const fn try_unicast(b: u8) -> Option<Self> {
        match b {
            0x01..=0xF9 => Some(Self(b)),
            _ => None,
        }
    }

    #[inline]
    pub const fn as_byte(self) -> u8 {
        self.0
    }

    #[inline]
    pub const fn is_broadcast(self) -> bool {
        self.0 == 0xFE
    }

    #[inline]
    pub const fn is_unicast(self) -> bool {
        matches!(self.0, 0x01..=0xF9)
    }

    #[inline]
    pub const fn is_valid(self) -> bool {
        self.is_unicast() || self.is_broadcast()
    }

    /// Servo-side "is this frame for me": true when the frame ID (`self`) is
    /// broadcast or equals the servo's own `other`.
    #[inline]
    pub const fn addresses(self, other: Id) -> bool {
        self.is_broadcast() || self.0 == other.0
    }
}

/// Instruction opcode, `INST` bits [6:4] (§5). `0x0` is invalid.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Opcode {
    Ping = 0x1,
    Read = 0x2,
    Write = 0x3,
    Commit = 0x4,
    Gread = 0x5,
    Gwrite = 0x6,
    Mgmt = 0x7,
}

impl Opcode {
    /// `b` is the already-extracted 3-bit field; `0x0` and `>0x7` reject.
    #[inline]
    pub const fn from_bits(b: u8) -> Option<Opcode> {
        match b {
            0x1 => Some(Opcode::Ping),
            0x2 => Some(Opcode::Read),
            0x3 => Some(Opcode::Write),
            0x4 => Some(Opcode::Commit),
            0x5 => Some(Opcode::Gread),
            0x6 => Some(Opcode::Gwrite),
            0x7 => Some(Opcode::Mgmt),
            _ => None,
        }
    }
}

/// Status result code, `INST` bits [6:2] (§5.3). `9..=31` reserved/invalid.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ResultCode {
    Ok = 0,
    Instruction = 1,
    Range = 2,
    Access = 3,
    Validation = 4,
    Busy = 5,
    Limit = 6,
    PredecessorSilent = 7,
    Hardware = 8,
}

impl ResultCode {
    /// `b` is the already-extracted 5-bit field; `9..=31` reject.
    #[inline]
    pub const fn from_bits(b: u8) -> Option<ResultCode> {
        match b {
            0 => Some(ResultCode::Ok),
            1 => Some(ResultCode::Instruction),
            2 => Some(ResultCode::Range),
            3 => Some(ResultCode::Access),
            4 => Some(ResultCode::Validation),
            5 => Some(ResultCode::Busy),
            6 => Some(ResultCode::Limit),
            7 => Some(ResultCode::PredecessorSilent),
            8 => Some(ResultCode::Hardware),
            _ => None,
        }
    }
}

/// MGMT sub-op, payload byte 0 (§9).
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MgmtOp {
    Enum = 0x01,
    Assign = 0x02,
    Save = 0x03,
    Reboot = 0x04,
    Factory = 0x05,
}

impl MgmtOp {
    #[inline]
    pub const fn from_byte(b: u8) -> Option<MgmtOp> {
        match b {
            0x01 => Some(MgmtOp::Enum),
            0x02 => Some(MgmtOp::Assign),
            0x03 => Some(MgmtOp::Save),
            0x04 => Some(MgmtOp::Reboot),
            0x05 => Some(MgmtOp::Factory),
            _ => None,
        }
    }
}

/// Packed `INST` byte. Bit 7 selects the layout: instruction (opcode [6:4] +
/// flags [3:0]) or status (result [6:2] + PAD bit 1 + ALERT bit 0). Every bit
/// pattern is representable; validity comes from the typed accessors.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Inst(pub u8);

impl Inst {
    pub const FLAG_HOLD: u8 = 1 << 0;
    pub const FLAG_PAD: u8 = 1 << 1;
    pub const FLAG_NOREPLY: u8 = 1 << 2;
    pub const FLAG_PER_TARGET: u8 = 1 << 3;

    const STATUS_BIT: u8 = 0x80;
    const ALERT_BIT: u8 = 1 << 0;

    #[inline]
    pub const fn instruction(op: Opcode, flags: u8) -> Self {
        Self(((op as u8) << 4) | (flags & 0x0F))
    }

    #[inline]
    pub const fn status(result: ResultCode, pad: bool, alert: bool) -> Self {
        let mut b = Self::STATUS_BIT | ((result as u8) << 2);
        if pad {
            b |= Self::FLAG_PAD;
        }
        if alert {
            b |= Self::ALERT_BIT;
        }
        Self(b)
    }

    #[inline]
    pub const fn is_status(self) -> bool {
        self.0 & Self::STATUS_BIT != 0
    }

    /// PAD occupies bit 1 in both layouts (§3.1).
    #[inline]
    pub const fn pad(self) -> bool {
        self.0 & Self::FLAG_PAD != 0
    }

    /// Opcode of an instruction frame; `None` for status frames or opcode `0`.
    #[inline]
    pub const fn opcode(self) -> Option<Opcode> {
        if self.is_status() {
            return None;
        }
        Opcode::from_bits((self.0 >> 4) & 0x07)
    }

    #[inline]
    pub const fn hold(self) -> bool {
        self.0 & Self::FLAG_HOLD != 0
    }

    #[inline]
    pub const fn noreply(self) -> bool {
        self.0 & Self::FLAG_NOREPLY != 0
    }

    #[inline]
    pub const fn per_target(self) -> bool {
        self.0 & Self::FLAG_PER_TARGET != 0
    }

    /// Result code of a status frame; `None` for instruction frames or a
    /// reserved code.
    #[inline]
    pub const fn result(self) -> Option<ResultCode> {
        if !self.is_status() {
            return None;
        }
        ResultCode::from_bits((self.0 >> 2) & 0x1F)
    }

    #[inline]
    pub const fn alert(self) -> bool {
        self.0 & Self::ALERT_BIT != 0
    }
}

/// Max payload bytes; sized so the largest frame fits whole in the ring (§3.1).
pub const MAX_PAYLOAD: u8 = 252;

/// Fixed CRC-covered prefix byte (§3.2).
pub const CRC_PREFIX: u8 = 0x00;

/// Pad iff the payload length is odd (§3.1) — an invariant, not an option.
#[inline]
pub const fn needs_pad(p: u8) -> bool {
    p & 1 == 1
}

/// `LEN` for a `p`-byte payload: `INST + payload + pad + CRC` = `3 + p + pad`.
/// Caller keeps `p <= MAX_PAYLOAD`.
#[inline]
pub const fn len_for(p: u8) -> u8 {
    3 + p + needs_pad(p) as u8
}

/// Payload length recovered from `LEN` and the PAD flag: `len - 3 - pad`.
#[inline]
pub const fn payload_len(len: u8, pad: bool) -> u8 {
    len - 3 - pad as u8
}

/// Ring bytes for a frame including the break byte: `3 + len` (max 258).
#[inline]
pub const fn footprint(len: u8) -> usize {
    3 + len as usize
}

/// CRC-covered bytes including the prefix: `1 + len`.
#[inline]
pub const fn covered_len(len: u8) -> usize {
    1 + len as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_classification() {
        assert!(Id::BROADCAST.is_broadcast());
        assert!(Id::BROADCAST.is_valid());
        assert!(!Id::BROADCAST.is_unicast());
        assert!(Id::new(0x01).is_unicast());
        assert!(Id::new(0xF9).is_unicast());
        assert!(!Id::new(0x00).is_valid());
        assert!(!Id::new(0xFF).is_valid());
        assert!(!Id::new(0xFA).is_valid());
    }

    #[test]
    fn id_try_unicast() {
        assert_eq!(Id::try_unicast(0x00), None);
        assert_eq!(Id::try_unicast(0x01), Some(Id::new(0x01)));
        assert_eq!(Id::try_unicast(0xF9), Some(Id::new(0xF9)));
        assert_eq!(Id::try_unicast(0xFA), None);
        assert_eq!(Id::try_unicast(0xFE), None);
    }

    #[test]
    fn id_addresses() {
        assert!(Id::BROADCAST.addresses(Id::new(0x05)));
        assert!(Id::new(0x05).addresses(Id::new(0x05)));
        assert!(!Id::new(0x05).addresses(Id::new(0x06)));
    }

    #[test]
    fn opcode_from_bits() {
        assert_eq!(Opcode::from_bits(0x0), None);
        assert_eq!(Opcode::from_bits(0x1), Some(Opcode::Ping));
        assert_eq!(Opcode::from_bits(0x7), Some(Opcode::Mgmt));
    }

    #[test]
    fn result_from_bits() {
        assert_eq!(ResultCode::from_bits(0), Some(ResultCode::Ok));
        assert_eq!(
            ResultCode::from_bits(7),
            Some(ResultCode::PredecessorSilent)
        );
        assert_eq!(ResultCode::from_bits(8), Some(ResultCode::Hardware));
        assert_eq!(ResultCode::from_bits(9), None);
        assert_eq!(ResultCode::from_bits(31), None);
    }

    #[test]
    fn mgmt_from_byte() {
        assert_eq!(MgmtOp::from_byte(0x00), None);
        assert_eq!(MgmtOp::from_byte(0x01), Some(MgmtOp::Enum));
        assert_eq!(MgmtOp::from_byte(0x05), Some(MgmtOp::Factory));
        assert_eq!(MgmtOp::from_byte(0x06), None);
    }

    #[test]
    fn inst_instruction_roundtrip() {
        let i = Inst::instruction(Opcode::Write, Inst::FLAG_HOLD | Inst::FLAG_NOREPLY);
        assert!(!i.is_status());
        assert_eq!(i.opcode(), Some(Opcode::Write));
        assert!(i.hold());
        assert!(i.noreply());
        assert!(!i.per_target());
        assert!(!i.pad());
        assert_eq!(i.result(), None);
    }

    #[test]
    fn inst_write_padded_matches_vector() {
        // Spec vector: WRITE id 2, INST byte 0x32 = opcode 0x3, PAD flag set.
        let i = Inst(0x32);
        assert_eq!(i.opcode(), Some(Opcode::Write));
        assert!(i.pad());
        assert!(!i.hold());
    }

    #[test]
    fn inst_status_roundtrip() {
        let s = Inst::status(ResultCode::Range, true, true);
        assert!(s.is_status());
        assert_eq!(s.result(), Some(ResultCode::Range));
        assert!(s.pad());
        assert!(s.alert());
        assert_eq!(s.opcode(), None);
    }

    #[test]
    fn inst_status_no_flags() {
        let s = Inst::status(ResultCode::Ok, false, false);
        assert_eq!(s.0, 0x80);
        assert!(!s.pad());
        assert!(!s.alert());
        assert_eq!(s.result(), Some(ResultCode::Ok));
    }

    #[test]
    fn span_math() {
        // Even payload: no pad.
        assert!(!needs_pad(2));
        assert_eq!(len_for(2), 5);
        assert_eq!(payload_len(5, false), 2);
        // Odd payload: pad.
        assert!(needs_pad(3));
        assert_eq!(len_for(3), 7);
        assert_eq!(payload_len(7, true), 3);
        // PING: empty payload.
        assert_eq!(len_for(0), 3);
        // Largest frame.
        assert_eq!(len_for(MAX_PAYLOAD), 255);
        assert_eq!(footprint(255), 258);
        assert_eq!(covered_len(255), 256);
        // LEN is always odd.
        assert_eq!(len_for(0) & 1, 1);
        assert_eq!(len_for(1) & 1, 1);
    }

    #[test]
    fn span_covered_matches_vectors() {
        // "00 05 07 30 80 01 2C 01" — WRITE id 5, p=4 payload, LEN 7.
        assert_eq!(len_for(4), 7);
        assert_eq!(covered_len(7), 8);
        assert_eq!(footprint(7), 10);
    }
}

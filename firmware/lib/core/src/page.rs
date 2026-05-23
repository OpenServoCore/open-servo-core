//! `crc32` is IEEE-32 over leading `used_size` body bytes.
//! `seq` is the monotonic per-region commit counter — A/B picks higher-seq valid page.

#[repr(u32)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PageMagic {
    Erased = u32::from_le_bytes([0xFF, 0xFF, 0xFF, 0xFF]),
    Config = u32::from_le_bytes(*b"OSCF"),
    CalibPotLut = u32::from_le_bytes(*b"OSCP"),
    CalibBemf = u32::from_le_bytes(*b"OSCB"),
}

impl PageMagic {
    pub const fn from_u32(v: u32) -> Option<Self> {
        const ERASED: u32 = PageMagic::Erased as u32;
        const CONFIG: u32 = PageMagic::Config as u32;
        const POTLUT: u32 = PageMagic::CalibPotLut as u32;
        const BEMF: u32 = PageMagic::CalibBemf as u32;
        match v {
            ERASED => Some(Self::Erased),
            CONFIG => Some(Self::Config),
            POTLUT => Some(Self::CalibPotLut),
            BEMF => Some(Self::CalibBemf),
            _ => None,
        }
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PageHeader {
    pub magic: u32,
    pub seq: u32,
    pub hw_rev_at_commit: u32,
    pub used_size: u16,
    /// Bit 0 = `page_active` (CONFIG A/B). Other bits reserved.
    pub flags: u8,
    pub _rsvd_align: u8,
    pub _rsvd: [u8; 12],
    pub crc32: u32,
}

#[allow(clippy::new_without_default)]
impl PageHeader {
    /// All 0xFF, mirroring fresh-erased flash; load treats this as "no valid data".
    pub const fn new() -> Self {
        Self {
            magic: PageMagic::Erased as u32,
            seq: 0xFFFF_FFFF,
            hw_rev_at_commit: 0xFFFF_FFFF,
            used_size: 0xFFFF,
            flags: 0xFF,
            _rsvd_align: 0xFF,
            _rsvd: [0xFF; 12],
            crc32: 0xFFFF_FFFF,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn page_header_is_32_bytes() {
        assert_eq!(size_of::<PageHeader>(), 32);
    }

    #[test]
    fn page_magic_round_trip() {
        for m in [
            PageMagic::Erased,
            PageMagic::Config,
            PageMagic::CalibPotLut,
            PageMagic::CalibBemf,
        ] {
            assert_eq!(PageMagic::from_u32(m as u32), Some(m));
        }
    }

    #[test]
    fn page_magic_rejects_unknown() {
        assert_eq!(PageMagic::from_u32(0), None);
        assert_eq!(PageMagic::from_u32(0xDEAD_BEEF), None);
        assert_eq!(PageMagic::from_u32(u32::from_le_bytes(*b"OSCX")), None);
    }

    #[test]
    fn new_matches_fresh_erased_flash() {
        let h = PageHeader::new();
        let bytes: [u8; 32] = unsafe { core::mem::transmute(h) };
        assert_eq!(bytes, [0xFFu8; 32]);
    }
}

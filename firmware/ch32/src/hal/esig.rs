//! ESIG electronic signature (RM ch. 19): factory-burned chip identity in
//! system ROM, readable at any width. Only the 96-bit UID is consumed here.

const UNIID_BASE: *const u32 = 0x1FFF_F7E8 as *const u32;

/// The factory UID in the protocol sec 9.2 wire shape: the 96-bit ESIG as 12
/// little-endian bytes (UNIID1's low byte first, then UNIID2, UNIID3 -- RM
/// sec 19.2), zero-padded to the fixed 16-byte field.
pub fn uid() -> [u8; 16] {
    let mut out = [0u8; 16];
    for w in 0..3 {
        // SAFETY: ESIG is an always-readable factory ROM region (RM sec 19.1).
        let v = unsafe { UNIID_BASE.add(w).read_volatile() };
        out[w * 4..w * 4 + 4].copy_from_slice(&v.to_le_bytes());
    }
    out
}

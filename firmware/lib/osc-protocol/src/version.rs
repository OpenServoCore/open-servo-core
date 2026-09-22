//! `firmware_version` encoding (sec 5.4): semver packed 5.5.6 into a u16,
//! the tinyboot-protocol layout, so the table register and the bootloader's
//! Info report are one number.

/// Pack `major.minor.patch` as `[major:5][minor:5][patch:6]`.
pub const fn pack_version(major: u8, minor: u8, patch: u8) -> u16 {
    ((major as u16) << 11) | ((minor as u16) << 6) | (patch as u16)
}

/// Split a packed version back into `(major, minor, patch)`.
pub const fn unpack_version(v: u16) -> (u8, u8, u8) {
    (
        (v >> 11) as u8 & 0x1F,
        (v >> 6) as u8 & 0x1F,
        v as u8 & 0x3F,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_unpack_round_trips() {
        assert_eq!(unpack_version(pack_version(1, 2, 3)), (1, 2, 3));
        assert_eq!(unpack_version(pack_version(31, 31, 63)), (31, 31, 63));
        assert_eq!(unpack_version(pack_version(0, 0, 0)), (0, 0, 0));
    }

    #[test]
    fn layout_is_five_five_six() {
        assert_eq!(pack_version(1, 2, 3), 0x0883);
        assert_eq!(pack_version(0, 1, 0), 0x0040);
        assert_eq!(pack_version(31, 31, 63), 0xFFFF);
    }
}

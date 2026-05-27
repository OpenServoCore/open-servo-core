pub fn brr_for(apb_hz: u32, bps: u32) -> Option<u32> {
    if bps < 9600 {
        return None;
    }
    let brr = apb_hz / bps;
    if !(16..=0xFFFF).contains(&brr) {
        return None;
    }
    Some(brr)
}

/// Empty rejection is load-bearing: callers would otherwise arm DMA with
/// NDTR=0, which is undefined on CH32 DMA peripherals.
pub fn decode_hex(s: &str, out: &mut [u8]) -> Option<usize> {
    let bytes = s.as_bytes();
    if bytes.is_empty() || !bytes.len().is_multiple_of(2) {
        return None;
    }
    let n = bytes.len() / 2;
    if n > out.len() {
        return None;
    }
    for i in 0..n {
        out[i] = (nib(bytes[2 * i])? << 4) | nib(bytes[2 * i + 1])?;
    }
    Some(n)
}

fn nib(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(10 + c - b'a'),
        b'A'..=b'F' => Some(10 + c - b'A'),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const APB1_HZ: u32 = 144_000_000;

    #[test]
    fn brr_for_standard_dxl_rates_are_exact() {
        assert_eq!(brr_for(APB1_HZ, 1_000_000), Some(144));
        assert_eq!(brr_for(APB1_HZ, 3_000_000), Some(48));
        assert_eq!(brr_for(APB1_HZ, 57_600), Some(2500));
        assert_eq!(brr_for(APB1_HZ, 9600), Some(15_000));
    }

    #[test]
    fn brr_for_rejects_below_min_baud() {
        assert_eq!(brr_for(APB1_HZ, 9599), None);
        assert_eq!(brr_for(APB1_HZ, 0), None);
    }

    #[test]
    fn brr_for_rejects_below_min_brr() {
        assert!(brr_for(APB1_HZ, 9_000_001).is_none());
        assert_eq!(brr_for(APB1_HZ, 9_000_000), Some(16));
    }

    #[test]
    fn brr_for_rejects_above_max_brr() {
        assert_eq!(brr_for(1_000_000, 9600), Some(104));
        assert_eq!(brr_for(1_000_000_000, 9600), None);
    }

    #[test]
    fn decode_hex_rejects_empty() {
        let mut out = [0u8; 16];
        assert_eq!(decode_hex("", &mut out), None);
    }

    #[test]
    fn decode_hex_rejects_odd_length() {
        let mut out = [0u8; 16];
        assert_eq!(decode_hex("a", &mut out), None);
        assert_eq!(decode_hex("abc", &mut out), None);
    }

    #[test]
    fn decode_hex_rejects_invalid_chars() {
        let mut out = [0u8; 16];
        assert_eq!(decode_hex("gg", &mut out), None);
        assert_eq!(decode_hex("0x", &mut out), None);
        assert_eq!(decode_hex(" 0", &mut out), None);
    }

    #[test]
    fn decode_hex_rejects_overflow() {
        let mut out = [0u8; 2];
        assert_eq!(decode_hex("aabbcc", &mut out), None);
    }

    #[test]
    fn decode_hex_accepts_mixed_case() {
        let mut out = [0u8; 4];
        assert_eq!(decode_hex("AaBbCcDd", &mut out), Some(4));
        assert_eq!(&out, &[0xAA, 0xBB, 0xCC, 0xDD]);
    }

    #[test]
    fn decode_hex_leaves_tail_untouched() {
        let mut out = [0xFFu8; 4];
        assert_eq!(decode_hex("1234", &mut out), Some(2));
        assert_eq!(&out, &[0x12, 0x34, 0xFF, 0xFF]);
    }
}

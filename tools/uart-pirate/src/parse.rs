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

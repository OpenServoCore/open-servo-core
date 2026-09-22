//! UID text form: 32 hex chars, most significant byte first - the order
//! `osc discover` prints and `osc assign` reads.

use osc_client::mgmt::Uid;
use osc_protocol::wire::UID_LEN;

pub fn to_hex(uid: &Uid) -> String {
    format!("{uid:?}")
}

pub fn parse(s: &str) -> Result<Uid, String> {
    let b = s.as_bytes();
    if b.len() != UID_LEN * 2 {
        return Err(format!("uid is {} hex chars, got {}", UID_LEN * 2, b.len()));
    }
    let mut uid = [0u8; UID_LEN];
    for (i, pair) in b.chunks(2).enumerate() {
        let byte = match (nibble(pair[0]), nibble(pair[1])) {
            (Some(hi), Some(lo)) => hi << 4 | lo,
            _ => return Err(format!("uid: not hex at char {}", i * 2)),
        };
        uid[UID_LEN - 1 - i] = byte;
    }
    Ok(Uid(uid))
}

fn nibble(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_round_trips_msb_first() {
        let mut raw = [0u8; UID_LEN];
        raw[0] = 0x01;
        raw[15] = 0xF0;
        let uid = Uid(raw);
        let s = to_hex(&uid);
        assert_eq!(s, "f0000000000000000000000000000001");
        assert_eq!(parse(&s), Ok(uid));
        assert_eq!(parse(&s.to_uppercase()), Ok(uid));
    }

    #[test]
    fn rejects_length_sign_and_junk() {
        assert!(parse("").is_err());
        assert!(parse(&"0".repeat(31)).is_err());
        assert!(parse(&"0".repeat(33)).is_err());
        assert!(parse(&format!("+f{}", "0".repeat(30))).is_err());
        assert!(parse(&format!("zz{}", "0".repeat(30))).is_err());
        assert!(parse(&format!("\u{e9}{}", "0".repeat(30))).is_err());
    }
}

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
    uid.copy_from_slice(&le_bytes(s, "uid")?);
    Ok(Uid(uid))
}

/// A tinyboot id as the device sends it (least significant byte first) in
/// the same text form: a servo's is its UID's last 24 chars.
pub fn boot_to_hex(id: &[u8]) -> String {
    id.iter().rev().map(|b| format!("{b:02x}")).collect()
}

pub fn boot_parse(s: &str) -> Result<Vec<u8>, String> {
    let n = s.len();
    if n == 0 || n % 2 != 0 || n > UID_LEN * 2 {
        return Err(format!(
            "boot uid is an even 2 to {} hex chars, got {n}",
            UID_LEN * 2
        ));
    }
    le_bytes(s, "boot uid")
}

/// An even run of hex pairs, most significant first, as bytes least
/// significant first.
fn le_bytes(s: &str, what: &str) -> Result<Vec<u8>, String> {
    let mut out = s
        .as_bytes()
        .chunks(2)
        .enumerate()
        .map(|(i, pair)| match (nibble(pair[0]), nibble(pair[1])) {
            (Some(hi), Some(lo)) => Ok(hi << 4 | lo),
            _ => Err(format!("{what}: not hex at char {}", i * 2)),
        })
        .collect::<Result<Vec<u8>, String>>()?;
    out.reverse();
    Ok(out)
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

    #[test]
    fn boot_uid_is_the_uid_tail() {
        let mut raw = [0u8; UID_LEN];
        raw[0] = 0x01;
        raw[11] = 0xAB;
        raw[15] = 0xF0;
        let full = to_hex(&Uid(raw));
        let boot = boot_to_hex(&raw[..12]);
        assert_eq!(boot, "ab0000000000000000000001");
        assert!(full.ends_with(&boot));
        assert_eq!(boot_parse(&boot), Ok(raw[..12].to_vec()));
        assert_eq!(boot_parse(&full), Ok(raw.to_vec()));
        for bad in ["", "abc", "zz", &"0".repeat(34)] {
            assert!(boot_parse(bad).is_err(), "{bad:?}");
        }
    }
}

//! JSON as Python's `json.dumps` writes it, byte for byte: the notebooks'
//! set files are rendered and hashed through it, so a set this crate writes
//! hashes the same in both tools.

use serde_json::Value;
use sha2::{Digest, Sha256};

/// `json.dumps(v)`: keys in their order, `", "` and `": "` between.
pub fn dumps(v: &Value) -> String {
    let mut out = String::new();
    write(&mut out, v, false);
    out
}

/// `json.dumps(v, sort_keys=True, separators=(",", ":"))`.
pub fn canonical(v: &Value) -> String {
    let mut out = String::new();
    write(&mut out, v, true);
    out
}

/// The notebooks' `config_hash`: sha256 of the canonical form, hex.
pub fn hash(v: &Value) -> String {
    hex(&Sha256::digest(canonical(v).as_bytes()))
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn write(out: &mut String, v: &Value, sorted: bool) {
    let (item, key) = if sorted { (",", ":") } else { (", ", ": ") };
    match v {
        Value::Null => out.push_str("null"),
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Number(n) => match (n.as_i64(), n.as_u64(), n.as_f64()) {
            (Some(i), _, _) => out.push_str(&i.to_string()),
            (None, Some(u), _) => out.push_str(&u.to_string()),
            (None, None, Some(x)) => out.push_str(&float(x)),
            (None, None, None) => out.push_str("null"),
        },
        Value::String(s) => string(out, s),
        Value::Array(a) => {
            out.push('[');
            for (k, x) in a.iter().enumerate() {
                if k > 0 {
                    out.push_str(item);
                }
                write(out, x, sorted);
            }
            out.push(']');
        }
        Value::Object(m) => {
            let mut keys: Vec<&String> = m.keys().collect();
            if sorted {
                keys.sort();
            }
            out.push('{');
            for (k, name) in keys.into_iter().enumerate() {
                if k > 0 {
                    out.push_str(item);
                }
                string(out, name);
                out.push_str(key);
                write(out, &m[name.as_str()], sorted);
            }
            out.push('}');
        }
    }
}

/// `ensure_ascii`: everything outside space to tilde is escaped.
fn string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            ' '..='~' => out.push(c),
            _ => {
                let mut units = [0u16; 2];
                for u in c.encode_utf16(&mut units) {
                    out.push_str(&format!("\\u{u:04x}"));
                }
            }
        }
    }
    out.push('"');
}

/// Python's `repr` of a float: the shortest digits that read back, fixed
/// from 1e-4 up to under 1e16, else an exponent of at least two digits.
pub fn float(x: f64) -> String {
    if x == 0.0 {
        return if x.is_sign_negative() { "-0.0" } else { "0.0" }.into();
    }
    let e = format!("{x:e}");
    let Some((mantissa, exp)) = e.split_once('e') else {
        return e;
    };
    let Ok(exp) = exp.parse::<i32>() else {
        return e;
    };
    let (sign, mantissa) = match mantissa.strip_prefix('-') {
        Some(m) => ("-", m),
        None => ("", mantissa),
    };
    let digits: String = mantissa.chars().filter(|c| *c != '.').collect();
    let n = digits.len() as i32;
    let point = exp + 1;
    let body = if -4 < point && point <= 16 {
        if point <= 0 {
            format!("0.{}{digits}", "0".repeat(-point as usize))
        } else if point >= n {
            format!("{digits}{}.0", "0".repeat((point - n) as usize))
        } else {
            let (int, frac) = digits.split_at(point as usize);
            format!("{int}.{frac}")
        }
    } else {
        let (lead, rest) = digits.split_at(1);
        let m = if rest.is_empty() {
            lead.to_string()
        } else {
            format!("{lead}.{rest}")
        };
        format!("{m}e{}{:02}", if exp < 0 { '-' } else { '+' }, exp.abs())
    };
    format!("{sign}{body}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floats_read_as_python_reprs_them() {
        for (x, py) in [
            (1135.013429521035, "1135.013429521035"),
            (400.0, "400.0"),
            (4.8, "4.8"),
            (308.0533, "308.0533"),
            (-185.0803793156189, "-185.0803793156189"),
            (0.0001, "0.0001"),
            (0.00001, "1e-05"),
            (1.5e-7, "1.5e-07"),
            (1e15, "1000000000000000.0"),
            (1e16, "1e+16"),
            (1.25e22, "1.25e+22"),
            (6.433118992563836e-09, "6.433118992563836e-09"),
            (-0.0, "-0.0"),
        ] {
            assert_eq!(float(x), py, "{x}");
        }
    }

    #[test]
    fn strings_escape_as_ensure_ascii_does() {
        let v = Value::String("a\"b\\c\nd\u{7f}\u{e9}\u{1f600}".into());
        assert_eq!(dumps(&v), r#""a\"b\\c\nd\u007f\u00e9\ud83d\ude00""#);
    }

    #[test]
    fn canonical_sorts_keys_and_drops_the_spaces() {
        let v: Value = serde_json::from_str(r#"{"b": [1, 2.5], "a": {"d": null, "c": true}}"#)
            .unwrap_or_default();
        assert_eq!(dumps(&v), r#"{"b": [1, 2.5], "a": {"d": null, "c": true}}"#);
        assert_eq!(canonical(&v), r#"{"a":{"c":true,"d":null},"b":[1,2.5]}"#);
    }
}

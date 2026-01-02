//! Debug shell service task.
//!
//! Provides an interactive text-based debug interface over RTT.
//! Uses ShadowStorage for register access and the registry for field lookup.
//!
//! ## Commands
//!
//! - `help`: Print available commands
//! - `list [prefix]`: List all fields (optional prefix filter)
//! - `read <name|0xADDR>`: Read register by name or hex address
//! - `write <name|0xADDR> <value>`: Write register (respects access control)
//! - `dump <0xADDR> <len>`: Raw hex dump of shadow bytes
//! - `status`: Print key telemetry fields

use core::fmt::Write as FmtWrite;

use embedded_io_async::{Read, Write};
use heapless::String;
use open_servo_device::shadow_storage::ShadowStorage;
use open_servo_registry::{vendor, Access, Encoding, RegSpec, VENDOR_FIELDS};

/// Line buffer capacity.
const LINE_BUF_CAP: usize = 128;

/// Output buffer capacity.
const OUT_BUF_CAP: usize = 256;

/// Run the debug shell.
///
/// Reads commands from IO, executes them against ShadowStorage, writes responses.
/// Generic over IO type and shadow table size.
///
/// This is not an embassy task - the board crate should wrap this in a task.
pub async fn run_debug_shell<IO, const N: usize>(mut io: IO, shadow: &'static ShadowStorage<N>)
where
    IO: Read + Write,
{
    let mut line_buf: String<LINE_BUF_CAP> = String::new();

    // Print banner
    let _ = write_str(&mut io, "\r\n=== Debug Shell ===\r\n> ").await;

    loop {
        let mut byte = [0u8; 1];
        if io.read(&mut byte).await.is_err() {
            continue;
        }

        match byte[0] {
            // Carriage return - ignore
            b'\r' => {}

            // Newline - execute command
            b'\n' => {
                let _ = write_str(&mut io, "\r\n").await;

                execute_command(&line_buf, shadow, &mut io).await;

                line_buf.clear();
                let _ = write_str(&mut io, "> ").await;
            }

            // Backspace (0x08 or 0x7F)
            0x08 | 0x7F => {
                if !line_buf.is_empty() {
                    line_buf.pop();
                    // Erase character on terminal: backspace, space, backspace
                    let _ = io.write_all(b"\x08 \x08").await;
                }
            }

            // Printable character
            c if c >= 0x20 && c < 0x7F => {
                if line_buf.push(c as char).is_ok() {
                    // Echo the character
                    let _ = io.write_all(&[c]).await;
                }
            }

            // Ignore other control characters
            _ => {}
        }
    }
}

/// Write a string to the IO.
async fn write_str<IO: Write>(io: &mut IO, s: &str) -> Result<(), IO::Error> {
    io.write_all(s.as_bytes()).await
}

/// Execute a command and stream output to IO.
async fn execute_command<IO: Write, const N: usize>(
    line: &str,
    shadow: &ShadowStorage<N>,
    io: &mut IO,
) {
    let line = line.trim();
    if line.is_empty() {
        return;
    }

    let mut parts = line.split_whitespace();
    let cmd = match parts.next() {
        Some(c) => c,
        None => return,
    };

    match cmd.to_ascii_lowercase().as_str() {
        "help" | "?" => cmd_help(io).await,
        "list" | "ls" => cmd_list(parts.next(), io).await,
        "read" | "r" => cmd_read(parts.next(), shadow, io).await,
        "write" | "w" => cmd_write(parts.next(), parts.next(), shadow, io).await,
        "dump" | "d" => cmd_dump(parts.next(), parts.next(), shadow, io).await,
        "status" | "st" => cmd_status(shadow, io).await,
        _ => {
            let mut buf: String<64> = String::new();
            let _ = write!(buf, "Unknown command: {}\r\n", cmd);
            let _ = io.write_all(buf.as_bytes()).await;
        }
    }
}

/// Help command.
async fn cmd_help<IO: Write>(io: &mut IO) {
    let _ = io
        .write_all(
            b"Commands:\r\n\
         help          - Show this help\r\n\
         list [prefix] - List fields\r\n\
         read <field>  - Read register\r\n\
         write <f> <v> - Write register\r\n\
         dump <addr> <len> - Hex dump\r\n\
         status        - Show status\r\n",
        )
        .await;
}

/// List command - list all vendor fields matching optional prefix (streaming).
async fn cmd_list<IO: Write>(prefix: Option<&str>, io: &mut IO) {
    let prefix = prefix.unwrap_or("");

    for spec in VENDOR_FIELDS {
        if starts_with_ignore_case(spec.name, prefix) {
            let access_ch = match spec.access {
                Access::RO => 'R',
                Access::WO => 'O',
                Access::RW => 'W',
                Access::RWE => 'E',
                Access::Reserved => '-',
            };
            let mut line: String<64> = String::new();
            let _ = write!(
                line,
                "{} 0x{:03X} {} {}\r\n",
                access_ch,
                spec.address,
                spec.encoding.len(),
                spec.name
            );
            let _ = io.write_all(line.as_bytes()).await;
        }
    }
}

/// Read command - read a register by name or address.
async fn cmd_read<IO: Write, const N: usize>(
    field: Option<&str>,
    shadow: &ShadowStorage<N>,
    io: &mut IO,
) {
    let field = match field {
        Some(f) => f,
        None => {
            let _ = io.write_all(b"Usage: read <name|0xADDR>\r\n").await;
            return;
        }
    };

    let spec = match lookup_field(field) {
        Some(s) => s,
        None => {
            let mut buf: String<64> = String::new();
            let _ = write!(buf, "Field not found: {}\r\n", field);
            let _ = io.write_all(buf.as_bytes()).await;
            return;
        }
    };

    // Read raw bytes
    let mut buf = [0u8; 4];
    let len = spec.encoding.len() as usize;
    if shadow.host_read(spec.address, &mut buf[..len]).is_err() {
        let _ = io.write_all(b"Read error\r\n").await;
        return;
    }

    // Format based on encoding
    let mut out: String<OUT_BUF_CAP> = String::new();
    let _ = write!(out, "{} = ", spec.name);
    format_value(&buf[..len], spec.encoding, &mut out);
    let _ = write!(out, "\r\n");
    let _ = io.write_all(out.as_bytes()).await;
}

/// Write command - write a register by name or address.
async fn cmd_write<IO: Write, const N: usize>(
    field: Option<&str>,
    value: Option<&str>,
    shadow: &ShadowStorage<N>,
    io: &mut IO,
) {
    let field = match field {
        Some(f) => f,
        None => {
            let _ = io.write_all(b"Usage: write <name|0xADDR> <value>\r\n").await;
            return;
        }
    };

    let value_str = match value {
        Some(v) => v,
        None => {
            let _ = io.write_all(b"Usage: write <name|0xADDR> <value>\r\n").await;
            return;
        }
    };

    let spec = match lookup_field(field) {
        Some(s) => s,
        None => {
            let mut buf: String<64> = String::new();
            let _ = write!(buf, "Field not found: {}\r\n", field);
            let _ = io.write_all(buf.as_bytes()).await;
            return;
        }
    };

    // Check access control
    match spec.access {
        Access::RO | Access::Reserved => {
            let mut buf: String<64> = String::new();
            let _ = write!(buf, "Error: {} is read-only\r\n", spec.name);
            let _ = io.write_all(buf.as_bytes()).await;
            return;
        }
        Access::RWE => {
            if is_torque_enabled(shadow) {
                let mut buf: String<64> = String::new();
                let _ = write!(buf, "Error: {} locked (torque enabled)\r\n", spec.name);
                let _ = io.write_all(buf.as_bytes()).await;
                return;
            }
        }
        Access::RW | Access::WO => {}
    }

    // Parse value
    let mut buf = [0u8; 4];
    let len = spec.encoding.len() as usize;
    if !parse_value(value_str, spec.encoding, &mut buf[..len]) {
        let mut out: String<64> = String::new();
        let _ = write!(out, "Invalid value: {}\r\n", value_str);
        let _ = io.write_all(out.as_bytes()).await;
        return;
    }

    // Write to shadow
    if shadow.host_write(spec.address, &buf[..len]).is_err() {
        let _ = io.write_all(b"Write error\r\n").await;
        return;
    }

    let mut out: String<OUT_BUF_CAP> = String::new();
    let _ = write!(out, "OK: {} = ", spec.name);
    format_value(&buf[..len], spec.encoding, &mut out);
    let _ = write!(out, "\r\n");
    let _ = io.write_all(out.as_bytes()).await;
}

/// Dump command - hex dump of shadow bytes.
async fn cmd_dump<IO: Write, const N: usize>(
    addr: Option<&str>,
    len: Option<&str>,
    shadow: &ShadowStorage<N>,
    io: &mut IO,
) {
    let addr = match addr.and_then(parse_hex_or_dec) {
        Some(a) => a as u16,
        None => {
            let _ = io.write_all(b"Usage: dump <0xADDR> <len>\r\n").await;
            return;
        }
    };

    let len = match len.and_then(parse_hex_or_dec) {
        Some(l) => l.min(64) as usize, // Cap at 64 bytes
        None => 16,
    };

    let mut buf = [0u8; 64];
    if shadow.host_read(addr, &mut buf[..len]).is_err() {
        let _ = io.write_all(b"Read error\r\n").await;
        return;
    }

    let mut out: String<OUT_BUF_CAP> = String::new();
    let _ = write!(out, "0x{:03X}:", addr);
    for b in &buf[..len] {
        let _ = write!(out, " {:02X}", b);
    }
    let _ = write!(out, "\r\n");
    let _ = io.write_all(out.as_bytes()).await;
}

/// Status command - show key telemetry fields.
async fn cmd_status<IO: Write, const N: usize>(shadow: &ShadowStorage<N>, io: &mut IO) {
    // Operating mode
    let mut buf = [0u8; 4];
    let _ = shadow.host_read(vendor::addr::OPERATING_MODE, &mut buf[..1]);
    let mode = match buf[0] {
        0 => "Position",
        1 => "OpenLoop",
        _ => "Unknown",
    };

    // Torque enable
    let _ = shadow.host_read(vendor::addr::TORQUE_ENABLE, &mut buf[..1]);
    let torque = if buf[0] != 0 { "ON" } else { "OFF" };

    // Goal position (i32)
    let _ = shadow.host_read(vendor::addr::GOAL_POS_CDEG, &mut buf[..4]);
    let goal_pos = i32::from_le_bytes(buf);

    // Present position (i32)
    let _ = shadow.host_read(vendor::addr::PRESENT_POS_CDEG, &mut buf[..4]);
    let pres_pos = i32::from_le_bytes(buf);

    // Present current (i16)
    let _ = shadow.host_read(vendor::addr::PRESENT_CURRENT_MA, &mut buf[..2]);
    let current = i16::from_le_bytes([buf[0], buf[1]]);

    // Fault status
    let _ = shadow.host_read(vendor::addr::FAULT_STATUS, &mut buf[..1]);
    let fault = buf[0];

    let mut out: String<OUT_BUF_CAP> = String::new();
    let _ = write!(
        out,
        "Mode: {} | Torque: {}\r\n\
         Goal: {} cdeg | Pos: {} cdeg\r\n\
         Current: {} mA | Fault: 0x{:02X}\r\n",
        mode, torque, goal_pos, pres_pos, current, fault
    );
    let _ = io.write_all(out.as_bytes()).await;
}

// ============================================================================
// Helpers
// ============================================================================

/// Check if torque is enabled (vendor::addr::TORQUE_ENABLE).
fn is_torque_enabled<const N: usize>(shadow: &ShadowStorage<N>) -> bool {
    let mut buf = [0u8; 1];
    let _ = shadow.host_read(vendor::addr::TORQUE_ENABLE, &mut buf);
    buf[0] != 0
}

/// Lookup field by name prefix or hex address.
fn lookup_field(s: &str) -> Option<&'static RegSpec> {
    // Try hex address first (0x...)
    if let Some(addr) = parse_hex_addr(s) {
        return open_servo_registry::find_by_address(addr);
    }

    // Try name prefix (vendor registers only)
    vendor::find(s)
}

/// Parse hex address (0x...).
fn parse_hex_addr(s: &str) -> Option<u16> {
    let s = s.trim();
    if s.starts_with("0x") || s.starts_with("0X") {
        u16::from_str_radix(&s[2..], 16).ok()
    } else {
        None
    }
}

/// Parse hex (0x...) or decimal number.
fn parse_hex_or_dec(s: &str) -> Option<u32> {
    let s = s.trim();
    if s.starts_with("0x") || s.starts_with("0X") {
        u32::from_str_radix(&s[2..], 16).ok()
    } else if s.starts_with('-') {
        // Parse as signed then cast
        i32::from_str_radix(s, 10).ok().map(|v| v as u32)
    } else {
        u32::from_str_radix(s, 10).ok()
    }
}

/// Format a value based on encoding.
fn format_value(buf: &[u8], encoding: Encoding, out: &mut String<OUT_BUF_CAP>) {
    match encoding {
        Encoding::U8 => {
            let v = buf[0];
            let _ = write!(out, "{} (0x{:02X})", v, v);
        }
        Encoding::U16Le => {
            let v = u16::from_le_bytes([buf[0], buf[1]]);
            let _ = write!(out, "{} (0x{:04X})", v, v);
        }
        Encoding::U32Le => {
            let v = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
            let _ = write!(out, "{} (0x{:08X})", v, v);
        }
        Encoding::I16Le => {
            let v = i16::from_le_bytes([buf[0], buf[1]]);
            let _ = write!(out, "{}", v);
        }
        Encoding::I32Le => {
            let v = i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
            let _ = write!(out, "{}", v);
        }
        Encoding::Bool => {
            let v = buf[0] != 0;
            let _ = write!(out, "{}", v);
        }
        Encoding::Enum(names) => {
            let idx = buf[0] as usize;
            if idx < names.len() {
                let _ = write!(out, "{} ({})", names[idx], idx);
            } else {
                let _ = write!(out, "unknown ({})", idx);
            }
        }
        Encoding::Reserved(len) => {
            let _ = write!(out, "<reserved {} bytes>", len);
        }
    }
}

/// Parse a value string into bytes based on encoding.
fn parse_value(s: &str, encoding: Encoding, buf: &mut [u8]) -> bool {
    let s = s.trim();

    match encoding {
        Encoding::U8 => {
            if let Some(v) = parse_hex_or_dec(s) {
                buf[0] = v as u8;
                return true;
            }
        }
        Encoding::U16Le => {
            if let Some(v) = parse_hex_or_dec(s) {
                buf[..2].copy_from_slice(&(v as u16).to_le_bytes());
                return true;
            }
        }
        Encoding::U32Le => {
            if let Some(v) = parse_hex_or_dec(s) {
                buf[..4].copy_from_slice(&v.to_le_bytes());
                return true;
            }
        }
        Encoding::I16Le => {
            if let Some(v) = parse_signed(s) {
                buf[..2].copy_from_slice(&(v as i16).to_le_bytes());
                return true;
            }
        }
        Encoding::I32Le => {
            if let Some(v) = parse_signed(s) {
                buf[..4].copy_from_slice(&v.to_le_bytes());
                return true;
            }
        }
        Encoding::Bool => {
            let lower = s.to_ascii_lowercase();
            match lower.as_str() {
                "true" | "1" => {
                    buf[0] = 1;
                    return true;
                }
                "false" | "0" => {
                    buf[0] = 0;
                    return true;
                }
                _ => {}
            }
        }
        Encoding::Enum(names) => {
            // Try numeric first
            if let Some(v) = parse_hex_or_dec(s) {
                buf[0] = v as u8;
                return true;
            }
            // Try name match (case-insensitive)
            for (i, name) in names.iter().enumerate() {
                if eq_ignore_case(s, name) {
                    buf[0] = i as u8;
                    return true;
                }
            }
        }
        Encoding::Reserved(_) => {
            // Reserved fields are not writable
            return false;
        }
    }

    false
}

/// Parse a signed integer (supports negative).
fn parse_signed(s: &str) -> Option<i32> {
    let s = s.trim();
    if s.starts_with("0x") || s.starts_with("0X") {
        // Hex - parse as unsigned then cast
        u32::from_str_radix(&s[2..], 16).ok().map(|v| v as i32)
    } else {
        i32::from_str_radix(s, 10).ok()
    }
}

/// Case-insensitive string equality.
fn eq_ignore_case(a: &str, b: &str) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.bytes()
        .zip(b.bytes())
        .all(|(a, b)| a.to_ascii_lowercase() == b.to_ascii_lowercase())
}

/// Case-insensitive prefix check.
fn starts_with_ignore_case(haystack: &str, needle: &str) -> bool {
    if needle.len() > haystack.len() {
        return false;
    }
    haystack
        .bytes()
        .zip(needle.bytes())
        .all(|(h, n)| h.to_ascii_lowercase() == n.to_ascii_lowercase())
}

/// Convert string to lowercase (for command matching).
trait ToAsciiLowercase {
    fn to_ascii_lowercase(&self) -> String<16>;
}

impl ToAsciiLowercase for &str {
    fn to_ascii_lowercase(&self) -> String<16> {
        let mut s: String<16> = String::new();
        for c in self.chars().take(16) {
            let _ = s.push(c.to_ascii_lowercase());
        }
        s
    }
}

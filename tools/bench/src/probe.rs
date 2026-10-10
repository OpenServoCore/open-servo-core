//! Budget probe reader: the bench image's `KERNEL_PROBE` and `BUS_PROBE`
//! records (servo-ch32 `probe`, `--features bench`), found by symbol in the
//! flashed ELF and read off the running chip with `wlink dump`. A dump can
//! halt the hart (and costs the kernel 16 ticks): read between windows only,
//! and check the servo still runs after.

use std::process::Command;

use anyhow::{Context, Result, bail, ensure};
use osc_servo_core::budget::probe::{BusProbe, KernelProbe};

pub const KERNEL_SYMBOL: &str = "KERNEL_PROBE";
pub const BUS_SYMBOL: &str = "BUS_PROBE";

/// One read of both records: decoded, the bus record's last frame closed,
/// and the words as dumped.
#[derive(Debug)]
pub struct Snapshot {
    pub kernel: KernelProbe,
    pub bus: BusProbe,
    pub kernel_words: Vec<u32>,
    pub bus_words: Vec<u32>,
}

/// Where the two records sit in the bench image's RAM.
#[derive(Copy, Clone, Debug)]
pub struct Reader {
    kernel: u32,
    bus: u32,
}

impl Reader {
    /// Locate both records in `elf`, checking each size against the layout
    /// this build decodes.
    pub fn from_elf(elf: &[u8]) -> Result<Self> {
        let at = |name: &str, words: usize| -> Result<u32> {
            let (addr, size) =
                symbol(elf, name)?.with_context(|| format!("{name} missing: not a bench image"))?;
            ensure!(
                size as usize == 4 * words,
                "{name}: {size} B in the image, {} B decoded: rebuild the bench tools",
                4 * words
            );
            Ok(addr)
        };
        Ok(Self {
            kernel: at(KERNEL_SYMBOL, KernelProbe::WORDS)?,
            bus: at(BUS_SYMBOL, BusProbe::WORDS)?,
        })
    }

    pub fn read(&self) -> Result<Snapshot> {
        let kernel_words = words(&dump(self.kernel, 4 * KernelProbe::WORDS)?);
        let bus_words = words(&dump(self.bus, 4 * BusProbe::WORDS)?);
        Ok(Snapshot {
            kernel: KernelProbe::from_words(&kernel_words).context("kernel record")?,
            bus: BusProbe::from_words(&bus_words)
                .context("bus record")?
                .closed(),
            kernel_words,
            bus_words,
        })
    }
}

/// Restart a hart a dump left halted.
pub fn resume() -> Result<()> {
    let st = Command::new("wlink").arg("resume").status()?;
    ensure!(st.success(), "wlink resume: {st}");
    Ok(())
}

fn dump(addr: u32, len: usize) -> Result<Vec<u8>> {
    let out = Command::new("wlink")
        .args(["dump", &format!("{addr:#x}"), &len.to_string()])
        .output()
        .context("run wlink")?;
    ensure!(out.status.success(), "wlink dump: {}", out.status);
    let bytes = parse_dump(&String::from_utf8_lossy(&out.stdout), addr, len);
    match bytes {
        Some(b) => Ok(b),
        None => bail!("wlink dump at {addr:#x}: short or unparsed output"),
    }
}

/// `len` bytes from `base` out of `wlink dump`'s hex listing: lines of an
/// 8-digit address, a colon, then up to 16 two-digit bytes (colour codes and
/// the text column ignored).
pub fn parse_dump(text: &str, base: u32, len: usize) -> Option<Vec<u8>> {
    let mut out = vec![None; len];
    for line in text.lines() {
        let line = strip_ansi(line);
        let Some((addr, rest)) = line.trim_start().split_once(':') else {
            continue;
        };
        if addr.len() != 8 {
            continue;
        }
        let Ok(addr) = u32::from_str_radix(addr, 16) else {
            continue;
        };
        let bytes = rest
            .split_whitespace()
            .map_while(|t| {
                (t.len() == 2)
                    .then(|| u8::from_str_radix(t, 16).ok())
                    .flatten()
            })
            .take(16);
        for (k, b) in bytes.enumerate() {
            let off = (addr as usize + k).wrapping_sub(base as usize);
            if let Some(slot) = out.get_mut(off) {
                *slot = Some(b);
            }
        }
    }
    out.into_iter().collect()
}

fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c == '\x1b' {
            for c in chars.by_ref() {
                if c.is_ascii_alphabetic() {
                    break;
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// Little-endian words.
pub fn words(bytes: &[u8]) -> Vec<u32> {
    let (words, _) = bytes.as_chunks::<4>();
    words.iter().map(|w| u32::from_le_bytes(*w)).collect()
}

const SHT_SYMTAB: u32 = 2;
const EHDR_LEN: usize = 52;
const SYM_LEN: usize = 16;

/// `name`'s value and size from a little-endian ELF32's symbol table.
pub fn symbol(elf: &[u8], name: &str) -> Result<Option<(u32, u32)>> {
    ensure!(
        elf.len() >= EHDR_LEN && elf[..4] == *b"\x7fELF" && elf[4] == 1 && elf[5] == 1,
        "not a little-endian ELF32"
    );
    let u16_at = |o: usize| elf.get(o..o + 2).map(|b| u16::from_le_bytes([b[0], b[1]]));
    let u32_at = |o: usize| {
        elf.get(o..o + 4)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    };
    let field = |o: usize| u32_at(o).context("truncated ELF");
    let shoff = field(0x20)? as usize;
    let shentsize = u16_at(0x2e).context("truncated ELF")? as usize;
    let shnum = u16_at(0x30).context("truncated ELF")? as usize;
    let section = |i: usize| -> Result<(u32, usize, usize, usize)> {
        let h = shoff + i * shentsize;
        Ok((
            field(h + 4)?,
            field(h + 16)? as usize,
            field(h + 20)? as usize,
            field(h + 24)? as usize,
        ))
    };
    for i in 0..shnum {
        let (kind, off, size, link) = section(i)?;
        if kind != SHT_SYMTAB {
            continue;
        }
        let (_, str_off, str_size, _) = section(link)?;
        let strtab = elf
            .get(str_off..str_off + str_size)
            .context("truncated strtab")?;
        for s in (off..off + size).step_by(SYM_LEN) {
            let at = field(s)? as usize;
            let sym = strtab.get(at..).unwrap_or(&[]);
            let end = sym.iter().position(|&b| b == 0).unwrap_or(sym.len());
            if &sym[..end] == name.as_bytes() {
                return Ok(Some((field(s + 4)?, field(s + 8)?)));
            }
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An ELF32 with a symbol table holding `syms`.
    fn elf(syms: &[(&str, u32, u32)]) -> Vec<u8> {
        let mut strtab = vec![0u8];
        let mut symtab = vec![0u8; SYM_LEN];
        for (name, value, size) in syms {
            symtab.extend((strtab.len() as u32).to_le_bytes());
            symtab.extend(value.to_le_bytes());
            symtab.extend(size.to_le_bytes());
            symtab.extend([0u8; 4]);
            strtab.extend(name.as_bytes());
            strtab.push(0);
        }
        let symtab_off = EHDR_LEN;
        let strtab_off = symtab_off + symtab.len();
        let shoff = strtab_off + strtab.len();
        let mut e = vec![0u8; EHDR_LEN];
        e[..6].copy_from_slice(b"\x7fELF\x01\x01");
        e[0x20..0x24].copy_from_slice(&(shoff as u32).to_le_bytes());
        e[0x2e..0x30].copy_from_slice(&40u16.to_le_bytes());
        e[0x30..0x32].copy_from_slice(&3u16.to_le_bytes());
        e.extend(&symtab);
        e.extend(&strtab);
        let header = |kind: u32, off: usize, size: usize, link: u32| {
            let mut h = vec![0u8; 40];
            h[4..8].copy_from_slice(&kind.to_le_bytes());
            h[16..20].copy_from_slice(&(off as u32).to_le_bytes());
            h[20..24].copy_from_slice(&(size as u32).to_le_bytes());
            h[24..28].copy_from_slice(&link.to_le_bytes());
            h
        };
        e.extend(header(0, 0, 0, 0));
        e.extend(header(SHT_SYMTAB, symtab_off, symtab.len(), 2));
        e.extend(header(3, strtab_off, strtab.len(), 0));
        e
    }

    #[test]
    fn symbols_resolve_by_name() {
        let e = elf(&[
            ("BUS_PROBE", 0x2000_0100, 120),
            ("KERNEL_PROBE", 0x2000_0200, 312),
        ]);
        assert_eq!(
            symbol(&e, "KERNEL_PROBE").unwrap(),
            Some((0x2000_0200, 312))
        );
        assert_eq!(symbol(&e, "BUS_PROBE").unwrap(), Some((0x2000_0100, 120)));
        assert_eq!(symbol(&e, "HIGH_PROBE").unwrap(), None);
        assert!(symbol(b"not an elf", "X").is_err());
    }

    #[test]
    fn a_reader_refuses_a_record_of_another_layout() {
        let k = 4 * KernelProbe::WORDS as u32;
        let e = elf(&[("BUS_PROBE", 0x100, 8), ("KERNEL_PROBE", 0x200, k)]);
        assert!(Reader::from_elf(&e).is_err());
        let b = 4 * BusProbe::WORDS as u32;
        let e = elf(&[("BUS_PROBE", 0x100, b), ("KERNEL_PROBE", 0x200, k)]);
        assert!(Reader::from_elf(&e).is_ok());
    }

    #[test]
    fn dumps_parse_across_lines_and_colour() {
        let text = "\x1b[32m20000100\x1b[0m:  01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10  |................|\n\
                    20000110:  11 12 13 14  |....|\n";
        let b = parse_dump(text, 0x2000_0102, 6).unwrap();
        assert_eq!(b, [3, 4, 5, 6, 7, 8]);
        let b = parse_dump(text, 0x2000_010e, 4).unwrap();
        assert_eq!(b, [0x0f, 0x10, 0x11, 0x12]);
        assert_eq!(parse_dump(text, 0x2000_0110, 8), None, "short");
        assert_eq!(words(&[1, 0, 0, 0, 0, 1, 0, 0]), [1, 256]);
    }
}

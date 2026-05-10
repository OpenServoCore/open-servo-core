use std::env;
use std::error::Error;
use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::{Path, PathBuf};

use ch32_metapac::metadata::METADATA;

fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers {
            println!("cargo:rustc-cfg={}_{}", regs.kind, regs.version);
        }
    }

    generate_pin_enum(&out)?;

    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}

fn port_index(port: char) -> usize {
    (port as usize) - ('A' as usize)
}

fn generate_pin_enum(out: &Path) -> Result<(), Box<dyn Error>> {
    let mut pins: Vec<(char, u8)> = Vec::new();
    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers
            && regs.kind == "gpio"
        {
            let port = p.name.chars().nth(4).unwrap();
            let pins_per_port: u8 = match regs.version {
                "v0" => 8,
                "v3" => 16,
                "x0" => 24,
                _ => 8,
            };
            for n in 0..pins_per_port {
                pins.push((port, n));
            }
        }
    }
    pins.sort();

    let mut code = String::new();
    writeln!(code, "#[derive(Copy, Clone, Debug, PartialEq, Eq)]")?;
    writeln!(code, "#[repr(u8)]")?;
    writeln!(code, "#[allow(dead_code)]")?;
    writeln!(code, "pub enum Pin {{")?;
    for &(port, num) in &pins {
        let discrim = (port_index(port) << 5) | (num as usize);
        writeln!(code, "    P{port}{num} = {discrim:#04x},")?;
    }
    writeln!(code, "}}")?;
    writeln!(code)?;

    writeln!(code, "impl Pin {{")?;
    writeln!(code, "    #[inline(always)]")?;
    writeln!(
        code,
        "    pub const fn port_index(self) -> usize {{ (self as u8 >> 5) as usize }}"
    )?;
    writeln!(code)?;
    writeln!(code, "    #[inline(always)]")?;
    writeln!(
        code,
        "    pub const fn pin_number(self) -> usize {{ (self as u8 & 0x1f) as usize }}"
    )?;
    writeln!(code)?;
    writeln!(code, "    #[inline(always)]")?;
    writeln!(
        code,
        "    pub fn gpio_regs(self) -> ch32_metapac::gpio::Gpio {{ ch32_metapac::GPIO(self.port_index()) }}"
    )?;
    writeln!(code, "}}")?;

    fs::write(out.join("generated.rs"), code)?;
    Ok(())
}

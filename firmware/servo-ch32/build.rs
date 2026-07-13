use std::collections::{BTreeMap, BTreeSet};
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

    generate_pin_and_usart_mapping(&out)?;

    // Ship the saved-config linker fragment (`INCLUDE osc-config.x` in a
    // board's memory.x); the link-search path propagates to the board link.
    fs::copy("osc-config.x", out.join("osc-config.x"))?;
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=osc-config.x");

    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}

fn port_index(port: char) -> usize {
    (port as usize) - ('A' as usize)
}

fn generate_pin_and_usart_mapping(out: &Path) -> Result<(), Box<dyn Error>> {
    let mut code = String::new();

    // discriminant = (port_index << 5) | pin_number.
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
    writeln!(code)?;

    struct RemapGroup {
        peripheral_name: String,
        tx_pin: Option<String>,
        rx_pin: Option<String>,
    }

    let mut groups: BTreeMap<(String, u8), RemapGroup> = BTreeMap::new();

    for p in METADATA.peripherals {
        if let Some(regs) = &p.registers {
            if regs.kind != "usart" {
                continue;
            }
            for pin_entry in p.pins {
                let remap_val = match pin_entry.remap {
                    Some(r) => r,
                    None => continue,
                };
                let key = (p.name.to_string(), remap_val);
                let group = groups.entry(key).or_insert_with(|| RemapGroup {
                    peripheral_name: p.name.to_string(),
                    tx_pin: None,
                    rx_pin: None,
                });
                match pin_entry.signal {
                    "TX" => group.tx_pin = Some(pin_entry.pin.to_string()),
                    "RX" => group.rx_pin = Some(pin_entry.pin.to_string()),
                    _ => {}
                }
            }
        }
    }

    writeln!(code, "#[derive(Copy, Clone, Debug, PartialEq, Eq)]")?;
    writeln!(code, "#[allow(dead_code)]")?;
    writeln!(code, "pub enum UsartMapping {{")?;
    for ((peri, remap), group) in &groups {
        let variant = format!("{}Remap{}", capitalize_peripheral(peri), remap);
        let tx = group.tx_pin.as_deref().unwrap_or("?");
        let rx = group.rx_pin.as_deref().unwrap_or("?");
        writeln!(code, "    /// {peri} remap {remap}: TX={tx}, RX={rx}")?;
        writeln!(code, "    {variant},")?;
    }
    writeln!(code, "}}")?;
    writeln!(code)?;

    writeln!(code, "impl UsartMapping {{")?;

    writeln!(code, "    pub const fn tx_pin(self) -> Pin {{")?;
    writeln!(code, "        match self {{")?;
    for ((peri, remap), group) in &groups {
        let variant = format!("{}Remap{}", capitalize_peripheral(peri), remap);
        let tx = group.tx_pin.as_ref().expect("USART mapping missing TX pin");
        writeln!(code, "            UsartMapping::{variant} => Pin::{tx},")?;
    }
    writeln!(code, "        }}")?;
    writeln!(code, "    }}")?;
    writeln!(code)?;

    writeln!(code, "    pub const fn rx_pin(self) -> Pin {{")?;
    writeln!(code, "        match self {{")?;
    for ((peri, remap), group) in &groups {
        let variant = format!("{}Remap{}", capitalize_peripheral(peri), remap);
        let rx = group.rx_pin.as_ref().expect("USART mapping missing RX pin");
        writeln!(code, "            UsartMapping::{variant} => Pin::{rx},")?;
    }
    writeln!(code, "        }}")?;
    writeln!(code, "    }}")?;
    writeln!(code)?;

    writeln!(code, "    pub const fn remap_value(self) -> u8 {{")?;
    writeln!(code, "        match self {{")?;
    for (peri, remap) in groups.keys() {
        let variant = format!("{}Remap{}", capitalize_peripheral(peri), remap);
        writeln!(code, "            UsartMapping::{variant} => {remap},")?;
    }
    writeln!(code, "        }}")?;
    writeln!(code, "    }}")?;
    writeln!(code)?;

    writeln!(
        code,
        "    pub const fn regs(self) -> ch32_metapac::usart::Usart {{"
    )?;
    writeln!(code, "        match self {{")?;
    for ((peri, remap), group) in &groups {
        let variant = format!("{}Remap{}", capitalize_peripheral(peri), remap);
        let peri_const = &group.peripheral_name;
        writeln!(
            code,
            "            UsartMapping::{variant} => ch32_metapac::{peri_const},"
        )?;
    }
    writeln!(code, "        }}")?;
    writeln!(code, "    }}")?;
    writeln!(code)?;

    writeln!(code, "    pub const fn peripheral_index(self) -> u8 {{")?;
    writeln!(code, "        match self {{")?;
    for (peri, remap) in groups.keys() {
        let variant = format!("{}Remap{}", capitalize_peripheral(peri), remap);
        let index: String = peri.chars().filter(|c| c.is_ascii_digit()).collect();
        writeln!(code, "            UsartMapping::{variant} => {index},")?;
    }
    writeln!(code, "        }}")?;
    writeln!(code, "    }}")?;

    writeln!(code, "}}")?;

    generate_timer_mappings(&mut code)?;

    fs::write(out.join("generated.rs"), code)?;
    Ok(())
}

fn generate_timer_mappings(code: &mut String) -> Result<(), Box<dyn Error>> {
    const SIGNALS: &[(&str, &str)] = &[
        ("CH1", "ch1_pin"),
        ("CH2", "ch2_pin"),
        ("CH3", "ch3_pin"),
        ("CH4", "ch4_pin"),
        ("ETR", "etr_pin"),
        ("BKIN", "bkin_pin"),
        ("CH1N", "ch1n_pin"),
        ("CH2N", "ch2n_pin"),
        ("CH3N", "ch3n_pin"),
    ];

    for p in METADATA.peripherals {
        let Some(regs) = &p.registers else { continue };
        if regs.kind != "timer" {
            continue;
        }

        let mut remaps: BTreeSet<u8> = BTreeSet::new();
        for pin in p.pins {
            if let Some(r) = pin.remap {
                remaps.insert(r);
            }
        }
        if remaps.is_empty() {
            continue;
        }

        let enum_name = format!("{}Mapping", capitalize_peripheral(p.name));
        let timer_has_signal: BTreeSet<&str> = p.pins.iter().map(|pp| pp.signal).collect();

        writeln!(code)?;
        writeln!(code, "#[derive(Copy, Clone, Debug, PartialEq, Eq)]")?;
        writeln!(code, "#[repr(u8)]")?;
        writeln!(code, "#[allow(dead_code)]")?;
        writeln!(code, "pub enum {enum_name} {{")?;
        for r in &remaps {
            writeln!(code, "    Remap{r} = {r},")?;
        }
        writeln!(code, "}}")?;
        writeln!(code)?;

        writeln!(code, "impl {enum_name} {{")?;
        writeln!(code, "    #[inline(always)]")?;
        writeln!(
            code,
            "    pub const fn remap_value(self) -> u8 {{ self as u8 }}"
        )?;

        for (signal, method) in SIGNALS {
            if !timer_has_signal.contains(signal) {
                continue;
            }

            let pins_by_remap: Vec<(u8, Option<&str>)> = remaps
                .iter()
                .map(|r| {
                    let pin = p
                        .pins
                        .iter()
                        .find(|pp| pp.remap == Some(*r) && pp.signal == *signal)
                        .map(|pp| pp.pin);
                    (*r, pin)
                })
                .collect();

            // Accessor returns `Pin` directly if every remap exposes signal; else `Option<Pin>`.
            let always_exposed = pins_by_remap.iter().all(|(_, pin)| pin.is_some());

            writeln!(code)?;
            if always_exposed {
                writeln!(code, "    pub const fn {method}(self) -> Pin {{")?;
                writeln!(code, "        match self {{")?;
                for (r, pin) in &pins_by_remap {
                    let name = pin.expect("always_exposed checked above");
                    writeln!(code, "            {enum_name}::Remap{r} => Pin::{name},")?;
                }
            } else {
                writeln!(code, "    pub const fn {method}(self) -> Option<Pin> {{")?;
                writeln!(code, "        match self {{")?;
                for (r, pin) in &pins_by_remap {
                    let value = match pin {
                        Some(name) => format!("Some(Pin::{name})"),
                        None => "None".to_string(),
                    };
                    writeln!(code, "            {enum_name}::Remap{r} => {value},")?;
                }
            }
            writeln!(code, "        }}")?;
            writeln!(code, "    }}")?;
        }

        writeln!(code, "}}")?;
    }

    Ok(())
}

fn capitalize_peripheral(name: &str) -> String {
    let mut result = String::new();
    let mut capitalize_next = true;
    for c in name.chars() {
        if capitalize_next {
            result.push(c.to_ascii_uppercase());
            capitalize_next = false;
        } else {
            result.push(c.to_ascii_lowercase());
        }
    }
    result
}

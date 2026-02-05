//! Registry panel - Read/write device registers.

use crate::app::ConnectCommand;
use crate::rtt::{RpcClient, RpcResponse};
use heapless::Vec as HVec;
use open_servo_registry::{all_fields, Encoding, RegSpec};
use open_servo_rpc::{ReadRegReq, RpcResp, WriteRegReq, KEY_REG_READ, KEY_REG_WRITE};
use std::collections::HashMap;
use std::sync::mpsc::Sender;

/// A register entry for display (resolved from registry).
struct RegisterEntry {
    spec: &'static RegSpec,
    value: Option<i64>,
    /// Text being edited (None = not editing)
    edit_text: Option<String>,
}

impl RegisterEntry {
    fn from_spec(spec: &'static RegSpec) -> Self {
        Self {
            spec,
            value: None,
            edit_text: None,
        }
    }

    /// Get the hex string for the current value, masked to correct byte width.
    fn hex_string(&self) -> String {
        if let Some(val) = self.value {
            let mask = match self.spec.len() {
                1 => 0xFF,
                2 => 0xFFFF,
                4 => 0xFFFF_FFFF,
                _ => 0xFFFF_FFFF_FFFF_FFFF,
            };
            let masked = (val as u64) & mask;
            format!(
                "0x{:0width$X}",
                masked,
                width = (self.spec.len() * 2) as usize
            )
        } else {
            "-".to_string()
        }
    }

    /// Infer unit type from register name suffix.
    fn unit_hint(&self) -> &'static str {
        let name = self.spec.name;
        if name.ends_with("_cdeg") {
            "cdeg"
        } else if name.ends_with("_centic") {
            "c°C"
        } else if name.ends_with("_mv") {
            "mV"
        } else if name.ends_with("_ma") {
            "mA"
        } else if name.ends_with("_dps10") {
            "dps/10"
        } else if name.ends_with("_raw") {
            "raw"
        } else if name.ends_with("_us") {
            "µs"
        } else {
            ""
        }
    }
}

pub struct RegistryPanel {
    registers: Vec<RegisterEntry>,
    filter: String,
    /// Maps seq number -> register address for pending reads
    pending_reads: HashMap<u32, u16>,
    /// Maps seq number -> register address for pending writes
    pending_writes: HashMap<u32, u16>,
}

impl RegistryPanel {
    pub fn new() -> Self {
        // Build register list from open-servo-registry (includes EEPROM, RAM, and Vendor)
        let registers: Vec<RegisterEntry> = all_fields()
            .filter(|spec| !spec.is_reserved())
            .map(RegisterEntry::from_spec)
            .collect();

        Self {
            registers,
            filter: String::new(),
            pending_reads: HashMap::new(),
            pending_writes: HashMap::new(),
        }
    }

    pub fn handle_response(&mut self, resp: &RpcResponse) {
        // Handle reg/read response
        if resp.key == KEY_REG_READ {
            let addr = self.pending_reads.remove(&resp.seq);

            match postcard::from_bytes::<RpcResp<HVec<u8, 4>>>(&resp.payload) {
                Ok(RpcResp::Ok(data)) => {
                    if let Some(addr) = addr {
                        for reg in &mut self.registers {
                            if reg.spec.address == addr {
                                let val = parse_register_value(&data, reg.spec.encoding);
                                reg.value = Some(val);
                                break;
                            }
                        }
                    }
                }
                Ok(RpcResp::Err(e)) => {
                    tracing::warn!("Read failed for addr {:?}: {:?}", addr, e);
                }
                Err(e) => {
                    tracing::error!("Failed to decode RpcResp: {:?}", e);
                }
            }
        }

        // Handle reg/write response
        if resp.key == KEY_REG_WRITE {
            let addr = self.pending_writes.remove(&resp.seq);

            match postcard::from_bytes::<RpcResp<()>>(&resp.payload) {
                Ok(RpcResp::Ok(())) => {
                    tracing::info!("Write succeeded for addr {:?}", addr);
                }
                Ok(RpcResp::Err(e)) => {
                    tracing::warn!("Write failed for addr {:?}: {:?}", addr, e);
                }
                Err(e) => {
                    tracing::error!("Failed to decode write RpcResp: {:?}", e);
                }
            }
        }
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        is_connected: bool,
        cmd_tx: &Option<Sender<ConnectCommand>>,
        rpc_client: &mut RpcClient,
    ) {
        ui.heading("Device Registers");
        ui.add_space(10.0);

        ui.horizontal(|ui| {
            ui.label("Filter:");
            ui.text_edit_singleline(&mut self.filter);

            if ui
                .add_enabled(is_connected, egui::Button::new("Read All"))
                .clicked()
            {
                self.read_all_registers(cmd_tx, rpc_client);
            }
        });

        ui.add_space(10.0);

        if !is_connected {
            ui.colored_label(egui::Color32::GRAY, "Connect to a device to read registers");
            return;
        }

        // Collect requests to avoid borrow issues
        let mut read_requests: Vec<(u16, u8)> = Vec::new();
        let mut write_requests: Vec<(u16, i64, Encoding)> = Vec::new();

        let filter_lower = self.filter.to_lowercase();

        // Register table with full width
        egui::ScrollArea::vertical().show(ui, |ui| {
            use egui_extras::{Column, TableBuilder};

            TableBuilder::new(ui)
                .striped(true)
                .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                .column(Column::exact(50.0)) // Addr
                .column(Column::remainder().at_least(150.0)) // Name
                .column(Column::exact(40.0)) // Type
                .column(Column::exact(40.0)) // Unit
                .column(Column::exact(80.0)) // Value
                .column(Column::exact(100.0)) // Hex
                .column(Column::exact(100.0)) // Actions
                .min_scrolled_height(0.0)
                .header(20.0, |mut header| {
                    header.col(|ui| {
                        ui.strong("Addr");
                    });
                    header.col(|ui| {
                        ui.strong("Name");
                    });
                    header.col(|ui| {
                        ui.strong("Type");
                    });
                    header.col(|ui| {
                        ui.strong("Unit");
                    });
                    header.col(|ui| {
                        ui.strong("Value");
                    });
                    header.col(|ui| {
                        ui.strong("Hex");
                    });
                    header.col(|ui| {
                        ui.strong("Actions");
                    });
                })
                .body(|mut body| {
                    // Build filtered indices list
                    let filtered_indices: Vec<usize> = (0..self.registers.len())
                        .filter(|&idx| {
                            let reg = &self.registers[idx];
                            self.filter.is_empty()
                                || reg.spec.name.to_lowercase().contains(&filter_lower)
                                || reg.spec.address.to_string().contains(&self.filter)
                        })
                        .collect();

                    for idx in filtered_indices {
                        let addr;
                        let size;
                        let encoding;
                        let writable;
                        let is_pending;
                        let name;
                        let wire_type;
                        let unit_hint;
                        let is_signed;
                        let current_value;

                        // Collect all needed data first (immutable borrow)
                        {
                            let reg = &self.registers[idx];
                            addr = reg.spec.address;
                            size = reg.spec.len();
                            encoding = reg.spec.encoding;
                            writable = reg.spec.is_writable();
                            is_pending = self.pending_reads.values().any(|&a| a == addr);
                            name = reg.spec.name;
                            wire_type = reg.spec.encoding.wire_type();
                            unit_hint = reg.unit_hint();
                            is_signed = reg.spec.encoding.is_signed();
                            current_value = reg.value;
                        }

                        body.row(20.0, |mut row| {
                            // Address
                            row.col(|ui| {
                                ui.monospace(format!("{}", addr));
                            });

                            // Name
                            row.col(|ui| {
                                ui.label(name);
                            });

                            // Type (wire type)
                            row.col(|ui| {
                                ui.monospace(wire_type);
                            });

                            // Unit
                            row.col(|ui| {
                                ui.label(unit_hint);
                            });

                            // Value (editable if writable)
                            row.col(|ui| {
                                if writable {
                                    let reg = &mut self.registers[idx];
                                    // Editable field
                                    let current_text = reg.edit_text.get_or_insert_with(|| {
                                        reg.value.map(|v| v.to_string()).unwrap_or_default()
                                    });
                                    let response = ui.add(
                                        egui::TextEdit::singleline(current_text)
                                            .desired_width(70.0)
                                            .font(egui::TextStyle::Monospace),
                                    );
                                    if response.changed() {
                                        // Update value from text if valid
                                        if let Ok(parsed) = current_text.parse::<i64>() {
                                            // Validate against type range
                                            let (min, max) = get_type_range(encoding);
                                            if parsed >= min && parsed <= max {
                                                reg.value = Some(parsed);
                                            }
                                        }
                                    }
                                    if response.lost_focus() {
                                        // Sync edit_text with actual value
                                        reg.edit_text = reg.value.map(|v| v.to_string());
                                    }
                                } else {
                                    // Read-only display
                                    if let Some(val) = current_value {
                                        let display = if is_signed {
                                            format!("{}", val)
                                        } else {
                                            format!("{}", val as u64)
                                        };
                                        ui.monospace(display);
                                    } else if is_pending {
                                        ui.label("...");
                                    } else {
                                        ui.label("-");
                                    }
                                }
                            });

                            // Hex (auto-updates when value changes)
                            row.col(|ui| {
                                // Re-compute hex in case value changed
                                let reg = &self.registers[idx];
                                ui.monospace(reg.hex_string());
                            });

                            // Actions
                            row.col(|ui| {
                                ui.horizontal(|ui| {
                                    if ui.button("Read").clicked() {
                                        read_requests.push((addr, size));
                                    }
                                    if writable {
                                        let reg = &self.registers[idx];
                                        let can_write = reg.value.is_some();
                                        if ui
                                            .add_enabled(can_write, egui::Button::new("Write"))
                                            .clicked()
                                        {
                                            if let Some(val) = reg.value {
                                                write_requests.push((addr, val, encoding));
                                            }
                                        }
                                    }
                                });
                            });
                        });
                    }
                });
        });

        // Execute collected read requests
        for (addr, size) in read_requests {
            self.read_register(addr, size, cmd_tx, rpc_client);
        }

        // Execute collected write requests
        for (addr, val, encoding) in write_requests {
            self.write_register(addr, val, encoding, cmd_tx, rpc_client);
        }
    }

    fn read_register(
        &mut self,
        addr: u16,
        size: u8,
        cmd_tx: &Option<Sender<ConnectCommand>>,
        rpc_client: &mut RpcClient,
    ) {
        if let Some(ref tx) = cmd_tx {
            let req = ReadRegReq { addr, len: size };
            if let Ok((data, seq)) = rpc_client.encode_request("reg/read", &req) {
                self.pending_reads.insert(seq, addr);
                let _ = tx.send(ConnectCommand::SendRpc(data));
            }
        }
    }

    fn write_register(
        &mut self,
        addr: u16,
        value: i64,
        encoding: Encoding,
        cmd_tx: &Option<Sender<ConnectCommand>>,
        rpc_client: &mut RpcClient,
    ) {
        if let Some(ref tx) = cmd_tx {
            // Convert value to bytes based on encoding
            let data: HVec<u8, 4> = encode_value(value, encoding);
            let req = WriteRegReq { addr, data };
            if let Ok((payload, seq)) = rpc_client.encode_request("reg/write", &req) {
                self.pending_writes.insert(seq, addr);
                let _ = tx.send(ConnectCommand::SendRpc(payload));
            }
        }
    }

    fn read_all_registers(
        &mut self,
        cmd_tx: &Option<Sender<ConnectCommand>>,
        rpc_client: &mut RpcClient,
    ) {
        let regs: Vec<_> = self
            .registers
            .iter()
            .map(|r| (r.spec.address, r.spec.len()))
            .collect();
        for (addr, size) in regs {
            self.read_register(addr, size, cmd_tx, rpc_client);
        }
    }
}

impl Default for RegistryPanel {
    fn default() -> Self {
        Self::new()
    }
}

/// Parse a register value from bytes based on encoding.
fn parse_register_value(data: &[u8], encoding: Encoding) -> i64 {
    match encoding {
        Encoding::U8 | Encoding::Bool | Encoding::Enum(_) => {
            data.first().copied().unwrap_or(0) as i64
        }
        Encoding::I16Le => i16::from_le_bytes([
            data.first().copied().unwrap_or(0),
            data.get(1).copied().unwrap_or(0),
        ]) as i64,
        Encoding::U16Le => u16::from_le_bytes([
            data.first().copied().unwrap_or(0),
            data.get(1).copied().unwrap_or(0),
        ]) as i64,
        Encoding::I32Le => i32::from_le_bytes([
            data.first().copied().unwrap_or(0),
            data.get(1).copied().unwrap_or(0),
            data.get(2).copied().unwrap_or(0),
            data.get(3).copied().unwrap_or(0),
        ]) as i64,
        Encoding::U32Le => u32::from_le_bytes([
            data.first().copied().unwrap_or(0),
            data.get(1).copied().unwrap_or(0),
            data.get(2).copied().unwrap_or(0),
            data.get(3).copied().unwrap_or(0),
        ]) as i64,
        Encoding::Reserved(_) => 0,
    }
}

/// Get min/max range for an encoding type.
fn get_type_range(encoding: Encoding) -> (i64, i64) {
    match encoding {
        Encoding::U8 => (0, u8::MAX as i64),
        Encoding::Bool => (0, 1),
        Encoding::Enum(variants) => (0, variants.len().saturating_sub(1) as i64),
        Encoding::I16Le => (i16::MIN as i64, i16::MAX as i64),
        Encoding::U16Le => (0, u16::MAX as i64),
        Encoding::I32Le => (i32::MIN as i64, i32::MAX as i64),
        Encoding::U32Le => (0, u32::MAX as i64),
        Encoding::Reserved(_) => (0, 0),
    }
}

/// Encode a value to bytes based on encoding.
fn encode_value(value: i64, encoding: Encoding) -> HVec<u8, 4> {
    let mut result = HVec::new();
    match encoding {
        Encoding::U8 | Encoding::Bool | Encoding::Enum(_) => {
            let _ = result.push(value as u8);
        }
        Encoding::I16Le => {
            let bytes = (value as i16).to_le_bytes();
            let _ = result.extend_from_slice(&bytes);
        }
        Encoding::U16Le => {
            let bytes = (value as u16).to_le_bytes();
            let _ = result.extend_from_slice(&bytes);
        }
        Encoding::I32Le => {
            let bytes = (value as i32).to_le_bytes();
            let _ = result.extend_from_slice(&bytes);
        }
        Encoding::U32Le => {
            let bytes = (value as u32).to_le_bytes();
            let _ = result.extend_from_slice(&bytes);
        }
        Encoding::Reserved(_) => {}
    }
    result
}

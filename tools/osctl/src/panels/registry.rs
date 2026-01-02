//! Registry panel - Read/write device registers.

use crate::app::ConnectCommand;
use crate::rtt::{RpcClient, RpcResponse};
use heapless::Vec as HVec;
use open_servo_registry::{all_fields, Encoding, RegSpec};
use open_servo_rpc::{ReadRegReq, RpcResp, KEY_REG_READ};
use std::collections::HashMap;
use std::sync::mpsc::Sender;

/// A register entry for display (resolved from registry).
struct RegisterEntry {
    spec: &'static RegSpec,
    value: Option<i64>,
}

impl RegisterEntry {
    fn from_spec(spec: &'static RegSpec) -> Self {
        Self { spec, value: None }
    }

    fn is_signed(&self) -> bool {
        matches!(self.spec.encoding, Encoding::I16Le | Encoding::I32Le)
    }
}

pub struct RegistryPanel {
    registers: Vec<RegisterEntry>,
    filter: String,
    /// Maps seq number -> register address for pending reads
    pending_reads: HashMap<u32, u16>,
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
        }
    }

    pub fn handle_response(&mut self, resp: &RpcResponse) {
        // Check if this is a reg/read response
        if resp.key == KEY_REG_READ {
            // Look up the address by seq number
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
            ui.colored_label(
                egui::Color32::GRAY,
                "Connect to a device to read registers",
            );
            return;
        }

        // Collect read requests to avoid borrow issues
        let mut read_requests: Vec<(u16, u8)> = Vec::new();

        // Register table
        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::Grid::new("register_grid")
                .num_columns(6)
                .striped(true)
                .spacing([10.0, 4.0])
                .show(ui, |ui| {
                    // Header
                    ui.strong("Addr");
                    ui.strong("Name");
                    ui.strong("Size");
                    ui.strong("Value");
                    ui.strong("Hex");
                    ui.strong("Actions");
                    ui.end_row();

                    let filter_lower = self.filter.to_lowercase();

                    for reg in &self.registers {
                        // Apply filter
                        if !self.filter.is_empty()
                            && !reg.spec.name.to_lowercase().contains(&filter_lower)
                            && !reg.spec.address.to_string().contains(&self.filter)
                        {
                            continue;
                        }

                        // Address with region indicator
                        let addr_str = format!("{}", reg.spec.address);
                        ui.monospace(addr_str);

                        // Name
                        ui.label(reg.spec.name);

                        // Size
                        ui.label(format!("{}B", reg.spec.len()));

                        // Value display
                        if let Some(val) = reg.value {
                            // Show as signed or unsigned based on encoding
                            let display_val = if reg.is_signed() {
                                format!("{}", val)
                            } else {
                                format!("{}", val as u64)
                            };
                            ui.monospace(display_val);
                            ui.monospace(format!(
                                "0x{:0width$X}",
                                val as u64,
                                width = (reg.spec.len() * 2) as usize
                            ));
                        } else if self
                            .pending_reads
                            .values()
                            .any(|&a| a == reg.spec.address)
                        {
                            ui.label("...");
                            ui.label("");
                        } else {
                            ui.label("-");
                            ui.label("-");
                        }

                        // Actions - collect requests to execute after iteration
                        let addr = reg.spec.address;
                        let size = reg.spec.len();
                        let writable = reg.spec.is_writable();
                        ui.horizontal(|ui| {
                            if ui.small_button("R").on_hover_text("Read").clicked() {
                                read_requests.push((addr, size));
                            }
                            if writable {
                                if ui.small_button("W").on_hover_text("Write").clicked() {
                                    // TODO: Open write dialog
                                }
                            }
                        });

                        ui.end_row();
                    }
                });
        });

        // Execute collected read requests
        for (addr, size) in read_requests {
            self.read_register(addr, size, cmd_tx, rpc_client);
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
        Encoding::I16Le => {
            i16::from_le_bytes([
                data.first().copied().unwrap_or(0),
                data.get(1).copied().unwrap_or(0),
            ]) as i64
        }
        Encoding::U16Le => {
            u16::from_le_bytes([
                data.first().copied().unwrap_or(0),
                data.get(1).copied().unwrap_or(0),
            ]) as i64
        }
        Encoding::I32Le => {
            i32::from_le_bytes([
                data.first().copied().unwrap_or(0),
                data.get(1).copied().unwrap_or(0),
                data.get(2).copied().unwrap_or(0),
                data.get(3).copied().unwrap_or(0),
            ]) as i64
        }
        Encoding::U32Le => {
            u32::from_le_bytes([
                data.first().copied().unwrap_or(0),
                data.get(1).copied().unwrap_or(0),
                data.get(2).copied().unwrap_or(0),
                data.get(3).copied().unwrap_or(0),
            ]) as i64
        }
        Encoding::Reserved(_) => 0,
    }
}

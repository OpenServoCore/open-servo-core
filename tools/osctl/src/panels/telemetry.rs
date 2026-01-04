//! Telemetry panel - Real-time register streaming and graphs.

use crate::app::ConnectCommand;
use crate::rtt::{RpcClient, RpcResponse};
use egui_plot::{Line, Plot, PlotPoints};
use heapless::Vec as HVec;
use open_servo_registry::{find, Encoding, RegSpec};
use open_servo_rpc::{RegStreamFrame, RegStreamStartReq, RegStreamStopReq, KEY_REG_DATA};
use std::collections::VecDeque;
use std::sync::mpsc::Sender;

const HISTORY_SIZE: usize = 500;

/// Default telemetry register names (looked up from open-servo-registry).
const DEFAULT_TELEMETRY: &[&str] = &[
    "present_pos_cdeg",
    "present_current_ma",
    "present_temp_centic",
    "present_voltage_mv",
    "position_raw",
    "motor_current_a_raw",
    "vrefint_raw",
];

/// Resolved telemetry register with display metadata.
struct TelemetryRegister {
    spec: &'static RegSpec,
    display_name: String,
    scale: f64,
    unit: &'static str,
}

impl TelemetryRegister {
    fn from_spec(spec: &'static RegSpec) -> Self {
        let (display_name, scale, unit) = derive_display_meta(spec.name);
        Self {
            spec,
            display_name,
            scale,
            unit,
        }
    }
}

/// Derive display metadata from register name conventions.
fn derive_display_meta(name: &str) -> (String, f64, &'static str) {
    // Convert snake_case to Title Case for display
    let display_name = name
        .split('_')
        .filter(|s| !s.is_empty())
        .map(|s| {
            let mut chars = s.chars();
            match chars.next() {
                Some(c) => c.to_uppercase().chain(chars).collect::<String>(),
                None => String::new(),
            }
        })
        .collect::<Vec<_>>()
        .join(" ");

    // Derive scale and unit from suffix
    let (scale, unit) = if name.ends_with("_cdeg") {
        (100.0, "deg")
    } else if name.ends_with("_centic") {
        (100.0, "°C")
    } else if name.ends_with("_ma") {
        (1.0, "mA")
    } else if name.ends_with("_mv") {
        (1.0, "mV")
    } else if name.ends_with("_raw") {
        (1.0, "")
    } else if name.ends_with("_dps10") {
        (10.0, "deg/s")
    } else {
        (1.0, "")
    };

    (display_name, scale, unit)
}

/// Parse a value from bytes using the register's encoding.
fn parse_value(data: &[u8], encoding: Encoding) -> f64 {
    match encoding {
        Encoding::U8 => data[0] as f64,
        Encoding::Bool => data[0] as f64,
        Encoding::I16Le => i16::from_le_bytes([data[0], data[1]]) as f64,
        Encoding::U16Le => u16::from_le_bytes([data[0], data[1]]) as f64,
        Encoding::I32Le => i32::from_le_bytes([data[0], data[1], data[2], data[3]]) as f64,
        Encoding::U32Le => u32::from_le_bytes([data[0], data[1], data[2], data[3]]) as f64,
        Encoding::Enum(_) => data[0] as f64,
        Encoding::Reserved(_) => 0.0,
    }
}

pub struct TelemetryPanel {
    /// Resolved telemetry registers from the registry
    registers: Vec<TelemetryRegister>,
    /// Which registers are selected for streaming
    selected: Vec<bool>,
    rate_hz: u8,

    /// Data history for each register
    history: Vec<VecDeque<f64>>,

    /// Streaming state
    streaming: bool,
    seq: u16,
}

impl TelemetryPanel {
    pub fn new() -> Self {
        // Resolve default telemetry registers from the registry
        let registers: Vec<TelemetryRegister> = DEFAULT_TELEMETRY
            .iter()
            .filter_map(|name| find(name))
            .map(TelemetryRegister::from_spec)
            .collect();

        let num_regs = registers.len();
        Self {
            registers,
            // Default: first 4 registers selected (Position, Current, Temp, Voltage)
            selected: (0..num_regs).map(|i| i < 4).collect(),
            rate_hz: 50,
            history: (0..num_regs)
                .map(|_| VecDeque::with_capacity(HISTORY_SIZE))
                .collect(),
            streaming: false,
            seq: 0,
        }
    }

    pub fn handle_response(&mut self, resp: &RpcResponse) {
        if resp.key == KEY_REG_DATA {
            match postcard::from_bytes::<RegStreamFrame>(&resp.payload) {
                Ok(frame) => {
                    self.seq = frame.seq;
                    self.parse_frame(&frame.data);
                }
                Err(e) => {
                    tracing::warn!("Failed to parse RegStreamFrame: {:?}", e);
                }
            }
        }
    }

    fn parse_frame(&mut self, data: &[u8]) {
        let mut offset = 0;

        // Parse data in order of selected registers
        // Note: firmware sends 4 bytes per register regardless of actual size
        for (i, reg) in self.registers.iter().enumerate() {
            if !self.selected[i] {
                continue;
            }

            if offset + 4 > data.len() {
                break;
            }

            // Parse value using encoding from registry
            let val = parse_value(&data[offset..], reg.spec.encoding);

            // Apply scale and push to history
            let scaled = val / reg.scale;
            Self::push_history(&mut self.history[i], scaled);
            offset += 4; // Firmware always sends 4 bytes per register
        }
    }

    fn push_history(history: &mut VecDeque<f64>, val: f64) {
        if history.len() >= HISTORY_SIZE {
            history.pop_front();
        }
        history.push_back(val);
    }

    fn plot_data(history: &VecDeque<f64>) -> PlotPoints {
        PlotPoints::from_iter(history.iter().enumerate().map(|(i, &v)| [i as f64, v]))
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        is_connected: bool,
        cmd_tx: &Option<Sender<ConnectCommand>>,
        rpc_client: &mut RpcClient,
    ) {
        ui.heading("Telemetry");
        ui.add_space(10.0);

        // Register selection
        ui.horizontal(|ui| {
            ui.label("Registers:");
            for (i, reg) in self.registers.iter().enumerate() {
                ui.checkbox(&mut self.selected[i], &reg.display_name);
            }
        });

        ui.add_space(5.0);

        ui.horizontal(|ui| {
            ui.label("Rate:");
            ui.radio_value(&mut self.rate_hz, 10, "10 Hz");
            ui.radio_value(&mut self.rate_hz, 50, "50 Hz");
            ui.radio_value(&mut self.rate_hz, 100, "100 Hz");

            ui.separator();

            let btn_text = if self.streaming {
                "⏹ Stop"
            } else {
                "▶ Start"
            };
            let btn_enabled = is_connected;

            if ui
                .add_enabled(btn_enabled, egui::Button::new(btn_text))
                .clicked()
            {
                if self.streaming {
                    self.stop_streaming(cmd_tx, rpc_client);
                } else {
                    self.start_streaming(cmd_tx, rpc_client);
                }
                self.streaming = !self.streaming;
            }

            if ui.button("Clear").clicked() {
                for h in &mut self.history {
                    h.clear();
                }
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let first_len = self.history.first().map(|h| h.len()).unwrap_or(0);
                ui.label(format!("seq: {} | {} samples", self.seq, first_len));
            });
        });

        if !is_connected {
            ui.add_space(20.0);
            ui.colored_label(
                egui::Color32::GRAY,
                "Connect to a device to stream telemetry",
            );
            return;
        }

        ui.add_space(10.0);
        ui.separator();

        // Count selected graphs
        let num_graphs = self.selected.iter().filter(|&&s| s).count().max(1);
        let available_height = ui.available_height();
        let graph_height = ((available_height - 40.0) / num_graphs as f32).max(80.0);

        // Plot colors
        let colors = [
            egui::Color32::LIGHT_BLUE,
            egui::Color32::YELLOW,
            egui::Color32::RED,
            egui::Color32::GREEN,
            egui::Color32::LIGHT_GRAY,
            egui::Color32::KHAKI,
            egui::Color32::LIGHT_GREEN,
        ];

        for (i, reg) in self.registers.iter().enumerate() {
            if !self.selected[i] {
                continue;
            }

            let label = if reg.unit.is_empty() {
                reg.display_name.clone()
            } else {
                format!("{} ({})", reg.display_name, reg.unit)
            };
            ui.label(label);

            Plot::new(format!("plot_{}", reg.spec.address))
                .height(graph_height)
                .show_axes([false, true])
                .allow_drag(false)
                .allow_zoom(false)
                .show(ui, |plot_ui| {
                    let color = colors[i % colors.len()];
                    let line = Line::new(Self::plot_data(&self.history[i])).color(color);
                    plot_ui.line(line);
                });
        }
    }

    fn start_streaming(&self, cmd_tx: &Option<Sender<ConnectCommand>>, rpc_client: &mut RpcClient) {
        if let Some(ref tx) = cmd_tx {
            // Build address list from selected registers
            let mut addresses: HVec<u16, 16> = HVec::new();
            for (i, reg) in self.registers.iter().enumerate() {
                if self.selected[i] {
                    let _ = addresses.push(reg.spec.address);
                }
            }

            let req = RegStreamStartReq {
                rate_hz: self.rate_hz,
                addresses,
            };

            if let Ok((data, _seq)) = rpc_client.encode_request("reg/stream/start", &req) {
                let _ = tx.send(ConnectCommand::SendRpc(data));
            }
        }
    }

    fn stop_streaming(&self, cmd_tx: &Option<Sender<ConnectCommand>>, rpc_client: &mut RpcClient) {
        if let Some(ref tx) = cmd_tx {
            let req = RegStreamStopReq;
            if let Ok((data, _seq)) = rpc_client.encode_request("reg/stream/stop", &req) {
                let _ = tx.send(ConnectCommand::SendRpc(data));
            }
        }
    }
}

impl Default for TelemetryPanel {
    fn default() -> Self {
        Self::new()
    }
}

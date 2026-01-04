//! Control panel - Position dial, slider, engage toggle.

use crate::app::ConnectCommand;
use crate::rtt::{RpcClient, RpcResponse};
use heapless::Vec as HVec;
use open_servo_rpc::WriteRegReq;
use std::sync::mpsc::Sender;

// Registry addresses (from open-servo-registry vendor module)
const ADDR_GOAL_POS_CDEG: u16 = 512;
const ADDR_TORQUE_ENABLE: u16 = 581;

#[derive(Default)]
pub struct ControlPanel {
    // Target position in centidegrees
    goal_pos_cdeg: i32,
    // Current position (read from device)
    current_pos_cdeg: i32,
    // Engagement state
    engaged: bool,
    // Control mode
    mode: ControlMode,
    // Live telemetry values
    current_ma: i16,
    temp_centic: i16,
}

#[derive(Default, Clone, Copy, PartialEq)]
pub enum ControlMode {
    #[default]
    Position,
    OpenLoop,
}

impl ControlPanel {
    // Convert centidegrees to degrees for display
    fn cdeg_to_deg(cdeg: i32) -> f32 {
        cdeg as f32 / 100.0
    }

    // Convert degrees to centidegrees
    fn deg_to_cdeg(deg: f32) -> i32 {
        (deg * 100.0) as i32
    }

    pub fn handle_response(&mut self, resp: &RpcResponse) {
        // Handle reg/write responses
        let write_key = fnv1a_hash(b"reg/write");
        if resp.key == write_key {
            // Write acknowledged - could parse WriteRegResp if needed
        }
    }

    fn send_position(&self, cmd_tx: &Option<Sender<ConnectCommand>>, rpc_client: &mut RpcClient) {
        if let Some(ref tx) = cmd_tx {
            let req = WriteRegReq {
                addr: ADDR_GOAL_POS_CDEG,
                data: HVec::from_slice(&self.goal_pos_cdeg.to_le_bytes()).unwrap(),
            };
            if let Ok((data, _seq)) = rpc_client.encode_request("reg/write", &req) {
                let _ = tx.send(ConnectCommand::SendRpc(data));
            }
        }
    }

    fn send_engage(&self, cmd_tx: &Option<Sender<ConnectCommand>>, rpc_client: &mut RpcClient) {
        if let Some(ref tx) = cmd_tx {
            let val = if self.engaged { 1u8 } else { 0u8 };
            let req = WriteRegReq {
                addr: ADDR_TORQUE_ENABLE,
                data: HVec::from_slice(&[val]).unwrap(),
            };
            if let Ok((data, _seq)) = rpc_client.encode_request("reg/write", &req) {
                let _ = tx.send(ConnectCommand::SendRpc(data));
            }
        }
    }

    fn send_mode(&self, _cmd_tx: &Option<Sender<ConnectCommand>>, _rpc_client: &mut RpcClient) {
        // TODO: Add OPERATING_MODE address and implement
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        is_connected: bool,
        cmd_tx: &Option<Sender<ConnectCommand>>,
        rpc_client: &mut RpcClient,
    ) {
        ui.heading("Servo Control");
        ui.add_space(10.0);

        if !is_connected {
            ui.colored_label(egui::Color32::GRAY, "Connect to a device to control servo");
            return;
        }

        ui.columns(2, |cols| {
            // Left column: Position dial
            cols[0].group(|ui| {
                ui.heading("Position");
                ui.add_space(10.0);

                // Dial visualization (simplified circular representation)
                let dial_size = 200.0;
                let (response, painter) =
                    ui.allocate_painter(egui::vec2(dial_size, dial_size), egui::Sense::drag());

                let center = response.rect.center();
                let radius = dial_size / 2.0 - 10.0;

                // Draw dial background
                painter.circle_stroke(center, radius, egui::Stroke::new(2.0, egui::Color32::GRAY));

                // Draw tick marks
                for i in 0..12 {
                    let angle = (i as f32 * 30.0 - 90.0).to_radians();
                    let inner = center
                        + egui::vec2(angle.cos() * (radius - 15.0), angle.sin() * (radius - 15.0));
                    let outer = center
                        + egui::vec2(angle.cos() * (radius - 5.0), angle.sin() * (radius - 5.0));
                    painter.line_segment(
                        [inner, outer],
                        egui::Stroke::new(1.0, egui::Color32::DARK_GRAY),
                    );
                }

                // Draw goal position indicator (blue)
                let goal_angle = (Self::cdeg_to_deg(self.goal_pos_cdeg) - 90.0).to_radians();
                let goal_end = center
                    + egui::vec2(
                        goal_angle.cos() * (radius - 5.0),
                        goal_angle.sin() * (radius - 5.0),
                    );
                painter.line_segment(
                    [center, goal_end],
                    egui::Stroke::new(3.0, egui::Color32::LIGHT_BLUE),
                );

                // Draw current position (green, if different from goal)
                if (self.current_pos_cdeg - self.goal_pos_cdeg).abs() > 50 {
                    let cur_angle = (Self::cdeg_to_deg(self.current_pos_cdeg) - 90.0).to_radians();
                    let cur_end = center
                        + egui::vec2(
                            cur_angle.cos() * (radius - 20.0),
                            cur_angle.sin() * (radius - 20.0),
                        );
                    painter.line_segment(
                        [center, cur_end],
                        egui::Stroke::new(2.0, egui::Color32::GREEN),
                    );
                }

                // Handle drag - send position continuously while dragging
                if response.dragged() {
                    if let Some(pos) = response.interact_pointer_pos() {
                        let delta = pos - center;
                        let angle = delta.y.atan2(delta.x).to_degrees() + 90.0;
                        let angle_normalized = if angle < 0.0 { angle + 360.0 } else { angle };
                        // Map 0-360 to -180 to 180
                        let angle_centered = if angle_normalized > 180.0 {
                            angle_normalized - 360.0
                        } else {
                            angle_normalized
                        };
                        let new_pos = Self::deg_to_cdeg(angle_centered);
                        if new_pos != self.goal_pos_cdeg {
                            self.goal_pos_cdeg = new_pos;
                            self.send_position(cmd_tx, rpc_client);
                        }
                    }
                }

                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    ui.label("Target:");
                    ui.colored_label(
                        egui::Color32::LIGHT_BLUE,
                        format!("{:.2}°", Self::cdeg_to_deg(self.goal_pos_cdeg)),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Current:");
                    ui.colored_label(
                        egui::Color32::GREEN,
                        format!("{:.2}°", Self::cdeg_to_deg(self.current_pos_cdeg)),
                    );
                });

                // Fine adjustment slider - send position continuously while dragging
                ui.add_space(10.0);
                ui.label("Fine adjustment:");
                let mut deg = Self::cdeg_to_deg(self.goal_pos_cdeg);
                let slider_response =
                    ui.add(egui::Slider::new(&mut deg, -180.0..=180.0).suffix("°"));
                if slider_response.changed() {
                    self.goal_pos_cdeg = Self::deg_to_cdeg(deg);
                    self.send_position(cmd_tx, rpc_client);
                }
            });

            // Right column: Quick controls
            cols[1].group(|ui| {
                ui.heading("Quick Controls");
                ui.add_space(10.0);

                // Engage toggle
                let engage_text = if self.engaged {
                    "⚡ Engaged"
                } else {
                    "○ Disengaged"
                };
                let engage_color = if self.engaged {
                    egui::Color32::from_rgb(50, 150, 50)
                } else {
                    egui::Color32::from_rgb(80, 80, 80)
                };
                if ui
                    .add(
                        egui::Button::new(engage_text)
                            .fill(engage_color)
                            .min_size(egui::vec2(120.0, 30.0)),
                    )
                    .clicked()
                {
                    self.engaged = !self.engaged;
                    self.send_engage(cmd_tx, rpc_client);
                }

                ui.add_space(10.0);

                // Mode selector
                ui.label("Mode:");
                let old_mode = self.mode;
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.mode, ControlMode::Position, "Position");
                    ui.radio_value(&mut self.mode, ControlMode::OpenLoop, "Open Loop");
                });
                if self.mode != old_mode {
                    self.send_mode(cmd_tx, rpc_client);
                }

                ui.add_space(20.0);
                ui.separator();
                ui.add_space(10.0);

                // Live values
                ui.heading("Live Values");
                ui.add_space(10.0);

                egui::Grid::new("live_values")
                    .num_columns(2)
                    .spacing([20.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Position:");
                        ui.monospace(format!("{} cdeg", self.current_pos_cdeg));
                        ui.end_row();

                        ui.label("Current:");
                        ui.monospace(format!("{} mA", self.current_ma));
                        ui.end_row();

                        ui.label("Temperature:");
                        ui.monospace(format!("{:.1} °C", self.temp_centic as f32 / 100.0));
                        ui.end_row();
                    });

                ui.add_space(20.0);

                // Quick position buttons
                ui.label("Presets:");
                ui.horizontal(|ui| {
                    if ui.button("-90°").clicked() {
                        self.goal_pos_cdeg = -9000;
                        self.send_position(cmd_tx, rpc_client);
                    }
                    if ui.button("0°").clicked() {
                        self.goal_pos_cdeg = 0;
                        self.send_position(cmd_tx, rpc_client);
                    }
                    if ui.button("+90°").clicked() {
                        self.goal_pos_cdeg = 9000;
                        self.send_position(cmd_tx, rpc_client);
                    }
                });
            });
        });
    }
}

fn fnv1a_hash(data: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf29ce484222325;
    for &byte in data {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

//! Flash panel - ELF selection and flashing.

use crate::app::{ConnectCommand, ConnectRequest};
use crate::rtt::{DebugProbeInfo, RttSession};
use std::path::PathBuf;
use std::sync::mpsc::Sender;

pub struct FlashPanel {
    elf_path: Option<PathBuf>,
    flash_progress: Option<f32>,
    flash_status: Option<String>,
    /// Pending connection request (processed by app).
    pub pending_connect: Option<ConnectRequest>,
    /// Cached list of available probes.
    probes: Vec<DebugProbeInfo>,
    /// Selected probe index.
    selected_probe_idx: usize,
}

impl FlashPanel {
    /// Create with pre-enumerated probes (enumeration must happen on main thread before event loop).
    pub fn with_probes(probes: Vec<DebugProbeInfo>) -> Self {
        Self {
            elf_path: None,
            flash_progress: None,
            flash_status: None,
            pending_connect: None,
            probes,
            selected_probe_idx: 0,
        }
    }
}

impl FlashPanel {
    pub fn set_elf_path(&mut self, path: PathBuf) {
        self.elf_path = Some(path);
    }

    /// Refresh the list of available probes.
    pub fn refresh_probes(&mut self) {
        self.probes = RttSession::list_probes();
        // Keep selection valid
        if self.selected_probe_idx >= self.probes.len() {
            self.selected_probe_idx = 0;
        }
    }

    /// Get a probe by index.
    pub fn get_probe(&self, idx: usize) -> Option<&DebugProbeInfo> {
        self.probes.get(idx)
    }

    /// Get all probes.
    pub fn probes(&self) -> &[DebugProbeInfo] {
        &self.probes
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        is_connected: bool,
        cmd_tx: &Option<Sender<ConnectCommand>>,
    ) {
        ui.heading("Flash Firmware");
        ui.add_space(10.0);

        // Probe selection
        ui.horizontal(|ui| {
            ui.label("Probe:");

            let selected_text = if self.probes.is_empty() {
                "(no probes found)".to_string()
            } else if let Some(probe) = self.probes.get(self.selected_probe_idx) {
                format_probe_name(probe)
            } else {
                "(select probe)".to_string()
            };

            egui::ComboBox::from_id_salt("probe_select")
                .selected_text(&selected_text)
                .width(300.0)
                .show_ui(ui, |ui| {
                    for (idx, probe) in self.probes.iter().enumerate() {
                        let text = format_probe_name(probe);
                        ui.selectable_value(&mut self.selected_probe_idx, idx, text);
                    }
                });

            if ui.button("🔄").on_hover_text("Refresh probe list").clicked() {
                self.refresh_probes();
            }
        });

        ui.add_space(10.0);

        // ELF file selection
        ui.horizontal(|ui| {
            ui.label("ELF File:");
            if let Some(ref path) = self.elf_path {
                ui.monospace(
                    path.file_name()
                        .map(|s| s.to_string_lossy().to_string())
                        .unwrap_or_else(|| path.display().to_string()),
                );
            } else {
                ui.label("(none selected)");
            }

            if ui.button("Browse...").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("ELF", &["elf", ""])
                    .set_title("Select firmware ELF")
                    .pick_file()
                {
                    self.elf_path = Some(path);
                }
            }
        });

        if let Some(ref path) = self.elf_path {
            ui.label(format!("Full path: {}", path.display()));
        }

        ui.add_space(20.0);

        ui.horizontal(|ui| {
            let has_probe = !self.probes.is_empty();
            let can_flash = self.elf_path.is_some() && self.flash_progress.is_none() && !is_connected && has_probe;

            if ui
                .add_enabled(can_flash, egui::Button::new("🔥 Flash & Connect"))
                .clicked()
            {
                if let Some(ref path) = &self.elf_path {
                    self.flash_status = Some("Connecting...".to_string());
                    self.pending_connect = Some(ConnectRequest {
                        elf_path: Some(path.clone()),
                        chip: None,
                        probe_idx: Some(self.selected_probe_idx),
                    });
                }
            }

            if ui
                .add_enabled(!is_connected && has_probe, egui::Button::new("🔗 Attach Only"))
                .clicked()
            {
                self.flash_status = Some("Attaching...".to_string());
                self.pending_connect = Some(ConnectRequest {
                    elf_path: None,
                    chip: None,
                    probe_idx: Some(self.selected_probe_idx),
                });
            }

            if is_connected {
                if ui.button("⏹ Disconnect").clicked() {
                    if let Some(ref tx) = cmd_tx {
                        let _ = tx.send(ConnectCommand::Disconnect);
                    }
                }
            }
        });

        if let Some(ref status) = self.flash_status {
            ui.add_space(10.0);
            ui.label(status);
        }

        ui.add_space(20.0);
        ui.separator();

        ui.heading("Connection Info");
        ui.add_space(10.0);

        if is_connected {
            ui.colored_label(egui::Color32::GREEN, "✓ Connected to target");
        } else if self.probes.is_empty() {
            ui.colored_label(egui::Color32::YELLOW, "No probes detected. Connect a debugger and click 🔄");
        } else {
            ui.label("Select a probe and ELF file, then click Flash or Attach.");
        }
    }
}

/// Format a probe name for display.
fn format_probe_name(probe: &DebugProbeInfo) -> String {
    let serial = probe.serial_number.as_deref().unwrap_or("no-serial");
    format!("{} ({})", probe.identifier, serial)
}

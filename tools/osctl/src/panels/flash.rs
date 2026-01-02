//! Flash panel - ELF selection and flashing.

use crate::app::{ConnectCommand, ConnectRequest};
use std::path::PathBuf;
use std::sync::mpsc::Sender;

#[derive(Default)]
pub struct FlashPanel {
    elf_path: Option<PathBuf>,
    flash_progress: Option<f32>,
    flash_status: Option<String>,
    /// Pending connection request (processed by app).
    pub pending_connect: Option<ConnectRequest>,
}

impl FlashPanel {
    pub fn set_elf_path(&mut self, path: PathBuf) {
        self.elf_path = Some(path);
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        is_connected: bool,
        cmd_tx: &Option<Sender<ConnectCommand>>,
    ) {
        ui.heading("Flash Firmware");
        ui.add_space(10.0);

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
            let can_flash = self.elf_path.is_some() && self.flash_progress.is_none() && !is_connected;

            if ui
                .add_enabled(can_flash, egui::Button::new("🔥 Flash & Connect"))
                .clicked()
            {
                if let Some(ref path) = &self.elf_path {
                    self.flash_status = Some("Connecting...".to_string());
                    self.pending_connect = Some(ConnectRequest {
                        elf_path: Some(path.clone()),
                        chip: None,
                        probe: None,
                    });
                }
            }

            if ui
                .add_enabled(!is_connected, egui::Button::new("🔗 Attach Only"))
                .clicked()
            {
                self.flash_status = Some("Attaching...".to_string());
                self.pending_connect = Some(ConnectRequest {
                    elf_path: None,
                    chip: None,
                    probe: None,
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
        } else {
            ui.label("Not connected. Select an ELF file and click Flash, or click Attach.");
        }

        ui.add_space(20.0);

        // Probe list
        ui.collapsing("Available Probes", |ui| {
            let probes = crate::rtt::RttSession::list_probes();
            if probes.is_empty() {
                ui.label("No probes detected");
            } else {
                for probe in probes {
                    ui.label(format!("• {}", probe));
                }
            }
        });
    }
}

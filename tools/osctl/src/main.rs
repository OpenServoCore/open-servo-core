//! Open Servo Control Tool
//!
//! GUI for flashing, monitoring, and controlling open-servo devices.
//!
//! Usage:
//!   osctl <elf-path>     Flash firmware and open GUI
//!   osctl --attach       Attach to running target (no flash)

mod app;
mod panels;
mod rtt;

use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

/// Open Servo Control Tool
#[derive(Parser, Debug, Clone)]
#[command(name = "osctl", version, about)]
pub struct Args {
    /// Path to ELF file to flash (optional if --attach)
    #[arg(value_name = "ELF")]
    pub elf_path: Option<PathBuf>,

    /// Attach to running target without flashing
    #[arg(long)]
    pub attach: bool,

    /// Target chip (auto-detected if not specified)
    #[arg(long)]
    pub chip: Option<String>,

    /// Probe selector (e.g., "0483:374b:001")
    #[arg(long)]
    pub probe: Option<String>,
}

fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive(tracing::Level::INFO.into()),
        )
        .init();

    let args = Args::parse();

    if !args.attach && args.elf_path.is_none() {
        anyhow::bail!("Either provide an ELF path or use --attach");
    }

    // Run the GUI
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1024.0, 768.0])
            .with_min_inner_size([800.0, 600.0]),
        ..Default::default()
    };

    eframe::run_native(
        "osctl - Open Servo Control",
        options,
        Box::new(|cc| Ok(Box::new(app::OsctlApp::new(cc, args)))),
    )
    .map_err(|e| anyhow::anyhow!("eframe error: {}", e))
}

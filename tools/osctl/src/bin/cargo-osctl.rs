//! Cargo subcommand for osctl.
//!
//! Usage:
//!   cargo osctl run           Build firmware, flash, and open GUI
//!   cargo osctl run --release Release build
//!   cargo osctl attach        Attach without building/flashing

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::path::PathBuf;
use std::process::Command;

/// Cargo subcommand for Open Servo Control Tool
#[derive(Parser, Debug)]
#[command(name = "cargo-osctl", bin_name = "cargo osctl")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Build firmware, flash, and open GUI
    Run {
        /// Build in release mode
        #[arg(long)]
        release: bool,

        /// Target chip (auto-detected if not specified)
        #[arg(long)]
        chip: Option<String>,

        /// Probe selector
        #[arg(long)]
        probe: Option<String>,

        /// Package to build (default: open-servo-board-stm32f301)
        #[arg(short, long, default_value = "open-servo-board-stm32f301")]
        package: String,
    },
    /// Attach to running target without building/flashing
    Attach {
        /// Target chip (auto-detected if not specified)
        #[arg(long)]
        chip: Option<String>,

        /// Probe selector
        #[arg(long)]
        probe: Option<String>,
    },
}

fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive(tracing::Level::INFO.into()),
        )
        .init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Run {
            release,
            chip,
            probe,
            package,
        } => {
            // Build the firmware
            tracing::info!("Building {}...", package);
            let mut build_cmd = Command::new("cargo");
            build_cmd.arg("build").arg("-p").arg(&package);
            if release {
                build_cmd.arg("--release");
            }

            let status = build_cmd.status().context("Failed to run cargo build")?;
            if !status.success() {
                anyhow::bail!("cargo build failed");
            }

            // Find the ELF file
            let profile = if release { "release" } else { "dev" };
            let target_dir = find_target_dir()?;
            let elf_path = target_dir
                .join("thumbv7em-none-eabihf")
                .join(profile)
                .join(&package);

            if !elf_path.exists() {
                anyhow::bail!("ELF not found at {}", elf_path.display());
            }

            tracing::info!("Built: {}", elf_path.display());

            // Launch osctl with the ELF
            let mut osctl_args = vec![elf_path.to_string_lossy().to_string()];
            if let Some(c) = chip {
                osctl_args.push("--chip".to_string());
                osctl_args.push(c);
            }
            if let Some(p) = probe {
                osctl_args.push("--probe".to_string());
                osctl_args.push(p);
            }

            // Execute osctl (either installed or from this build)
            let osctl_exe = std::env::current_exe()?
                .parent()
                .unwrap()
                .join(if cfg!(windows) { "osctl.exe" } else { "osctl" });

            let status = Command::new(&osctl_exe)
                .args(&osctl_args)
                .status()
                .with_context(|| format!("Failed to run osctl at {}", osctl_exe.display()))?;

            if !status.success() {
                anyhow::bail!("osctl exited with error");
            }
        }
        Commands::Attach { chip, probe } => {
            // Launch osctl in attach mode
            let mut osctl_args = vec!["--attach".to_string()];
            if let Some(c) = chip {
                osctl_args.push("--chip".to_string());
                osctl_args.push(c);
            }
            if let Some(p) = probe {
                osctl_args.push("--probe".to_string());
                osctl_args.push(p);
            }

            let osctl_exe = std::env::current_exe()?
                .parent()
                .unwrap()
                .join(if cfg!(windows) { "osctl.exe" } else { "osctl" });

            let status = Command::new(&osctl_exe)
                .args(&osctl_args)
                .status()
                .with_context(|| format!("Failed to run osctl at {}", osctl_exe.display()))?;

            if !status.success() {
                anyhow::bail!("osctl exited with error");
            }
        }
    }

    Ok(())
}

fn find_target_dir() -> Result<PathBuf> {
    // Try to find target directory by looking for Cargo.toml
    let mut dir = std::env::current_dir()?;
    loop {
        let cargo_toml = dir.join("Cargo.toml");
        if cargo_toml.exists() {
            let target = dir.join("target");
            if target.exists() {
                return Ok(target);
            }
            // Check for workspace target dir
            let workspace_target = dir.parent().and_then(|p| {
                let t = p.join("target");
                if t.exists() {
                    Some(t)
                } else {
                    None
                }
            });
            if let Some(t) = workspace_target {
                return Ok(t);
            }
            return Ok(target);
        }
        if !dir.pop() {
            break;
        }
    }
    anyhow::bail!("Could not find target directory")
}

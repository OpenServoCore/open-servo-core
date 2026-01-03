//! RTT session management using probe-rs.

use anyhow::{Context, Result};
use probe_rs::flashing::{DownloadOptions, FlashProgress as ProbeFlashProgress, Format, ProgressEvent};
use probe_rs::probe::list::Lister;
pub use probe_rs::probe::DebugProbeInfo;
use probe_rs::rtt::Rtt;
use probe_rs::{Permissions, Session};
use std::cell::Cell;
use std::path::Path;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crate::app::{FlashPhase, FlashProgress};

/// RTT channel indices.
pub mod channels {
    /// defmt log output (target → host)
    pub const DEFMT_UP: usize = 0;
    /// RPC responses + telemetry (target → host)
    pub const RPC_UP: usize = 1;
    /// RPC requests (host → target)
    pub const RPC_DOWN: usize = 0;
}

/// Commands sent to the RTT polling thread.
pub enum RttCommand {
    /// Send data on RPC down channel.
    SendRpc(Vec<u8>),
    /// Stop the polling thread.
    Stop,
}

/// Events from the RTT polling thread.
pub enum RttEvent {
    /// defmt log data received.
    DefmtData(Vec<u8>),
    /// RPC response/telemetry data received.
    RpcData(Vec<u8>),
    /// Connection lost.
    Disconnected(String),
}

/// RTT session wrapper.
pub struct RttSession {
    /// Handle to polling thread.
    poll_thread: Option<JoinHandle<()>>,
    /// Channel to send commands to polling thread.
    cmd_tx: Sender<RttCommand>,
    /// Channel to receive events from polling thread.
    event_rx: Receiver<RttEvent>,
    /// Chip name.
    chip: String,
}

impl RttSession {
    /// List available debug probes (must be called from main thread on macOS).
    pub fn list_probes() -> Vec<DebugProbeInfo> {
        let lister = Lister::new();
        lister.list_all()
    }

    /// Select a probe by selector string, or return the first one.
    pub fn select_probe(probes: Vec<DebugProbeInfo>, selector: Option<&str>) -> Result<DebugProbeInfo> {
        if probes.is_empty() {
            anyhow::bail!("No debug probes found");
        }

        if let Some(selector) = selector {
            probes
                .into_iter()
                .find(|p| {
                    p.identifier.contains(selector)
                        || p.serial_number
                            .as_ref()
                            .map(|s| s.contains(selector))
                            .unwrap_or(false)
                })
                .context("No matching probe found")
        } else {
            Ok(probes.into_iter().next().unwrap())
        }
    }

    /// Connect to a pre-selected probe and optionally flash firmware.
    ///
    /// Note: On macOS, probe enumeration (list_probes/select_probe) must be done
    /// on the main thread. This function can be called from any thread.
    pub fn connect(
        probe_info: DebugProbeInfo,
        elf_path: Option<&Path>,
        chip: Option<&str>,
        progress_tx: Option<&Sender<FlashProgress>>,
    ) -> Result<Self> {
        let probe = probe_info.open().context("Failed to open probe")?;

        // Determine chip
        let chip_name = chip
            .map(String::from)
            .or_else(|| {
                // Try to auto-detect from ELF or use default
                Some("STM32F301K8Tx".to_string())
            })
            .unwrap();

        // Create session
        let mut session = probe
            .attach(&chip_name, Permissions::default())
            .context("Failed to attach to target")?;

        // Flash if ELF provided
        if let Some(elf) = elf_path {
            // Track progress state (Cell for interior mutability in Fn closure)
            let erase_done = Cell::new(0u64);
            let program_total = Cell::new(0u64);
            let program_done = Cell::new(0u64);

            let progress_tx_clone = progress_tx.cloned();
            let progress = ProbeFlashProgress::new(move |event| {
                if let Some(ref tx) = progress_tx_clone {
                    match event {
                        ProgressEvent::Initialized { .. } => {
                            let _ = tx.send(FlashProgress {
                                phase: FlashPhase::Erasing,
                                progress: 0.0,
                                message: String::new(),
                            });
                        }
                        ProgressEvent::StartedErasing => {
                            erase_done.set(0);
                        }
                        ProgressEvent::SectorErased { size, .. } => {
                            let done = erase_done.get() + size;
                            erase_done.set(done);
                            let _ = tx.send(FlashProgress {
                                phase: FlashPhase::Erasing,
                                progress: 0.5, // Indeterminate - we don't know total
                                message: format!("{} bytes erased", done),
                            });
                        }
                        ProgressEvent::FinishedErasing => {
                            let _ = tx.send(FlashProgress {
                                phase: FlashPhase::Erasing,
                                progress: 1.0,
                                message: "Erase complete".to_string(),
                            });
                        }
                        ProgressEvent::StartedProgramming { length } => {
                            program_total.set(length);
                            program_done.set(0);
                            let _ = tx.send(FlashProgress {
                                phase: FlashPhase::Programming,
                                progress: 0.0,
                                message: format!("0 / {} bytes", length),
                            });
                        }
                        ProgressEvent::PageProgrammed { size, .. } => {
                            let done = program_done.get() + size as u64;
                            program_done.set(done);
                            let total = program_total.get();
                            let progress_val = if total > 0 {
                                (done as f32 / total as f32).min(1.0)
                            } else {
                                0.5
                            };
                            let _ = tx.send(FlashProgress {
                                phase: FlashPhase::Programming,
                                progress: progress_val,
                                message: format!("{} / {} bytes", done, total),
                            });
                        }
                        ProgressEvent::FinishedProgramming => {
                            let _ = tx.send(FlashProgress {
                                phase: FlashPhase::Programming,
                                progress: 1.0,
                                message: "Programming complete".to_string(),
                            });
                        }
                        _ => {}
                    }
                }
            });

            let mut options = DownloadOptions::default();
            options.progress = Some(progress);

            probe_rs::flashing::download_file_with_options(&mut session, elf, Format::Elf, options)
                .context("Failed to flash firmware")?;
        }

        // Reset and run
        {
            let mut core = session.core(0).context("Failed to access core")?;
            core.reset().context("Failed to reset target")?;
        }

        // Set up channels for thread communication
        let (cmd_tx, cmd_rx) = mpsc::channel();
        let (event_tx, event_rx) = mpsc::channel();

        let chip_clone = chip_name.clone();

        // Spawn polling thread
        let poll_thread = thread::spawn(move || {
            if let Err(e) = rtt_poll_loop(session, cmd_rx, event_tx) {
                tracing::error!("RTT poll loop error: {}", e);
            }
        });

        Ok(Self {
            poll_thread: Some(poll_thread),
            cmd_tx,
            event_rx,
            chip: chip_clone,
        })
    }

    /// Get chip name.
    pub fn chip(&self) -> &str {
        &self.chip
    }

    /// Send RPC data to the target.
    pub fn send_rpc(&self, data: Vec<u8>) -> Result<()> {
        self.cmd_tx
            .send(RttCommand::SendRpc(data))
            .map_err(|_| anyhow::anyhow!("RTT thread disconnected"))
    }

    /// Poll for events (non-blocking).
    pub fn poll_events(&self) -> Vec<RttEvent> {
        let mut events = Vec::new();
        while let Ok(event) = self.event_rx.try_recv() {
            events.push(event);
        }
        events
    }

}

impl Drop for RttSession {
    fn drop(&mut self) {
        let _ = self.cmd_tx.send(RttCommand::Stop);
        if let Some(thread) = self.poll_thread.take() {
            let _ = thread.join();
        }
    }
}

/// RTT polling loop running in background thread.
fn rtt_poll_loop(
    mut session: Session,
    cmd_rx: Receiver<RttCommand>,
    event_tx: Sender<RttEvent>,
) -> Result<()> {
    let mut core = session.core(0)?;

    // Attach RTT with retry
    let mut rtt = None;
    for attempt in 0..50 {
        match Rtt::attach(&mut core) {
            Ok(r) => {
                rtt = Some(r);
                break;
            }
            Err(_) if attempt < 49 => {
                thread::sleep(Duration::from_millis(100));
            }
            Err(e) => {
                anyhow::bail!("Failed to attach RTT: {}", e);
            }
        }
    }

    let mut rtt = rtt.context("RTT not found")?;

    // Buffers for reading
    let mut defmt_buf = [0u8; 1024];
    let mut rpc_buf = [0u8; 1024];

    loop {
        // Check for commands
        match cmd_rx.try_recv() {
            Ok(RttCommand::Stop) => {
                break;
            }
            Ok(RttCommand::SendRpc(data)) => {
                if let Some(down) = rtt.down_channel(channels::RPC_DOWN) {
                    if let Err(e) = down.write(&mut core, &data) {
                        tracing::error!("Failed to write RPC: {}", e);
                    }
                }
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => break,
        }

        // Read defmt channel
        if let Some(up) = rtt.up_channel(channels::DEFMT_UP) {
            match up.read(&mut core, &mut defmt_buf) {
                Ok(n) if n > 0 => {
                    let _ = event_tx.send(RttEvent::DefmtData(defmt_buf[..n].to_vec()));
                }
                Ok(_) => {}
                Err(e) => {
                    let _ = event_tx.send(RttEvent::Disconnected(e.to_string()));
                    break;
                }
            }
        }

        // Read RPC channel
        if let Some(up) = rtt.up_channel(channels::RPC_UP) {
            match up.read(&mut core, &mut rpc_buf) {
                Ok(n) if n > 0 => {
                    let _ = event_tx.send(RttEvent::RpcData(rpc_buf[..n].to_vec()));
                }
                Ok(_) => {}
                Err(e) => {
                    let _ = event_tx.send(RttEvent::Disconnected(e.to_string()));
                    break;
                }
            }
        }

        // Small sleep to avoid busy-looping
        thread::sleep(Duration::from_millis(1));
    }

    // Clean up: detach RTT and reset target to leave probe in good state
    drop(rtt);
    let _ = core.reset();

    Ok(())
}

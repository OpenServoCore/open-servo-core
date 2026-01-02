//! RTT session management using probe-rs.

use anyhow::{Context, Result};
use probe_rs::flashing::{download_file, Format};
use probe_rs::probe::list::Lister;
use probe_rs::rtt::Rtt;
use probe_rs::{Permissions, Session};
use std::path::Path;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// RTT channel indices.
pub mod channels {
    /// defmt log output (target → host)
    pub const DEFMT_UP: usize = 0;
    /// RPC responses + telemetry (target → host)
    pub const RPC_UP: usize = 2;
    /// RPC requests (host → target)
    pub const RPC_DOWN: usize = 1;
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
    /// List available debug probes.
    pub fn list_probes() -> Vec<String> {
        let lister = Lister::new();
        lister
            .list_all()
            .into_iter()
            .map(|p| format!("{} ({})", p.identifier, p.serial_number.unwrap_or_default()))
            .collect()
    }

    /// Connect to a probe and optionally flash firmware.
    pub fn connect(
        elf_path: Option<&Path>,
        chip: Option<&str>,
        probe_selector: Option<&str>,
    ) -> Result<Self> {
        let lister = Lister::new();
        let probes = lister.list_all();

        if probes.is_empty() {
            anyhow::bail!("No debug probes found");
        }

        // Select probe
        let probe_info = if let Some(selector) = probe_selector {
            probes
                .into_iter()
                .find(|p| {
                    p.identifier.contains(selector)
                        || p.serial_number
                            .as_ref()
                            .map(|s| s.contains(selector))
                            .unwrap_or(false)
                })
                .context("No matching probe found")?
        } else {
            probes.into_iter().next().unwrap()
        };

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
            download_file(&mut session, elf, Format::Elf).context("Failed to flash firmware")?;
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

//! Main application state and UI.

use crate::panels::{ControlPanel, FlashPanel, RegistryPanel, TelemetryPanel};
use crate::rtt::{DefmtDecoder, RpcClient, RttEvent, RttSession};
use crate::Args;
use egui::Context;
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

/// Active panel tab.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ActivePanel {
    #[default]
    Flash,
    Registry,
    Telemetry,
    Control,
}

/// Connection state.
#[derive(Debug, Clone)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected {
        chip: String,
        version: Option<(u8, u8, u8)>,
    },
    Error(String),
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self::Disconnected
    }
}

/// A log entry from defmt.
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: String,
    pub level: LogLevel,
    pub module: String,
    pub message: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

impl LogLevel {
    pub fn color(&self) -> egui::Color32 {
        match self {
            LogLevel::Trace => egui::Color32::GRAY,
            LogLevel::Debug => egui::Color32::LIGHT_BLUE,
            LogLevel::Info => egui::Color32::GREEN,
            LogLevel::Warn => egui::Color32::YELLOW,
            LogLevel::Error => egui::Color32::RED,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            LogLevel::Trace => "TRACE",
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO",
            LogLevel::Warn => "WARN",
            LogLevel::Error => "ERROR",
        }
    }
}

/// A request to connect (stored in FlashPanel, processed by app).
#[derive(Clone)]
pub struct ConnectRequest {
    pub elf_path: Option<PathBuf>,
    pub chip: Option<String>,
    pub probe: Option<String>,
}

/// Commands to the connection thread.
pub enum ConnectCommand {
    Disconnect,
    SendRpc(Vec<u8>),
}

/// Events from the connection thread.
pub enum ConnectEvent {
    Connected { chip: String },
    Disconnected,
    Error(String),
    Log(LogEntry),
    RpcResponse(Vec<u8>),
}

/// Main application.
pub struct OsctlApp {
    active_panel: ActivePanel,
    connection_state: ConnectionState,

    // Communication with connection thread
    cmd_tx: Option<Sender<ConnectCommand>>,
    event_rx: Option<Receiver<ConnectEvent>>,

    // Panels
    flash_panel: FlashPanel,
    registry_panel: RegistryPanel,
    telemetry_panel: TelemetryPanel,
    control_panel: ControlPanel,

    // Logs
    logs: Vec<LogEntry>,
    log_filter: LogFilter,

    // RPC
    rpc_client: RpcClient,
}

#[derive(Clone)]
pub struct LogFilter {
    pub trace: bool,
    pub debug: bool,
    pub info: bool,
    pub warn: bool,
    pub error: bool,
    pub auto_scroll: bool,
}

impl Default for LogFilter {
    fn default() -> Self {
        Self {
            trace: false,
            debug: true,
            info: true,
            warn: true,
            error: true,
            auto_scroll: true,
        }
    }
}

impl OsctlApp {
    pub fn new(_cc: &eframe::CreationContext<'_>, args: Args) -> Self {
        let mut app = Self {
            active_panel: ActivePanel::default(),
            connection_state: ConnectionState::default(),
            cmd_tx: None,
            event_rx: None,
            flash_panel: FlashPanel::default(),
            registry_panel: RegistryPanel::default(),
            telemetry_panel: TelemetryPanel::default(),
            control_panel: ControlPanel::default(),
            logs: Vec::new(),
            log_filter: LogFilter::default(),
            rpc_client: RpcClient::new(),
        };

        // If an ELF path was provided, set it in the flash panel
        if let Some(ref elf) = args.elf_path {
            app.flash_panel.set_elf_path(elf.clone());
        }

        // Auto-connect if ELF provided or attach mode
        if args.elf_path.is_some() || args.attach {
            app.start_connection(args.elf_path, args.chip, args.probe);
        }

        app
    }

    fn start_connection(
        &mut self,
        elf_path: Option<PathBuf>,
        chip: Option<String>,
        probe: Option<String>,
    ) {
        let (cmd_tx, cmd_rx) = mpsc::channel();
        let (event_tx, event_rx) = mpsc::channel();

        self.cmd_tx = Some(cmd_tx.clone());
        self.event_rx = Some(event_rx);
        self.connection_state = ConnectionState::Connecting;

        let elf_for_decoder = elf_path.clone();

        // Spawn connection thread
        thread::spawn(move || {
            connection_thread(cmd_rx, event_tx, elf_path, elf_for_decoder, chip, probe);
        });
    }

    fn process_events(&mut self) {
        // Collect events first to avoid borrow conflict
        let events: Vec<_> = self
            .event_rx
            .as_ref()
            .map(|rx| {
                let mut events = Vec::new();
                while let Ok(event) = rx.try_recv() {
                    events.push(event);
                }
                events
            })
            .unwrap_or_default();

        // Now process collected events
        for event in events {
            match event {
                ConnectEvent::Connected { chip } => {
                    self.connection_state = ConnectionState::Connected {
                        chip,
                        version: None,
                    };
                    self.add_log(LogLevel::Info, "system", "Connected to target");
                }
                ConnectEvent::Disconnected => {
                    self.connection_state = ConnectionState::Disconnected;
                    self.add_log(LogLevel::Info, "system", "Disconnected");
                }
                ConnectEvent::Error(msg) => {
                    self.connection_state = ConnectionState::Error(msg.clone());
                    self.add_log(LogLevel::Error, "system", &msg);
                }
                ConnectEvent::Log(entry) => {
                    self.logs.push(entry);
                    // Keep log size bounded
                    if self.logs.len() > 10000 {
                        self.logs.drain(0..1000);
                    }
                }
                ConnectEvent::RpcResponse(data) => {
                    // Handle RPC responses
                    self.handle_rpc_response(&data);
                }
            }
        }
    }

    fn handle_rpc_response(&mut self, data: &[u8]) {
        let responses = self.rpc_client.decode_responses(data);
        for resp in &responses {
            self.registry_panel.handle_response(resp);
            self.control_panel.handle_response(resp);
            self.telemetry_panel.handle_response(resp);
        }
    }

    fn add_log(&mut self, level: LogLevel, module: &str, message: &str) {
        self.logs.push(LogEntry {
            timestamp: chrono_lite_now(),
            level,
            module: module.to_string(),
            message: message.to_string(),
        });
    }

    fn connection_status_ui(&self, ui: &mut egui::Ui) {
        match &self.connection_state {
            ConnectionState::Disconnected => {
                ui.colored_label(egui::Color32::GRAY, "⚫ Disconnected");
            }
            ConnectionState::Connecting => {
                ui.colored_label(egui::Color32::YELLOW, "🔄 Connecting...");
            }
            ConnectionState::Connected { chip, version } => {
                let ver_str = version
                    .map(|(ma, mi, pa)| format!(" v{}.{}.{}", ma, mi, pa))
                    .unwrap_or_default();
                ui.colored_label(
                    egui::Color32::GREEN,
                    format!("🟢 {}{}", chip, ver_str),
                );
            }
            ConnectionState::Error(e) => {
                ui.colored_label(egui::Color32::RED, format!("🔴 {}", e));
            }
        }
    }
}

impl eframe::App for OsctlApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        // Process events from connection thread
        self.process_events();

        // Process pending connect request from flash panel
        if let Some(req) = self.flash_panel.pending_connect.take() {
            if !matches!(self.connection_state, ConnectionState::Connected { .. } | ConnectionState::Connecting) {
                self.start_connection(req.elf_path, req.chip, req.probe);
            }
        }

        // Top panel with tabs and connection status
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("osctl");
                ui.separator();

                // Panel tabs
                ui.selectable_value(&mut self.active_panel, ActivePanel::Flash, "Flash");
                ui.selectable_value(&mut self.active_panel, ActivePanel::Registry, "Registry");
                ui.selectable_value(&mut self.active_panel, ActivePanel::Telemetry, "Telemetry");
                ui.selectable_value(&mut self.active_panel, ActivePanel::Control, "Control");

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    self.connection_status_ui(ui);
                });
            });
        });

        // Bottom panel with logs
        egui::TopBottomPanel::bottom("logs_panel")
            .resizable(true)
            .min_height(100.0)
            .default_height(180.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Logs");
                    ui.separator();

                    // Level filters
                    let filter = &mut self.log_filter;
                    ui.toggle_value(&mut filter.trace, "T")
                        .on_hover_text("Trace");
                    ui.toggle_value(&mut filter.debug, "D")
                        .on_hover_text("Debug");
                    ui.toggle_value(&mut filter.info, "I")
                        .on_hover_text("Info");
                    ui.toggle_value(&mut filter.warn, "W")
                        .on_hover_text("Warn");
                    ui.toggle_value(&mut filter.error, "E")
                        .on_hover_text("Error");

                    ui.separator();

                    if ui.button("Clear").clicked() {
                        self.logs.clear();
                    }

                    ui.toggle_value(&mut self.log_filter.auto_scroll, "⬇")
                        .on_hover_text("Auto-scroll");

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(format!("{} entries", self.logs.len()));
                    });
                });

                ui.separator();

                // Log content
                egui::ScrollArea::vertical()
                    .auto_shrink([false; 2])
                    .stick_to_bottom(self.log_filter.auto_scroll)
                    .show(ui, |ui| {
                        for entry in self.logs.iter() {
                            let show = match entry.level {
                                LogLevel::Trace => self.log_filter.trace,
                                LogLevel::Debug => self.log_filter.debug,
                                LogLevel::Info => self.log_filter.info,
                                LogLevel::Warn => self.log_filter.warn,
                                LogLevel::Error => self.log_filter.error,
                            };
                            if show {
                                ui.horizontal(|ui| {
                                    ui.monospace(&entry.timestamp);
                                    ui.colored_label(
                                        entry.level.color(),
                                        format!("{:5}", entry.level.label()),
                                    );
                                    if !entry.module.is_empty() {
                                        ui.colored_label(egui::Color32::DARK_GRAY, &entry.module);
                                    }
                                    ui.monospace(&entry.message);
                                });
                            }
                        }
                    });
            });

        // Central panel with active panel content
        egui::CentralPanel::default().show(ctx, |ui| {
            let is_connected = matches!(self.connection_state, ConnectionState::Connected { .. });
            let cmd_tx = self.cmd_tx.clone();

            match self.active_panel {
                ActivePanel::Flash => {
                    self.flash_panel.ui(ui, is_connected, &cmd_tx);
                }
                ActivePanel::Registry => {
                    self.registry_panel.ui(ui, is_connected, &cmd_tx, &mut self.rpc_client);
                }
                ActivePanel::Telemetry => {
                    self.telemetry_panel.ui(ui, is_connected, &cmd_tx, &mut self.rpc_client);
                }
                ActivePanel::Control => {
                    self.control_panel.ui(ui, is_connected, &cmd_tx, &mut self.rpc_client);
                }
            }
        });

        // Request continuous repaint for real-time updates
        ctx.request_repaint();
    }
}

/// Connection thread - handles probe-rs session and RTT communication.
fn connection_thread(
    cmd_rx: Receiver<ConnectCommand>,
    event_tx: Sender<ConnectEvent>,
    elf_path: Option<PathBuf>,
    elf_for_decoder: Option<PathBuf>,
    chip: Option<String>,
    probe: Option<String>,
) {
    // Try to connect
    let session = match RttSession::connect(
        elf_path.as_deref(),
        chip.as_deref(),
        probe.as_deref(),
    ) {
        Ok(s) => s,
        Err(e) => {
            let _ = event_tx.send(ConnectEvent::Error(e.to_string()));
            return;
        }
    };

    let _ = event_tx.send(ConnectEvent::Connected {
        chip: session.chip().to_string(),
    });

    // Try to create defmt decoder
    let mut defmt_decoder = elf_for_decoder
        .as_ref()
        .and_then(|p| DefmtDecoder::from_elf(p).ok());

    // Main loop
    loop {
        // Check for commands
        match cmd_rx.try_recv() {
            Ok(ConnectCommand::Disconnect) => {
                let _ = event_tx.send(ConnectEvent::Disconnected);
                break;
            }
            Ok(ConnectCommand::SendRpc(data)) => {
                if let Err(e) = session.send_rpc(data) {
                    let _ = event_tx.send(ConnectEvent::Error(e.to_string()));
                }
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => break,
        }

        // Poll RTT events
        for event in session.poll_events() {
            match event {
                RttEvent::DefmtData(data) => {
                    if let Some(ref mut decoder) = defmt_decoder {
                        for entry in decoder.decode(&data) {
                            let _ = event_tx.send(ConnectEvent::Log(entry));
                        }
                    }
                }
                RttEvent::RpcData(data) => {
                    let _ = event_tx.send(ConnectEvent::RpcResponse(data));
                }
                RttEvent::Disconnected(msg) => {
                    let _ = event_tx.send(ConnectEvent::Error(msg));
                    let _ = event_tx.send(ConnectEvent::Disconnected);
                    return;
                }
            }
        }

        // Small sleep
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

/// Simple timestamp (no chrono dependency).
fn chrono_lite_now() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let secs = now.as_secs() % 86400; // Seconds since midnight
    let hours = secs / 3600;
    let mins = (secs % 3600) / 60;
    let secs = secs % 60;
    let millis = now.subsec_millis();
    format!("{:02}:{:02}:{:02}.{:03}", hours, mins, secs, millis)
}

//! Debug shell / REPL for interactive debugging.
//!
//! This module provides a line-oriented command shell that runs in Tier 3
//! (main loop), generic over the underlying debug I/O transport.

use core::fmt::Write;
use heapless::String;

use crate::App;
use open_servo_control::ControlLoop;
use open_servo_hw::{BdcMotorDriver, DebugIo};

/// Maximum bytes to read per poll() call to prevent starvation of other tasks.
const MAX_BYTES_PER_POLL: usize = 32;

/// Line-oriented debug shell.
///
/// Collects input bytes into a line buffer and executes commands when
/// a newline is received. Designed to run in the main loop (Tier 3).
pub struct DebugShell<D> {
    io: D,
    line_buf: String<128>,
}

impl<D: DebugIo> DebugShell<D> {
    /// Create a new debug shell with the given I/O backend.
    pub fn new(io: D) -> Self {
        Self {
            io,
            line_buf: String::new(),
        }
    }

    /// Pump the debug shell: read input, parse commands, execute them.
    ///
    /// This should be called from the main loop. It reads up to
    /// MAX_BYTES_PER_POLL bytes per call to avoid starving other tasks.
    pub fn poll<C, H>(&mut self, app: &mut App<C>, hw: &mut H)
    where
        C: ControlLoop,
        H: BdcMotorDriver,
    {
        for _ in 0..MAX_BYTES_PER_POLL {
            let Some(b) = self.io.try_read() else { break };

            if self.push_byte(b) {
                // Got a complete line
                let line = self.take_line();
                self.handle_line(app, hw, &line);
            }
        }
    }

    /// Push a byte into the line buffer.
    /// Returns true if the line is complete (newline received).
    fn push_byte(&mut self, b: u8) -> bool {
        match b {
            b'\r' => false, // Ignore carriage return
            b'\n' => true,  // Line complete
            _ => {
                // Ignore if buffer is full
                let _ = self.line_buf.push(b as char);
                false
            }
        }
    }

    /// Take the current line buffer contents, leaving it empty.
    fn take_line(&mut self) -> String<128> {
        let mut out: String<128> = String::new();
        core::mem::swap(&mut out, &mut self.line_buf);
        out
    }

    /// Handle a complete command line.
    fn handle_line<C, H>(&mut self, app: &mut App<C>, hw: &mut H, line: &str)
    where
        C: ControlLoop,
        H: BdcMotorDriver,
    {
        let line = line.trim();
        if line.is_empty() {
            return;
        }

        let mut parts = line.split_whitespace();
        match parts.next() {
            Some("help") | Some("?") => self.cmd_help(),
            Some("state") | Some("s") => self.cmd_state(app),
            Some("fault") => self.cmd_fault(app, hw, parts.next()),
            Some("set") => self.cmd_set(app, parts),
            Some(cmd) => {
                self.println("unknown command");
                let mut buf: String<64> = String::new();
                let _ = write!(buf, "got: {}", cmd);
                self.println(&buf);
            }
            None => {}
        }
    }

    // --- Commands ---

    fn cmd_help(&mut self) {
        self.println("debug shell commands:");
        self.println("  help, ?       - show this help");
        self.println("  state, s      - show system state");
        self.println("  fault clear   - clear latched fault");
        self.println("  set sp <val>  - set setpoint (centidegrees)");
    }

    fn cmd_state<C: ControlLoop>(&mut self, app: &App<C>) {
        let s = app.get_system_state();
        let mut buf: String<96> = String::new();

        // Format: sp=XXX pos=XXX pwm=XXX I=XXXmA V=XXXmV T=XXXdC
        let _ = write!(
            buf,
            "sp={} pos={} pwm={} I={}mA V={}mV",
            s.setpoint.as_cdeg(),
            s.position.as_cdeg(),
            s.pwm_duty,
            s.current.as_ma(),
            s.bus_voltage.as_mv(),
        );
        self.println(&buf);

        // Temperature on separate line if available
        if let Some(temp) = s.temperature {
            buf.clear();
            let _ = write!(buf, "T={}dC", temp.as_dc());
            self.println(&buf);
        }

        // Fault status
        if app.is_faulted() {
            self.println("FAULT: latched");
        } else {
            self.println("fault: none");
        }
    }

    fn cmd_fault<C, H>(&mut self, app: &mut App<C>, hw: &mut H, subcmd: Option<&str>)
    where
        C: ControlLoop,
        H: BdcMotorDriver,
    {
        match subcmd {
            Some("clear") => {
                app.clear_fault(hw);
                self.println("fault cleared");
            }
            Some(other) => {
                let mut buf: String<64> = String::new();
                let _ = write!(buf, "unknown: fault {}", other);
                self.println(&buf);
                self.println("usage: fault clear");
            }
            None => {
                if app.is_faulted() {
                    self.println("FAULT: latched");
                } else {
                    self.println("fault: none");
                }
            }
        }
    }

    fn cmd_set<'a, C, I>(&mut self, app: &mut App<C>, mut parts: I)
    where
        C: ControlLoop,
        I: Iterator<Item = &'a str>,
    {
        match (parts.next(), parts.next()) {
            (Some("sp"), Some(val_str)) => {
                if let Ok(val) = val_str.parse::<i16>() {
                    let new_sp = open_servo_control::CentiDeg::from_cdeg(val);
                    app.set_setpoint(new_sp);
                    let mut buf: String<64> = String::new();
                    let _ = write!(buf, "ok, sp={}", new_sp.as_cdeg());
                    self.println(&buf);
                } else {
                    self.println("invalid value");
                }
            }
            (Some("sp"), None) => {
                self.println("usage: set sp <centidegrees>");
            }
            (Some(param), _) => {
                let mut buf: String<64> = String::new();
                let _ = write!(buf, "unknown param: {}", param);
                self.println(&buf);
            }
            (None, _) => {
                self.println("usage: set <param> <value>");
                self.println("params: sp");
            }
        }
    }

    // --- Output helpers ---

    fn println(&mut self, s: &str) {
        self.io.write(s.as_bytes());
        self.io.write(b"\r\n");
    }
}

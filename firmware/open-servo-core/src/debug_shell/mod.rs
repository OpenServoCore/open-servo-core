//! Debug shell / REPL for interactive debugging.
//!
//! This module provides a line-oriented command shell that runs in Tier 3
//! (main loop), generic over the underlying debug I/O transport.
//!
//! ## Architecture
//!
//! The shell is split into modules along pure vs hardware-aware boundaries:
//!
//! - `arg_parser` - Pure argument parsing helper (host-testable)
//! - `command` - Command enums and parsing (host-testable)
//! - `exec` - Command execution with hardware side effects

mod arg_parser;
mod command;
mod exec;

use core::fmt::Write;
use heapless::String;

use crate::App;
use open_servo_control::ControlLoop;
use open_servo_hw::{BdcMotorDriver, DebugIo};

use arg_parser::{ArgError, ArgParser};
use command::{Command, ParseError};

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

        let mut ap = ArgParser::new(line);
        match Command::parse(&mut ap) {
            Ok(cmd) => self.exec_command(app, hw, cmd),
            Err(ParseError::Empty) => {}
            Err(e) => self.print_parse_error(e),
        }
    }

    /// Print a parse error message.
    fn print_parse_error(&mut self, e: ParseError<'_>) {
        match e {
            ParseError::Empty => {}
            ParseError::UnknownCommand(c) => {
                let mut buf: String<64> = String::new();
                let _ = write!(buf, "unknown command: {}", c);
                self.println(&buf);
            }
            ParseError::UnknownSubcommand(s) => {
                if s.is_empty() {
                    self.println("missing subcommand");
                } else {
                    let mut buf: String<64> = String::new();
                    let _ = write!(buf, "unknown subcommand: {}", s);
                    self.println(&buf);
                }
            }
            ParseError::Arg(ArgError::Missing(name)) => {
                let mut buf: String<64> = String::new();
                let _ = write!(buf, "missing argument: {}", name);
                self.println(&buf);
            }
            ParseError::Arg(ArgError::Invalid(name)) => {
                let mut buf: String<64> = String::new();
                let _ = write!(buf, "invalid {}", name);
                self.println(&buf);
            }
            ParseError::Arg(ArgError::Extra) => {
                self.println("too many arguments");
            }
        }
    }

    // ========================================================================
    // Output helpers
    // ========================================================================

    pub(super) fn println(&mut self, s: &str) {
        self.io.write(s.as_bytes());
        self.io.write(b"\r\n");
    }
}

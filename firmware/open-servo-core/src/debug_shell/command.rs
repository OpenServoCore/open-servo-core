//! Command enums and parsing logic.
//!
//! Pure parsing with no hardware dependencies - can be unit tested on host.

use super::arg_parser::{ArgError, ArgParser};
#[cfg(feature = "pid")]
use open_servo_math::{DerivativeMode, Gain};

// ============================================================================
// Command Enums
// ============================================================================

/// Top-level parsed command.
#[derive(Debug)]
pub enum Command {
    Help,
    State,
    Fault(FaultCmd),
    Set(SetCmd),
    Limit(LimitCmd),
    #[cfg(feature = "pid")]
    Pid(PidCmd),
    Motor(MotorCmd),
    Compliance(ComplianceCmd),
}

/// Fault subcommands.
#[derive(Debug)]
pub enum FaultCmd {
    Show,
    Clear,
}

/// Motor control subcommands.
#[derive(Debug)]
pub enum MotorCmd {
    Status,
    Engage,
    Disengage,
}

/// Compliance subcommands.
#[derive(Debug)]
pub enum ComplianceCmd {
    Show,
    MoveMa(i16),
    HoldMa(i16),
    Vel(i16),
}

/// Set subcommands.
#[derive(Debug)]
pub enum SetCmd {
    Sp(i16),
}

/// Limit subcommands.
#[derive(Debug)]
pub enum LimitCmd {
    Show,
    #[cfg(feature = "current-sense-bus")]
    Current(Option<i16>),
    Temp(Option<i16>),
    Delta(Option<i16>),
    Faults(Option<u8>),
    Pos(i16, i16),
    Stall(Option<u16>),
    Error(Option<i16>),
    Reset,
}

/// PID subcommands for position loop.
///
/// Grammar:
/// - `pid pos show`              → Show
/// - `pid pos set kp <gain>`     → SetOne { Kp, value }
/// - `pid pos set ki <gain>`     → SetOne { Ki, value }
/// - `pid pos set kd <gain>`     → SetOne { Kd, value }
/// - `pid pos set <kp> <ki> <kd>` → SetAll { kp, ki, kd }
/// - `pid pos mode err|meas`     → Mode(DerivativeMode)
///
/// Gains are parsed as fixed-point (e.g. "40.0" → Gain) to avoid
/// linking the ~10KB core::num::dec2flt float parsing code.
#[cfg(feature = "pid")]
#[derive(Debug)]
pub enum PidCmd {
    /// Show all gains
    Show,
    /// Set a single gain field
    SetOne { field: PidField, value: Gain },
    /// Set all three gains atomically
    SetAll { kp: Gain, ki: Gain, kd: Gain },
    /// Set derivative mode
    Mode(DerivativeMode),
}

/// PID gain field selector.
#[cfg(feature = "pid")]
#[derive(Debug, Clone, Copy)]
pub enum PidField {
    Kp,
    Ki,
    Kd,
}

// ============================================================================
// Parse Errors
// ============================================================================

/// Command parsing errors.
#[derive(Debug)]
pub enum ParseError<'a> {
    /// Empty input (not really an error, just skip)
    Empty,
    /// Unknown top-level command
    UnknownCommand(&'a str),
    /// Unknown subcommand
    UnknownSubcommand(&'a str),
    /// Argument error
    Arg(ArgError),
}

impl<'a> From<ArgError> for ParseError<'a> {
    fn from(e: ArgError) -> Self {
        ParseError::Arg(e)
    }
}

// ============================================================================
// Parsing
// ============================================================================

impl Command {
    /// Parse a command from the argument parser.
    pub fn parse<'a>(ap: &mut ArgParser<'a>) -> Result<Self, ParseError<'a>> {
        let Some(cmd) = ap.next_str() else {
            return Err(ParseError::Empty);
        };

        match cmd {
            "help" | "?" => Ok(Command::Help),
            "state" | "s" => Ok(Command::State),

            "fault" => match ap.next_str() {
                Some("clear") => Ok(Command::Fault(FaultCmd::Clear)),
                None => Ok(Command::Fault(FaultCmd::Show)),
                Some(other) => Err(ParseError::UnknownSubcommand(other)),
            },

            "set" => match ap.next_str() {
                Some("sp") => {
                    let val = ap.req::<i16>("value")?;
                    ap.end()?;
                    Ok(Command::Set(SetCmd::Sp(val)))
                }
                Some(other) => Err(ParseError::UnknownSubcommand(other)),
                None => Err(ParseError::UnknownSubcommand("")),
            },

            "limit" => match ap.next_str() {
                None => Ok(Command::Limit(LimitCmd::Show)),
                #[cfg(feature = "current-sense-bus")]
                Some("current") => {
                    let val = ap.next::<i16>("mA")?;
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Current(val)))
                }
                Some("temp") => {
                    let val = ap.next::<i16>("dC")?;
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Temp(val)))
                }
                Some("delta") => {
                    let val = ap.next::<i16>("cdeg")?;
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Delta(val)))
                }
                Some("faults") => {
                    let val = ap.next::<u8>("count")?;
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Faults(val)))
                }
                Some("pos") => {
                    let min = ap.req::<i16>("min")?;
                    let max = ap.req::<i16>("max")?;
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Pos(min, max)))
                }
                Some("stall") => {
                    let val = ap.next::<u16>("ticks")?;
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Stall(val)))
                }
                Some("error") => {
                    let val = ap.next::<i16>("cdeg")?;
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Error(val)))
                }
                Some("reset") => {
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Reset))
                }
                Some(other) => Err(ParseError::UnknownSubcommand(other)),
            },

            #[cfg(feature = "pid")]
            "pid" => Self::parse_pid(ap),

            "motor" => match ap.next_str() {
                None | Some("status") => {
                    ap.end()?;
                    Ok(Command::Motor(MotorCmd::Status))
                }
                Some("engage") => {
                    ap.end()?;
                    Ok(Command::Motor(MotorCmd::Engage))
                }
                Some("disengage") => {
                    ap.end()?;
                    Ok(Command::Motor(MotorCmd::Disengage))
                }
                Some(other) => Err(ParseError::UnknownSubcommand(other)),
            },

            "compliance" => match ap.next_str() {
                None | Some("show") => {
                    ap.end()?;
                    Ok(Command::Compliance(ComplianceCmd::Show))
                }
                Some("move") => {
                    let ma = ap.req::<i16>("milliamps")?;
                    ap.end()?;
                    Ok(Command::Compliance(ComplianceCmd::MoveMa(ma)))
                }
                Some("hold") => {
                    let ma = ap.req::<i16>("milliamps")?;
                    ap.end()?;
                    Ok(Command::Compliance(ComplianceCmd::HoldMa(ma)))
                }
                Some("vel") => {
                    let dps = ap.req::<i16>("deg/s")?;
                    ap.end()?;
                    Ok(Command::Compliance(ComplianceCmd::Vel(dps)))
                }
                Some(other) => Err(ParseError::UnknownSubcommand(other)),
            },

            other => Err(ParseError::UnknownCommand(other)),
        }
    }

    /// Parse PID subcommands.
    /// Grammar: pid pos show | pid pos set ... | pid pos mode ...
    #[cfg(feature = "pid")]
    fn parse_pid<'a>(ap: &mut ArgParser<'a>) -> Result<Self, ParseError<'a>> {
        match ap.next_str() {
            // pid pos ...
            Some("pos") => Self::parse_pid_loop(ap),
            // Future: Some("vel") => ..., Some("cur") => ...
            None => Err(ParseError::UnknownSubcommand("")),
            Some(other) => Err(ParseError::UnknownSubcommand(other)),
        }
    }

    /// Parse PID loop subcommands (show, set, mode).
    #[cfg(feature = "pid")]
    fn parse_pid_loop<'a>(ap: &mut ArgParser<'a>) -> Result<Self, ParseError<'a>> {
        match ap.next_str() {
            Some("show") | None => {
                ap.end()?;
                Ok(Command::Pid(PidCmd::Show))
            }
            Some("set") => Self::parse_pid_set(ap),
            Some("mode") => Self::parse_pid_mode(ap),
            Some(other) => Err(ParseError::UnknownSubcommand(other)),
        }
    }

    /// Parse: pid pos set kp <gain> | pid pos set <kp> <ki> <kd>
    #[cfg(feature = "pid")]
    fn parse_pid_set<'a>(ap: &mut ArgParser<'a>) -> Result<Self, ParseError<'a>> {
        match ap.next_str() {
            Some("kp") => {
                let val = ap.req::<Gain>("value")?;
                ap.end()?;
                Ok(Command::Pid(PidCmd::SetOne {
                    field: PidField::Kp,
                    value: val,
                }))
            }
            Some("ki") => {
                let val = ap.req::<Gain>("value")?;
                ap.end()?;
                Ok(Command::Pid(PidCmd::SetOne {
                    field: PidField::Ki,
                    value: val,
                }))
            }
            Some("kd") => {
                let val = ap.req::<Gain>("value")?;
                ap.end()?;
                Ok(Command::Pid(PidCmd::SetOne {
                    field: PidField::Kd,
                    value: val,
                }))
            }
            // Try parsing as three gains: pid pos set <kp> <ki> <kd>
            Some(first) => {
                let kp: Gain = first.parse().map_err(|_| ArgError::Invalid("kp"))?;
                let ki = ap.req::<Gain>("ki")?;
                let kd = ap.req::<Gain>("kd")?;
                ap.end()?;
                Ok(Command::Pid(PidCmd::SetAll { kp, ki, kd }))
            }
            None => Err(ParseError::Arg(ArgError::Missing("field or kp"))),
        }
    }

    /// Parse: pid pos mode err|meas
    #[cfg(feature = "pid")]
    fn parse_pid_mode<'a>(ap: &mut ArgParser<'a>) -> Result<Self, ParseError<'a>> {
        match ap.next_str() {
            Some("err") | Some("error") => {
                ap.end()?;
                Ok(Command::Pid(PidCmd::Mode(DerivativeMode::OnError)))
            }
            Some("meas") | Some("measurement") => {
                ap.end()?;
                Ok(Command::Pid(PidCmd::Mode(DerivativeMode::OnMeasurement)))
            }
            Some(other) => Err(ParseError::UnknownSubcommand(other)),
            None => Err(ParseError::Arg(ArgError::Missing("mode"))),
        }
    }
}

//! Command enums and parsing logic.
//!
//! Pure parsing with no hardware dependencies - can be unit tested on host.

use super::arg_parser::{ArgError, ArgParser};
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
    Pid(PidCmd),
}

/// Fault subcommands.
#[derive(Debug)]
pub enum FaultCmd {
    Show,
    Clear,
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
    Current(Option<i16>),
    Temp(Option<i16>),
    Delta(Option<i16>),
    Faults(Option<u8>),
    Pos(i16, i16),
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
                Some("reset") => {
                    ap.end()?;
                    Ok(Command::Limit(LimitCmd::Reset))
                }
                Some(other) => Err(ParseError::UnknownSubcommand(other)),
            },

            "pid" => Self::parse_pid(ap),

            other => Err(ParseError::UnknownCommand(other)),
        }
    }

    /// Parse PID subcommands.
    /// Grammar: pid pos show | pid pos set ... | pid pos mode ...
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

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(input: &str) -> Result<Command, ParseError<'_>> {
        let mut ap = ArgParser::new(input);
        Command::parse(&mut ap)
    }

    #[test]
    fn test_help() {
        assert!(matches!(parse("help"), Ok(Command::Help)));
        assert!(matches!(parse("?"), Ok(Command::Help)));
    }

    #[test]
    fn test_state() {
        assert!(matches!(parse("state"), Ok(Command::State)));
        assert!(matches!(parse("s"), Ok(Command::State)));
    }

    #[test]
    fn test_fault() {
        assert!(matches!(parse("fault"), Ok(Command::Fault(FaultCmd::Show))));
        assert!(matches!(
            parse("fault clear"),
            Ok(Command::Fault(FaultCmd::Clear))
        ));
    }

    #[test]
    fn test_set_sp() {
        match parse("set sp 9000") {
            Ok(Command::Set(SetCmd::Sp(val))) => assert_eq!(val, 9000),
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_limit_show() {
        assert!(matches!(parse("limit"), Ok(Command::Limit(LimitCmd::Show))));
    }

    #[test]
    fn test_limit_current() {
        // Get current value
        assert!(matches!(
            parse("limit current"),
            Ok(Command::Limit(LimitCmd::Current(None)))
        ));

        // Set current value
        match parse("limit current 500") {
            Ok(Command::Limit(LimitCmd::Current(Some(val)))) => assert_eq!(val, 500),
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_limit_pos() {
        match parse("limit pos 100 17000") {
            Ok(Command::Limit(LimitCmd::Pos(min, max))) => {
                assert_eq!(min, 100);
                assert_eq!(max, 17000);
            }
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_unknown_command() {
        assert!(matches!(
            parse("foobar"),
            Err(ParseError::UnknownCommand("foobar"))
        ));
    }

    #[test]
    fn test_unknown_subcommand() {
        assert!(matches!(
            parse("fault foo"),
            Err(ParseError::UnknownSubcommand("foo"))
        ));
    }

    #[test]
    fn test_empty() {
        assert!(matches!(parse(""), Err(ParseError::Empty)));
    }

    // ========================================================================
    // PID command tests
    // ========================================================================

    #[test]
    fn test_pid_pos_show() {
        assert!(matches!(parse("pid pos"), Ok(Command::Pid(PidCmd::Show))));
        assert!(matches!(
            parse("pid pos show"),
            Ok(Command::Pid(PidCmd::Show))
        ));
    }

    #[test]
    fn test_pid_pos_set_kp() {
        match parse("pid pos set kp 40.0") {
            Ok(Command::Pid(PidCmd::SetOne { field, value })) => {
                assert!(matches!(field, PidField::Kp));
                assert!((value.as_f32() - 40.0).abs() < 0.01);
            }
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_pid_pos_set_ki() {
        match parse("pid pos set ki 0.1") {
            Ok(Command::Pid(PidCmd::SetOne { field, value })) => {
                assert!(matches!(field, PidField::Ki));
                assert!((value.as_f32() - 0.1).abs() < 0.01);
            }
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_pid_pos_set_kd() {
        match parse("pid pos set kd 10.0") {
            Ok(Command::Pid(PidCmd::SetOne { field, value })) => {
                assert!(matches!(field, PidField::Kd));
                assert!((value.as_f32() - 10.0).abs() < 0.01);
            }
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_pid_pos_set_all() {
        match parse("pid pos set 40.0 0.1 10.0") {
            Ok(Command::Pid(PidCmd::SetAll { kp, ki, kd })) => {
                assert!((kp.as_f32() - 40.0).abs() < 0.01);
                assert!((ki.as_f32() - 0.1).abs() < 0.01);
                assert!((kd.as_f32() - 10.0).abs() < 0.01);
            }
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_pid_pos_mode_err() {
        match parse("pid pos mode err") {
            Ok(Command::Pid(PidCmd::Mode(mode))) => {
                assert!(matches!(mode, DerivativeMode::OnError));
            }
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn test_pid_pos_mode_meas() {
        match parse("pid pos mode meas") {
            Ok(Command::Pid(PidCmd::Mode(mode))) => {
                assert!(matches!(mode, DerivativeMode::OnMeasurement));
            }
            other => panic!("unexpected: {:?}", other),
        }
    }
}

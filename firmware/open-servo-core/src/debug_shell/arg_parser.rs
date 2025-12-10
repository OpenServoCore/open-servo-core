//! Argument parser for debug shell commands.
//!
//! Pure parsing helper with no hardware dependencies - can be unit tested on host.

use core::str::FromStr;

/// Argument parsing errors
#[derive(Debug, Clone, Copy)]
pub enum ArgError {
    /// Required argument was not provided
    Missing(&'static str),
    /// Argument could not be parsed as expected type
    Invalid(&'static str),
    /// Extra arguments provided
    Extra,
}

/// Helper for parsing whitespace-separated command arguments with typed extraction.
///
/// # Example
/// ```ignore
/// let mut ap = ArgParser::new("set sp 9000");
/// assert_eq!(ap.next_str(), Some("set"));
/// assert_eq!(ap.next_str(), Some("sp"));
/// assert_eq!(ap.req::<i16>("value"), Ok(9000));
/// assert!(ap.end().is_ok());
/// ```
pub struct ArgParser<'a> {
    parts: core::str::SplitWhitespace<'a>,
}

impl<'a> ArgParser<'a> {
    /// Create a new parser for the given input string.
    pub fn new(input: &'a str) -> Self {
        Self {
            parts: input.split_whitespace(),
        }
    }

    /// Get next argument as a string slice.
    pub fn next_str(&mut self) -> Option<&'a str> {
        self.parts.next()
    }

    /// Get next argument as optional typed value.
    ///
    /// Returns:
    /// - `Ok(None)` if no more arguments
    /// - `Ok(Some(v))` if parsed successfully
    /// - `Err(Invalid)` if argument exists but fails to parse
    pub fn next<T: FromStr>(&mut self, name: &'static str) -> Result<Option<T>, ArgError> {
        match self.parts.next() {
            None => Ok(None),
            Some(s) => s
                .parse::<T>()
                .map(Some)
                .map_err(|_| ArgError::Invalid(name)),
        }
    }

    /// Get next argument as required typed value.
    ///
    /// Returns:
    /// - `Ok(v)` if parsed successfully
    /// - `Err(Missing)` if no more arguments
    /// - `Err(Invalid)` if argument exists but fails to parse
    pub fn req<T: FromStr>(&mut self, name: &'static str) -> Result<T, ArgError> {
        self.next::<T>(name)?.ok_or(ArgError::Missing(name))
    }

    /// Assert that there are no more arguments.
    ///
    /// Returns `Err(Extra)` if any arguments remain.
    pub fn end(&mut self) -> Result<(), ArgError> {
        if self.parts.clone().next().is_some() {
            Err(ArgError::Extra)
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_parsing() {
        let mut ap = ArgParser::new("help");
        assert_eq!(ap.next_str(), Some("help"));
        assert_eq!(ap.next_str(), None);
    }

    #[test]
    fn test_typed_parsing() {
        let mut ap = ArgParser::new("set sp 9000");
        assert_eq!(ap.next_str(), Some("set"));
        assert_eq!(ap.next_str(), Some("sp"));
        assert_eq!(ap.req::<i16>("value").unwrap(), 9000);
        assert!(ap.end().is_ok());
    }

    #[test]
    fn test_optional_arg() {
        let mut ap = ArgParser::new("limit current");
        assert_eq!(ap.next_str(), Some("limit"));
        assert_eq!(ap.next_str(), Some("current"));
        assert_eq!(ap.next::<i16>("mA").unwrap(), None);
    }

    #[test]
    fn test_optional_arg_present() {
        let mut ap = ArgParser::new("limit current 500");
        assert_eq!(ap.next_str(), Some("limit"));
        assert_eq!(ap.next_str(), Some("current"));
        assert_eq!(ap.next::<i16>("mA").unwrap(), Some(500));
    }

    #[test]
    fn test_missing_required() {
        let mut ap = ArgParser::new("set sp");
        ap.next_str(); // "set"
        ap.next_str(); // "sp"
        assert!(matches!(
            ap.req::<i16>("value"),
            Err(ArgError::Missing("value"))
        ));
    }

    #[test]
    fn test_invalid_value() {
        let mut ap = ArgParser::new("set sp notanumber");
        ap.next_str(); // "set"
        ap.next_str(); // "sp"
        assert!(matches!(
            ap.req::<i16>("value"),
            Err(ArgError::Invalid("value"))
        ));
    }

    #[test]
    fn test_extra_args() {
        let mut ap = ArgParser::new("help extra stuff");
        ap.next_str(); // "help"
        assert!(matches!(ap.end(), Err(ArgError::Extra)));
    }
}

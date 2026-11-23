//! Where a verb keeps its files: a folder in the CLI, memory in the
//! browser. The library names a file by its path in the run and never
//! opens one itself.

use std::collections::BTreeMap;
use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SinkError(pub String);

impl fmt::Display for SinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for SinkError {}

pub trait Sink {
    /// Write `bytes` whole at `path`, a `/`-separated path in the run.
    fn write(&mut self, path: &str, bytes: &[u8]) -> Result<(), SinkError>;
    /// The bytes at `path` as they are kept now.
    fn read(&mut self, path: &str) -> Result<Vec<u8>, SinkError>;
    /// Where `path` is kept, in the words a user finds it by.
    fn shown(&self, path: &str) -> String {
        path.to_string()
    }
}

/// Files in memory: the browser's run, and the tests'.
#[derive(Clone, Debug, Default)]
pub struct MemSink {
    pub files: BTreeMap<String, Vec<u8>>,
}

impl Sink for MemSink {
    fn write(&mut self, path: &str, bytes: &[u8]) -> Result<(), SinkError> {
        self.files.insert(path.to_string(), bytes.to_vec());
        Ok(())
    }

    fn read(&mut self, path: &str) -> Result<Vec<u8>, SinkError> {
        self.files
            .get(path)
            .cloned()
            .ok_or_else(|| SinkError(format!("{path}: no such file")))
    }
}

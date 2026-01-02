//! Defmt log decoder.
//!
//! Decodes defmt frames from RTT channel 0 into log entries.

use crate::app::{LogEntry, LogLevel};
use anyhow::{Context, Result};
use defmt_decoder::{DecodeError, Frame, Locations, Table};
use std::path::Path;

/// Defmt decoder state.
///
/// Note: We store the raw ELF data and recreate the decoder as needed
/// because the StreamDecoder borrows from Table which complicates lifetimes.
pub struct DefmtDecoder {
    elf_data: Vec<u8>,
    locations: Option<Locations>,
    buffer: Vec<u8>,
}

impl DefmtDecoder {
    /// Create a new decoder from ELF file.
    pub fn from_elf(elf_path: &Path) -> Result<Self> {
        let elf_data = std::fs::read(elf_path).context("Failed to read ELF file")?;

        // Verify we can parse the table
        let table = Table::parse(&elf_data)
            .map_err(|e| anyhow::anyhow!("Failed to parse defmt table: {:?}", e))?
            .context("No defmt data in ELF")?;

        let locations = table.get_locations(&elf_data).ok();

        Ok(Self {
            elf_data,
            locations,
            buffer: Vec::new(),
        })
    }

    /// Decode raw RTT data into log entries.
    pub fn decode(&mut self, data: &[u8]) -> Vec<LogEntry> {
        // Accumulate data
        self.buffer.extend_from_slice(data);

        // Parse table and create decoder
        let table = match Table::parse(&self.elf_data) {
            Ok(Some(t)) => t,
            _ => return Vec::new(),
        };

        let mut decoder = table.new_stream_decoder();
        decoder.received(&self.buffer);

        let mut entries = Vec::new();

        loop {
            match decoder.decode() {
                Ok(frame) => {
                    if let Some(entry) = self.frame_to_entry(frame) {
                        entries.push(entry);
                    }
                }
                Err(DecodeError::UnexpectedEof) => break,
                Err(DecodeError::Malformed) => {
                    tracing::warn!("Malformed defmt frame");
                    self.buffer.clear(); // Reset on malformed
                    break;
                }
            }
        }

        // Clear processed data (simplified - in production track consumed bytes)
        if !entries.is_empty() {
            self.buffer.clear();
        }

        entries
    }

    fn frame_to_entry(&self, frame: Frame) -> Option<LogEntry> {
        let display = frame.display_message().to_string();

        // Parse level from frame - use Debug format since Level is private
        let level = if let Some(level) = frame.level() {
            let level_str = format!("{:?}", level);
            match level_str.as_str() {
                "Trace" => LogLevel::Trace,
                "Debug" => LogLevel::Debug,
                "Info" => LogLevel::Info,
                "Warn" => LogLevel::Warn,
                "Error" => LogLevel::Error,
                _ => LogLevel::Info,
            }
        } else {
            LogLevel::Info
        };

        // Get location info if available
        let module = if let Some(ref locs) = self.locations {
            if let Some(loc) = locs.get(&frame.index()) {
                format!("{}:{}", loc.file.display(), loc.line)
            } else {
                String::new()
            }
        } else {
            String::new()
        };

        // Format timestamp
        let timestamp = frame
            .display_timestamp()
            .map(|ts| format!("{}", ts))
            .unwrap_or_else(|| "          ".to_string());

        Some(LogEntry {
            timestamp,
            level,
            module,
            message: display,
        })
    }
}

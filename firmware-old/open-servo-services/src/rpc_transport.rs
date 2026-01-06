//! RTT transport for postcard-rpc.
//!
//! Implements async WireTx/WireRx traits for RTT channels using COBS framing.

use embedded_io_async::{Read, Write};
use heapless::Vec;

/// Maximum RPC message size.
pub const MAX_MSG_SIZE: usize = 128;

/// COBS-encoded buffer size (worst case: every byte is 0x00).
pub const COBS_BUF_SIZE: usize = MAX_MSG_SIZE + (MAX_MSG_SIZE / 254) + 2;

/// Capacity of the RPC TX queue.
pub const TX_QUEUE_CAPACITY: usize = 4;

/// RTT-based transport for RPC using a single IO type.
///
/// This transport uses COBS framing (0x00 delimiter) for message boundaries.
pub struct RttTransport<'a, IO: Read + Write> {
    io: IO,
    rx_buf: &'a mut [u8],
    rx_pos: usize,
    tx_buf: Vec<u8, COBS_BUF_SIZE>,
}

impl<'a, IO: Read + Write> RttTransport<'a, IO> {
    /// Create a new RTT transport.
    pub fn new(io: IO, rx_buf: &'a mut [u8]) -> Self {
        Self {
            io,
            rx_buf,
            rx_pos: 0,
            tx_buf: Vec::new(),
        }
    }

    /// Send a COBS-framed message.
    pub async fn send(&mut self, data: &[u8]) -> Result<(), IO::Error> {
        self.tx_buf.clear();

        // COBS encode
        let mut code_idx = 0;
        let _ = self.tx_buf.push(0); // Placeholder for first code byte
        let mut code: u8 = 1;

        for &byte in data {
            if byte == 0 {
                if code_idx < self.tx_buf.len() {
                    self.tx_buf[code_idx] = code;
                }
                code_idx = self.tx_buf.len();
                let _ = self.tx_buf.push(0);
                code = 1;
            } else {
                let _ = self.tx_buf.push(byte);
                code += 1;
                if code == 0xFF {
                    if code_idx < self.tx_buf.len() {
                        self.tx_buf[code_idx] = code;
                    }
                    code_idx = self.tx_buf.len();
                    let _ = self.tx_buf.push(0);
                    code = 1;
                }
            }
        }

        if code_idx < self.tx_buf.len() {
            self.tx_buf[code_idx] = code;
        }
        let _ = self.tx_buf.push(0); // Frame delimiter

        // Write to RTT
        self.io.write_all(&self.tx_buf).await?;

        Ok(())
    }

    /// Try to receive a COBS-framed message.
    ///
    /// Returns the decoded message length, or 0 if no complete message available.
    pub async fn recv(&mut self, out: &mut [u8]) -> Result<usize, IO::Error> {
        // Read available data
        loop {
            // Check if we have a complete frame
            if let Some(delim_pos) = self.rx_buf[..self.rx_pos].iter().position(|&b| b == 0) {
                if delim_pos > 0 {
                    // Decode COBS frame
                    let frame = &self.rx_buf[..delim_pos];
                    let decoded_len = cobs_decode(frame, out);

                    // Remove processed data
                    self.rx_buf.copy_within(delim_pos + 1..self.rx_pos, 0);
                    self.rx_pos -= delim_pos + 1;

                    if decoded_len > 0 {
                        return Ok(decoded_len);
                    }
                } else {
                    // Empty frame, skip delimiter
                    self.rx_buf.copy_within(1..self.rx_pos, 0);
                    self.rx_pos -= 1;
                }
                continue;
            }

            // Need more data
            if self.rx_pos >= self.rx_buf.len() {
                // Buffer full without delimiter - reset
                self.rx_pos = 0;
            }

            let n = self.io.read(&mut self.rx_buf[self.rx_pos..]).await?;
            if n == 0 {
                return Ok(0);
            }
            self.rx_pos += n;
        }
    }
}

/// Decode a COBS frame in-place.
///
/// Returns the number of decoded bytes written to `out`.
fn cobs_decode(data: &[u8], out: &mut [u8]) -> usize {
    let mut out_idx = 0;
    let mut i = 0;

    while i < data.len() {
        let code = data[i] as usize;
        if code == 0 {
            return 0; // Invalid
        }

        i += 1;

        for _ in 1..code {
            if i >= data.len() || out_idx >= out.len() {
                return 0; // Truncated
            }
            out[out_idx] = data[i];
            out_idx += 1;
            i += 1;
        }

        if code < 0xFF && i < data.len() {
            if out_idx >= out.len() {
                return 0; // Output buffer too small
            }
            out[out_idx] = 0;
            out_idx += 1;
        }
    }

    out_idx
}

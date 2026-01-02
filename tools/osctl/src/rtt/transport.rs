//! RTT transport for postcard-rpc.
//!
//! Implements COBS framing for RPC messages over RTT channels.

use anyhow::Result;

/// COBS encoder for outgoing messages.
pub struct CobsEncoder {
    buffer: Vec<u8>,
}

impl CobsEncoder {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(256),
        }
    }

    /// Encode data with COBS framing (0x00 delimiter).
    pub fn encode(&mut self, data: &[u8]) -> &[u8] {
        self.buffer.clear();

        // COBS encode
        let mut code_idx = 0;
        self.buffer.push(0); // Placeholder for first code byte
        let mut code: u8 = 1;

        for &byte in data {
            if byte == 0 {
                self.buffer[code_idx] = code;
                code_idx = self.buffer.len();
                self.buffer.push(0); // Placeholder for next code
                code = 1;
            } else {
                self.buffer.push(byte);
                code += 1;
                if code == 0xFF {
                    self.buffer[code_idx] = code;
                    code_idx = self.buffer.len();
                    self.buffer.push(0);
                    code = 1;
                }
            }
        }

        self.buffer[code_idx] = code;
        self.buffer.push(0); // Frame delimiter

        &self.buffer
    }
}

/// COBS decoder for incoming messages.
pub struct CobsDecoder {
    buffer: Vec<u8>,
    decoded: Vec<u8>,
}

impl CobsDecoder {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(1024),
            decoded: Vec::with_capacity(1024),
        }
    }

    /// Feed raw data and return any complete decoded frames.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Vec<u8>> {
        self.buffer.extend_from_slice(data);

        let mut frames = Vec::new();

        // Process complete frames (delimited by 0x00)
        while let Some(delim_pos) = self.buffer.iter().position(|&b| b == 0) {
            if delim_pos > 0 {
                // Copy frame data to avoid borrow conflict
                let frame_data: Vec<u8> = self.buffer[..delim_pos].to_vec();
                if let Some(decoded) = self.decode_cobs(&frame_data) {
                    frames.push(decoded);
                }
            }
            self.buffer.drain(..=delim_pos);
        }

        frames
    }

    /// Decode a single COBS frame.
    fn decode_cobs(&mut self, data: &[u8]) -> Option<Vec<u8>> {
        self.decoded.clear();

        let mut i = 0;
        while i < data.len() {
            let code = data[i] as usize;
            if code == 0 {
                return None; // Invalid
            }

            i += 1;

            for _ in 1..code {
                if i >= data.len() {
                    return None; // Truncated
                }
                self.decoded.push(data[i]);
                i += 1;
            }

            if code < 0xFF && i < data.len() {
                self.decoded.push(0);
            }
        }

        Some(self.decoded.clone())
    }
}

/// RPC client for communication over RTT.
pub struct RpcClient {
    encoder: CobsEncoder,
    decoder: CobsDecoder,
    seq: u32,
}

impl RpcClient {
    pub fn new() -> Self {
        Self {
            encoder: CobsEncoder::new(),
            decoder: CobsDecoder::new(),
            seq: 0,
        }
    }

    /// Encode an RPC request for transmission. Returns (encoded_data, seq_number).
    pub fn encode_request<T: serde::Serialize>(&mut self, path: &str, payload: &T) -> Result<(Vec<u8>, u32)> {
        // postcard-rpc wire format:
        // [key: 8 bytes][seq: 4 bytes][payload: postcard]

        let mut data = Vec::new();

        // Simple key hash (FNV-1a of path)
        let key = fnv1a_hash(path.as_bytes());
        data.extend_from_slice(&key.to_le_bytes());

        // Sequence number
        let seq = self.seq;
        data.extend_from_slice(&seq.to_le_bytes());
        self.seq = self.seq.wrapping_add(1);

        // Payload
        let payload_bytes = postcard::to_allocvec(payload)?;
        data.extend_from_slice(&payload_bytes);

        // COBS encode
        let encoded = self.encoder.encode(&data);
        Ok((encoded.to_vec(), seq))
    }

    /// Feed received data and decode any complete responses.
    pub fn decode_responses(&mut self, data: &[u8]) -> Vec<RpcResponse> {
        let frames = self.decoder.feed(data);

        frames
            .into_iter()
            .filter_map(|frame| {
                if frame.len() < 8 {
                    return None;
                }

                let key = u64::from_le_bytes(frame[0..8].try_into().ok()?);
                let seq = u32::from_le_bytes(frame[8..12].try_into().ok()?);
                let payload = frame[12..].to_vec();

                Some(RpcResponse { key, seq, payload })
            })
            .collect()
    }
}

/// Decoded RPC response.
pub struct RpcResponse {
    pub key: u64,
    pub seq: u32,
    pub payload: Vec<u8>,
}

/// FNV-1a hash for endpoint keys.
fn fnv1a_hash(data: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf29ce484222325;
    for &byte in data {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cobs_roundtrip() {
        let mut encoder = CobsEncoder::new();
        let mut decoder = CobsDecoder::new();

        let original = vec![0x01, 0x02, 0x00, 0x03, 0x04];
        let encoded = encoder.encode(&original);
        let frames = decoder.feed(encoded);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], original);
    }

    #[test]
    fn test_cobs_no_zeros() {
        let mut encoder = CobsEncoder::new();
        let mut decoder = CobsDecoder::new();

        let original = vec![0x01, 0x02, 0x03, 0x04];
        let encoded = encoder.encode(&original);
        let frames = decoder.feed(encoded);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], original);
    }
}

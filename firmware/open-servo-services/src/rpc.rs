//! RPC service task.
//!
//! Embassy async task that handles RPC requests over RTT.

use crate::rpc_transport::{RttTransport, MAX_MSG_SIZE};
use embassy_time::{Duration, Instant};
use embedded_io_async::{Read, Write};
use heapless::Vec;
use open_servo_runtime::reg_ops::{RegOps, StreamPlan};
use open_servo_runtime::shadow_storage::ShadowStorage;
use open_servo_rpc::{
    DeviceInfo, ReadRegReq, RegStreamFrame, RegStreamStartReq, RpcError, RpcResp, WriteRegReq,
    KEY_REG_DATA, KEY_REG_READ, KEY_REG_STREAM_START, KEY_REG_STREAM_STOP, KEY_REG_WRITE,
    KEY_SYS_INFO, KEY_SYS_PING,
};

/// Maximum packed size for streaming data.
const MAX_STREAM_DATA_SIZE: usize = 64;

/// RPC service state.
pub struct RpcService<'a, IO: Read + Write, const N: usize> {
    transport: RttTransport<'a, IO>,
    shadow: &'static ShadowStorage<N>,
    /// Streaming configuration
    stream_config: Option<StreamConfig>,
    /// Streaming sequence number
    stream_seq: u16,
}

/// Active streaming configuration.
struct StreamConfig {
    /// Stream plan with fields to read.
    plan: StreamPlan<16>,
    /// Interval between frames (source of truth for timing).
    interval: Duration,
    /// Next send time.
    next_send: Instant,
}

impl<'a, IO: Read + Write, const N: usize> RpcService<'a, IO, N> {
    /// Create a new RPC service.
    pub fn new(io: IO, transport_buf: &'a mut [u8], shadow: &'static ShadowStorage<N>) -> Self {
        Self {
            transport: RttTransport::new(io, transport_buf),
            shadow,
            stream_config: None,
            stream_seq: 0,
        }
    }

    /// Run the RPC service loop.
    pub async fn run(&mut self) -> ! {
        let mut rx_buf = [0u8; MAX_MSG_SIZE];

        loop {
            // If streaming, wait for either RPC data or stream timer
            if let Some(ref config) = self.stream_config {
                let now = Instant::now();
                if now >= config.next_send {
                    // Time to send a stream frame
                    self.send_stream_frame().await;
                    // Update next send time
                    if let Some(ref mut config) = self.stream_config {
                        config.next_send = now + config.interval;
                    }
                    continue;
                }

                // Wait for either RPC data or stream timer with short poll
                let timeout = config.next_send.saturating_duration_since(now);
                match embassy_time::with_timeout(timeout, self.transport.recv(&mut rx_buf)).await {
                    Ok(Ok(len)) if len > 0 => {
                        self.handle_request(&rx_buf[..len]).await;
                    }
                    Ok(Ok(_)) | Ok(Err(_)) | Err(_) => {
                        // Timeout or no data - loop will check stream timer
                    }
                }
            } else {
                // Not streaming - just wait for RPC data
                match self.transport.recv(&mut rx_buf).await {
                    Ok(len) if len > 0 => {
                        self.handle_request(&rx_buf[..len]).await;
                    }
                    Ok(_) | Err(_) => {}
                }
            }
        }
    }

    /// Send a stream frame with current register values.
    async fn send_stream_frame(&mut self) {
        let config = match &self.stream_config {
            Some(c) => c,
            None => return,
        };

        let ops = RegOps::new(self.shadow);

        // Use StreamPlan::snapshot to read all fields
        // Buffer sized to MAX_STREAM_DATA_SIZE to match capacity limit
        let mut data_buf = [0u8; MAX_STREAM_DATA_SIZE];
        let data_len = match config.plan.snapshot(&ops, &mut data_buf) {
            Ok(len) => len,
            Err(_) => return, // Drop frame on any read error (existing behavior)
        };

        let mut data: Vec<u8, 64> = Vec::new(); // matches RegStreamFrame::data capacity
        let _ = data.extend_from_slice(&data_buf[..data_len]);

        let frame = RegStreamFrame {
            seq: self.stream_seq,
            data,
        };
        self.stream_seq = self.stream_seq.wrapping_add(1);

        // Send frame with same header format as RPC responses
        let mut buf = [0u8; MAX_MSG_SIZE];
        buf[0..8].copy_from_slice(&KEY_REG_DATA.to_le_bytes());
        buf[8..12].copy_from_slice(&(frame.seq as u32).to_le_bytes());

        if let Ok(encoded) = postcard::to_slice(&frame, &mut buf[12..]) {
            let total_len = 12 + encoded.len();
            let _ = self.transport.send(&buf[..total_len]).await;
        }
    }

    async fn handle_request(&mut self, data: &[u8]) {
        // Wire format: [key: 8 bytes][seq: 4 bytes][payload...]
        if data.len() < 12 {
            return;
        }

        let key = u64::from_le_bytes(data[0..8].try_into().unwrap_or_default());
        let seq = u32::from_le_bytes(data[8..12].try_into().unwrap_or_default());
        let payload = &data[12..];

        // Encode directly to response buffer (no intermediate Vec)
        let mut resp_buf = [0u8; MAX_MSG_SIZE];
        resp_buf[0..8].copy_from_slice(&key.to_le_bytes());
        resp_buf[8..12].copy_from_slice(&seq.to_le_bytes());

        let payload_len = match key {
            KEY_SYS_PING => {
                let resp: RpcResp<()> = RpcResp::ok(());
                postcard::to_slice(&resp, &mut resp_buf[12..])
                    .map(|s| s.len())
                    .unwrap_or(0)
            }
            KEY_SYS_INFO => {
                let info = DeviceInfo {
                    version: (0, 1, 0),
                    device_id: 1,
                    model_number: 1000,
                    motor_type: 0,
                };
                let resp: RpcResp<DeviceInfo> = RpcResp::ok(info);
                postcard::to_slice(&resp, &mut resp_buf[12..])
                    .map(|s| s.len())
                    .unwrap_or(0)
            }
            KEY_REG_READ => {
                if let Ok(req) = postcard::from_bytes::<ReadRegReq>(payload) {
                    let resp = self.handle_read_reg(req);
                    postcard::to_slice(&resp, &mut resp_buf[12..])
                        .map(|s| s.len())
                        .unwrap_or(0)
                } else {
                    0
                }
            }
            KEY_REG_WRITE => {
                if let Ok(req) = postcard::from_bytes::<WriteRegReq>(payload) {
                    let resp = self.handle_write_reg(req);
                    postcard::to_slice(&resp, &mut resp_buf[12..])
                        .map(|s| s.len())
                        .unwrap_or(0)
                } else {
                    0
                }
            }
            KEY_REG_STREAM_START => {
                if let Ok(req) = postcard::from_bytes::<RegStreamStartReq>(payload) {
                    let resp = self.handle_stream_start(req);
                    postcard::to_slice(&resp, &mut resp_buf[12..])
                        .map(|s| s.len())
                        .unwrap_or(0)
                } else {
                    0
                }
            }
            KEY_REG_STREAM_STOP => {
                let resp = self.handle_stream_stop();
                postcard::to_slice(&resp, &mut resp_buf[12..])
                    .map(|s| s.len())
                    .unwrap_or(0)
            }
            _ => 0,
        };

        if payload_len > 0 {
            let _ = self.transport.send(&resp_buf[..12 + payload_len]).await;
        }
    }

    fn handle_read_reg(&self, req: ReadRegReq) -> RpcResp<Vec<u8, 4>> {
        let ops = RegOps::new(self.shadow);
        let mut buf = [0u8; 4];
        let len = (req.len as usize).min(4);

        if ops.read_range(req.addr, &mut buf[..len]).is_ok() {
            let mut data = Vec::new();
            let _ = data.extend_from_slice(&buf[..len]);
            RpcResp::ok(data)
        } else {
            RpcResp::err(RpcError::InvalidAddress)
        }
    }

    fn handle_write_reg(&mut self, req: WriteRegReq) -> RpcResp<()> {
        let ops = RegOps::new(self.shadow);
        if ops.write_range(req.addr, &req.data).is_ok() {
            RpcResp::ok(())
        } else {
            RpcResp::err(RpcError::InvalidAddress)
        }
    }

    fn handle_stream_start(&mut self, req: RegStreamStartReq) -> RpcResp<()> {
        // Estimate size: 4 bytes per address (preserve existing behavior)
        let estimated_size = req.addresses.len() * 4;
        if estimated_size > MAX_STREAM_DATA_SIZE {
            return RpcResp::err(RpcError::TooManyBytes);
        }

        if req.rate_hz == 0 {
            return RpcResp::err(RpcError::InvalidLength);
        }

        // Build StreamPlan from addresses (4 bytes each)
        let mut plan = StreamPlan::<16>::new();
        for &addr in &req.addresses {
            let _ = plan.fields.push((addr, 4));
        }

        // Interval calculation with floor at 1ms for rate_hz > 1000
        let interval_ms = (1000u64 / req.rate_hz as u64).max(1);
        self.stream_config = Some(StreamConfig {
            plan,
            interval: Duration::from_millis(interval_ms),
            next_send: Instant::now(),
        });
        self.stream_seq = 0;

        RpcResp::ok(())
    }

    fn handle_stream_stop(&mut self) -> RpcResp<()> {
        self.stream_config = None;
        RpcResp::ok(())
    }
}

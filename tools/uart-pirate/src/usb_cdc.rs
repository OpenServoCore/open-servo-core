//! USB-CDC ACM transport. Originally modelled on ch32-hal's
//! `examples/ch32v203/src/bin/usbd.rs`; the driver type now comes from the
//! vendored `crate::usbd` module.

use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use heapless::Vec;

use crate::proto::{self, Reply};
use crate::rx::{self, ByteRecord, FALL_LEN};
use crate::tx::TX_BUF_LEN;
use crate::usbd::Driver;

// `SEND bytes=<TX_BUF_LEN*2 hex> at=<u32>` ≤ TX_BUF_LEN*2 + 35; +slop.
const LINE_BUF_LEN: usize = TX_BUF_LEN * 2 + 64;

/// CDC bulk EP max-packet size. CdcAcmClass::new sets this on both
/// endpoints; `class.write_packet` requires the buffer to be ≤ this size
/// (an oversized write hangs the EP without erroring).
const CDC_BULK_PACKET: u16 = 64;

/// BBATCH per-call cap. Each record on the wire is 6 bytes (u32 tick LE,
/// u8 byte, u8 flags); a 64-record batch is 384 B plus 16 B ASCII header
/// and newline — ~6 CDC bulk packets per batch. Sized to keep the per-call
/// stack buffer small while amortizing CDC overhead.
const BBATCH_MAX: usize = 64;
const BSTAMP_SIZE: usize = 6;

#[embassy_executor::task]
pub async fn run() {
    let driver = Driver::new();

    let mut config = embassy_usb::Config::new(0xC0DE, 0xCAFE);
    config.manufacturer = Some("OpenServoCore");
    config.product = Some("uart-pirate");
    config.serial_number = Some("bench-1");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.device_class = 0x02;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, CDC_BULK_PACKET);
    let mut usb = builder.build();

    let usb_fut = usb.run();
    let io_fut = async {
        loop {
            class.wait_connection().await;
            let _ = serve(&mut class).await;
        }
    };
    embassy_futures::join::join(usb_fut, io_fut).await;
}

async fn serve<'d>(class: &mut CdcAcmClass<'d, Driver<'d>>) -> Result<(), EndpointError> {
    let mut rx = [0u8; 64];
    let mut line: Vec<u8, LINE_BUF_LEN> = Vec::new();

    loop {
        let n = class.read_packet(&mut rx).await?;
        for &b in &rx[..n] {
            if b == b'\r' {
                continue;
            }
            if b == b'\n' {
                if let Some(rest) = line.strip_prefix(b"BBATCH ") {
                    handle_batch(class, rest).await?;
                } else if line.as_slice() == b"BICSNAP" {
                    handle_ic_snapshot(class).await?;
                } else {
                    let reply = proto::handle_line(&line);
                    send_reply(class, reply).await?;
                }
                line.clear();
            } else if line.push(b).is_err() {
                line.clear();
                send_reply(class, Reply::Err("overflow")).await?;
            }
        }
    }
}

async fn handle_batch<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d>>,
    rest: &[u8],
) -> Result<(), EndpointError> {
    if let Some(cause) = rx::desync_cause() {
        return send_reply(class, Reply::Err(proto::desync_err_for(cause))).await;
    }
    let req = match proto::parse_batch(rest) {
        Ok(r) => r,
        Err(e) => return send_reply(class, Reply::Err(e)).await,
    };

    let mut records = [ByteRecord {
        tick: 0,
        byte: 0,
        flags: 0,
    }; BBATCH_MAX];
    let want = (req.count as usize).min(BBATCH_MAX);
    // Host-pull: burst tails no longer ride an idle event (self-masking,
    // see rx::isr) — the drain walks on demand instead.
    rx::host_walk();
    let n = rx::drain_batch(&mut records[..want]);

    // Binary frame: sync header 0xA5 0x5A + count:u16 LE + n × (tick:u32 LE,
    // byte:u8, flags:u8). No trailing newline; framing is length-prefixed.
    // The sync header lets a host that lost framing scan-and-relock on the
    // next call.
    let mut hdr: Vec<u8, 4> = Vec::new();
    let _ = hdr.push(0xA5);
    let _ = hdr.push(0x5A);
    let _ = hdr.extend_from_slice(&(n as u16).to_le_bytes());
    class.write_packet(&hdr).await?;

    let mut chunk: Vec<u8, { CDC_BULK_PACKET as usize }> = Vec::new();
    for r in &records[..n] {
        if chunk.len() + BSTAMP_SIZE > chunk.capacity() {
            class.write_packet(&chunk).await?;
            chunk.clear();
        }
        let _ = chunk.extend_from_slice(&r.tick.to_le_bytes());
        let _ = chunk.push(r.byte);
        let _ = chunk.push(r.flags);
    }
    if !chunk.is_empty() {
        class.write_packet(&chunk).await?;
    }
    Ok(())
}

/// Diagnostic IC-ring snapshot. Bypasses the desync guard — meant for
/// post-trip inspection. Wire layout:
///
///   [0xA5][0x5C]
///   [ref_tick:u32 LE][falling_total:u32 LE][walked:u32 LE]
///   [rx_total:u32 LE][byte_head:u32 LE][bit_ticks:u32 LE][cc_filter_delay:u32 LE]
///   [entries:u16 LE]
///   [ticks: u32 LE × entries]    // oldest-first, pre-lifted on chip
///
/// Header = 32 B fits one CDC bulk packet; the tick window streams as
/// `entries × 4 B` over additional packets. Each tick is already lifted
/// from the raw u16 capture into the correct `tick32` wrap by the
/// walker, so the host can compare directly against stamp ticks without
/// re-lifting.
async fn handle_ic_snapshot<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d>>,
) -> Result<(), EndpointError> {
    let mut ticks = [0u32; FALL_LEN];
    let (snap, n) = rx::ic_snapshot(&mut ticks);

    let mut hdr: Vec<u8, 32> = Vec::new();
    let _ = hdr.push(0xA5);
    let _ = hdr.push(0x5C);
    let _ = hdr.extend_from_slice(&snap.ref_tick.to_le_bytes());
    let _ = hdr.extend_from_slice(&snap.falling_total.to_le_bytes());
    let _ = hdr.extend_from_slice(&snap.walked.to_le_bytes());
    let _ = hdr.extend_from_slice(&snap.rx_total.to_le_bytes());
    let _ = hdr.extend_from_slice(&snap.byte_head.to_le_bytes());
    let _ = hdr.extend_from_slice(&snap.bit_ticks.to_le_bytes());
    let _ = hdr.extend_from_slice(&snap.cc_filter_delay.to_le_bytes());
    let _ = hdr.extend_from_slice(&(n as u16).to_le_bytes());
    class.write_packet(&hdr).await?;

    let mut chunk: Vec<u8, { CDC_BULK_PACKET as usize }> = Vec::new();
    for &v in &ticks[..n] {
        if chunk.len() + 4 > chunk.capacity() {
            class.write_packet(&chunk).await?;
            chunk.clear();
        }
        let _ = chunk.extend_from_slice(&v.to_le_bytes());
    }
    if !chunk.is_empty() {
        class.write_packet(&chunk).await?;
    }
    Ok(())
}

async fn send_reply<'d>(
    class: &mut CdcAcmClass<'d, Driver<'d>>,
    reply: Reply,
) -> Result<(), EndpointError> {
    let mut out: Vec<u8, 64> = Vec::new();
    match reply {
        Reply::Ok => {
            let _ = out.extend_from_slice(b"OK\n");
        }
        Reply::Empty => {
            let _ = out.extend_from_slice(b"EMPTY\n");
        }
        Reply::Err(r) => {
            let _ = out.extend_from_slice(b"ERR ");
            let _ = out.extend_from_slice(r.as_bytes());
            let _ = out.push(b'\n');
        }
        Reply::Tick(t) => write_u32(&mut out, b"TICK ", t),
        Reply::Last(t) => write_u32(&mut out, b"LAST ", t),
        Reply::HzPerUs(n) => write_u32(&mut out, b"HZ ", n),
        Reply::BStamp { tick, byte, flags } => write_bstamp(&mut out, tick, byte, flags),
        Reply::BTrace(r) => write_btrace(&mut out, r),
        Reply::Status {
            baud,
            avail,
            cause,
            last_tick,
        } => write_status(&mut out, baud, avail, cause, last_tick),
        Reply::Comp { pipe, bit_q4 } => write_comp(&mut out, pipe, bit_q4),
    }
    debug_assert!(out.len() <= out.capacity());
    class.write_packet(&out).await
}

fn write_status(
    out: &mut Vec<u8, 64>,
    baud: u32,
    avail: u32,
    cause: Option<rx::DesyncCause>,
    last_tick: u32,
) {
    let _ = out.extend_from_slice(b"STATUS ");
    push_dec_u32(out, baud);
    let _ = out.push(b' ');
    push_dec_u32(out, avail);
    let _ = out.push(b' ');
    let _ = out.push(if cause.is_some() { b'1' } else { b'0' });
    let _ = out.push(b' ');
    let cause_str = match cause {
        Some(c) => c.as_str(),
        None => "none",
    };
    let _ = out.extend_from_slice(cause_str.as_bytes());
    let _ = out.push(b' ');
    push_dec_u32(out, last_tick);
    let _ = out.push(b'\n');
}

fn write_comp(out: &mut Vec<u8, 64>, pipe: u32, bit_q4: u32) {
    let _ = out.extend_from_slice(b"COMP pipe=");
    push_dec_u32(out, pipe);
    let _ = out.extend_from_slice(b" bit_q4=");
    push_dec_u32(out, bit_q4);
    let _ = out.push(b'\n');
}

fn write_bstamp(out: &mut Vec<u8, 64>, tick: u32, byte: u8, flags: u8) {
    let _ = out.extend_from_slice(b"BSTAMP ");
    push_dec_u32(out, tick);
    let _ = out.push(b' ');
    push_dec_u32(out, byte as u32);
    let _ = out.push(b' ');
    push_dec_u32(out, flags as u32);
    let _ = out.push(b'\n');
}

fn write_btrace(out: &mut Vec<u8, 64>, r: rx::WalkerTrace) {
    let _ = out.extend_from_slice(b"BTRACE ");
    push_dec_u32(out, r.phase as u32);
    let _ = out.push(b' ');
    push_dec_u32(out, r.tim2_cnt_entry as u32);
    let _ = out.push(b' ');
    push_dec_u32(out, r.tim2_cnt_exit as u32);
    let _ = out.push(b' ');
    push_dec_u32(out, r.falling_pending_entry as u32);
    let _ = out.push(b' ');
    push_dec_u32(out, r.edges_consumed as u32);
    let _ = out.push(b' ');
    push_dec_u32(out, r.bytes_emitted as u32);
    let _ = out.push(b' ');
    push_dec_u32(out, r.falling_total);
    let _ = out.push(b' ');
    push_dec_u32(out, r.rx_total);
    let _ = out.push(b'\n');
}

fn write_u32<const N: usize>(out: &mut Vec<u8, N>, prefix: &[u8], v: u32) {
    let _ = out.extend_from_slice(prefix);
    push_dec_u32(out, v);
    let _ = out.push(b'\n');
}

fn push_dec_u32<const N: usize>(out: &mut Vec<u8, N>, mut v: u32) {
    if v == 0 {
        let _ = out.push(b'0');
        return;
    }
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    while v != 0 {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    let _ = out.extend_from_slice(&buf[i..]);
}

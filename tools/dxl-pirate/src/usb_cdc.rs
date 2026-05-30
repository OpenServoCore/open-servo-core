//! USB-CDC ACM transport. Structure mirrors ch32-hal's `examples/ch32v203/src/bin/usbd.rs`.

use ch32_hal as hal;
use ch32_hal::Peri;
use ch32_hal::peripherals::{PA11, PA12, USBD};
use ch32_hal::usbd::{Driver, Instance, InterruptHandler};
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use heapless::Vec;

use crate::inject::TX_BUF_LEN;
use crate::proto::{self, Reply};

// `FIRE bytes=<TX_BUF_LEN*2 hex> at=<u64>` = TX_BUF_LEN*2 + 35; +slop.
const LINE_BUF_LEN: usize = TX_BUF_LEN * 2 + 64;

hal::bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => InterruptHandler<USBD>;
});

#[embassy_executor::task]
pub async fn run(usbd: Peri<'static, USBD>, dp: Peri<'static, PA12>, dm: Peri<'static, PA11>) {
    let driver = Driver::new(usbd, Irqs, dp, dm);

    let mut config = embassy_usb::Config::new(0xC0DE, 0xCAFE);
    config.manufacturer = Some("OpenServoCore");
    config.product = Some("dxl-pirate");
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

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
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

async fn serve<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), EndpointError> {
    let mut rx = [0u8; 64];
    let mut line: Vec<u8, LINE_BUF_LEN> = Vec::new();

    loop {
        let n = class.read_packet(&mut rx).await?;
        for &b in &rx[..n] {
            if b == b'\r' {
                continue;
            }
            if b == b'\n' {
                let reply = proto::handle_line(&line);
                send_reply(class, reply).await?;
                line.clear();
            } else if line.push(b).is_err() {
                line.clear();
                send_reply(class, Reply::Err("overflow")).await?;
            }
        }
    }
}

async fn send_reply<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
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
        Reply::Tick(t) => write_u64(&mut out, b"TICK ", t),
        Reply::Last(t) => write_u32(&mut out, b"LAST ", t),
        Reply::Req(t) => write_u32(&mut out, b"REQ ", t),
        Reply::First(t) => write_u32(&mut out, b"FIRST ", t),
        Reply::Bytes(n) => write_u32(&mut out, b"BYTES ", n),
        Reply::HzPerUs(n) => write_u32(&mut out, b"HZ ", n),
        Reply::Stamp { tick, head } => write_stamp(&mut out, tick, head),
        Reply::Round { req, first, last, head } => write_round(&mut out, req, first, last, head),
    }
    // Catches a future reply format that overflows the 64-byte Vec — the
    // `let _` swallows above would silently truncate otherwise.
    debug_assert!(out.len() <= out.capacity());
    class.write_packet(&out).await
}

fn write_stamp(out: &mut Vec<u8, 64>, tick: u32, head: u16) {
    let _ = out.extend_from_slice(b"STAMP ");
    push_dec_u32(out, tick);
    let _ = out.push(b' ');
    push_dec_u32(out, head as u32);
    let _ = out.push(b'\n');
}

fn write_round(out: &mut Vec<u8, 64>, req: u32, first: u32, last: u32, head: u16) {
    let _ = out.extend_from_slice(b"ROUND ");
    push_dec_u32(out, req);
    let _ = out.push(b' ');
    push_dec_u32(out, first);
    let _ = out.push(b' ');
    push_dec_u32(out, last);
    let _ = out.push(b' ');
    push_dec_u32(out, head as u32);
    let _ = out.push(b'\n');
}

fn write_u32(out: &mut Vec<u8, 64>, prefix: &[u8], v: u32) {
    let _ = out.extend_from_slice(prefix);
    push_dec_u32(out, v);
    let _ = out.push(b'\n');
}

fn write_u64(out: &mut Vec<u8, 64>, prefix: &[u8], v: u64) {
    let _ = out.extend_from_slice(prefix);
    push_dec_u64(out, v);
    let _ = out.push(b'\n');
}

fn push_dec_u32(out: &mut Vec<u8, 64>, mut v: u32) {
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

fn push_dec_u64(out: &mut Vec<u8, 64>, mut v: u64) {
    if v == 0 {
        let _ = out.push(b'0');
        return;
    }
    let mut buf = [0u8; 20];
    let mut i = buf.len();
    while v != 0 {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    let _ = out.extend_from_slice(&buf[i..]);
}

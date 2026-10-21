//! The adapter's host-session boundary on silicon: a client that opens
//! without SET_CONFIGURATION (WebUSB skips it when configuration 1 is
//! already active) must not inherit the previous session's records once it
//! sends OPEN_SESSION. Its own test binary: every test claims the adapter
//! itself, which the hardware suite's shared wire would hold.
//!
//! ```text
//! cargo test --test usb_session -- --test-threads=1
//! ```

use std::env;
use std::thread::sleep;
use std::time::Duration;

use futures_lite::future;
use futures_timer::Delay;
use nusb::Interface;
use nusb::transfer::{Control, ControlType, Recipient, RequestBuffer, TransferError};
use osc_client::pipe::{PID, VID};
use osc_client::record::{REC_SUBMIT, USB_REQ_OPEN_SESSION};
use osc_client::session::{Record, Session};
use osc_client::{Id, Inst, Opcode};
use osc_host::engine::Command;
use serial_test::serial;

const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;
/// Longer than any reply the tests provoke takes to start arriving.
const QUIET: Duration = Duration::from_millis(300);

fn bench_id() -> Id {
    Id::new(
        env::var("BENCH_ID")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(1),
    )
}

/// Claim the adapter the way WebUSB reopens it: no SET_CONFIGURATION.
fn claim() -> Interface {
    nusb::list_devices()
        .expect("list USB devices")
        .find(|d| d.vendor_id() == VID && d.product_id() == PID)
        .expect("an osc-adapter on USB")
        .open()
        .expect("open the osc-adapter")
        .claim_interface(0)
        .expect("claim the osc-adapter")
}

fn open_session(i: &Interface) -> Result<usize, TransferError> {
    let control = Control {
        control_type: ControlType::Vendor,
        recipient: Recipient::Interface,
        request: USB_REQ_OPEN_SESSION,
        value: 0,
        index: 0,
    };
    i.control_out_blocking(control, &[], Duration::from_secs(1))
}

fn send(i: &Interface, bytes: Vec<u8>) {
    future::block_on(i.bulk_out(EP_OUT, bytes))
        .into_result()
        .expect("bulk OUT");
}

fn hello(i: &Interface) {
    let mut out = Vec::new();
    Session::encode_hello(&mut out);
    send(i, out);
}

/// Every record the adapter delivers until it goes quiet.
fn records(i: &Interface) -> Vec<Record> {
    let mut q = i.bulk_in_queue(EP_IN);
    q.submit(RequestBuffer::new(4096));
    let mut session = Session::new();
    let mut out = Vec::new();
    loop {
        let done = future::block_on(future::or(async { Some(q.next_complete().await) }, async {
            Delay::new(QUIET).await;
            None
        }));
        let Some(done) = done else {
            break;
        };
        session.on_bytes(&done.into_result().expect("bulk IN"));
        q.submit(RequestBuffer::new(4096));
        while let Some(r) = session.next_record().expect("records parse") {
            out.push(r);
        }
    }
    q.cancel_all();
    out
}

/// A session that queues a PING's replies and an INFO, then closes
/// without reading them.
fn abandon_with_replies_queued() {
    let i = claim();
    open_session(&i).expect("OPEN_SESSION");
    let mut session = Session::new();
    let mut out = Vec::new();
    session.encode_submit(
        &mut out,
        &Command::Exchange {
            id: bench_id(),
            inst: Inst::instruction(Opcode::Ping, 0),
            payload: &[],
        },
    );
    Session::encode_hello(&mut out);
    send(&i, out);
    sleep(Duration::from_millis(50));
}

fn only_info(records: &[Record]) -> bool {
    matches!(records, [Record::Info(_)])
}

/// The control for the tests below: without OPEN_SESSION the old records
/// lead the new session, so they would catch a request that does nothing.
#[serial]
#[test]
fn a_reopen_without_open_session_inherits_the_old_records() {
    abandon_with_replies_queued();
    let i = claim();
    hello(&i);
    let got = records(&i);
    assert!(
        got.len() > 1,
        "expected stale records ahead of INFO, got {got:?}"
    );
}

#[serial]
#[test]
fn open_session_drops_the_old_sessions_records() {
    abandon_with_replies_queued();
    let i = claim();
    open_session(&i).expect("OPEN_SESSION");
    hello(&i);
    let got = records(&i);
    assert!(only_info(&got), "expected INFO alone, got {got:?}");
}

#[serial]
#[test]
fn open_session_drops_a_half_received_record() {
    {
        let i = claim();
        open_session(&i).expect("OPEN_SESSION");
        // A SUBMIT whose length promises 10 bytes, cut after 3.
        send(&i, vec![10, 0, REC_SUBMIT, 0, 0]);
    }
    let i = claim();
    open_session(&i).expect("OPEN_SESSION");
    hello(&i);
    let got = records(&i);
    assert!(only_info(&got), "expected INFO alone, got {got:?}");
}

/// The client's own open path sends the request and the adapter accepts
/// it. A rails readback, not a PING: its reply needs no servo at the
/// adapter's current baud, and a stale TERMINAL ahead of it is a desync.
#[serial]
#[test]
fn client_open_sends_open_session() {
    abandon_with_replies_queued();
    let pipe = osc_client::nusb::NusbPipe::open().expect("open with OPEN_SESSION");
    let mut c = osc_client::blocking::Client::connect(pipe).expect("HELLO answered by INFO");
    c.rails()
        .expect("rails readback after an abandoned session");
}

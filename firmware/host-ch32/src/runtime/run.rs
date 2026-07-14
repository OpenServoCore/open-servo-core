//! The main loop: USB pump -> pipe bytes into the link server -> engine
//! events out as records -> USB IN. The transport ISRs run underneath;
//! every main-loop touch of the bus sits inside a critical section.

use osc_host::link::{AdapterRequest, LinkServer, RecordSink};
use osc_host::traits::tick_reached;

use crate::hal::systick;
use crate::providers::edges::Edges;
use crate::providers::pins;
use crate::runtime::{Drivers, iap, init, usb::UsbDevice};

/// Outbound record staging between the sans-io server and USB IN packets.
/// Sized so a whole RX ring of statuses fits; the pump gate below keeps
/// `record` from ever hitting a full queue (a reliable pipe must not drop).
const TXQ_CAP: usize = 4096;

/// Free space required before letting the server produce more records:
/// bounds one `pump`/`on_pipe` call's worst-case output.
const PUMP_HEADROOM: usize = 2048;

/// Pipe flush bound before the bootloader reset: a host that stops reading
/// after requesting the bootloader forfeits its ack.
const FLUSH_BOUND_US: u32 = 100_000;

/// LED pulse stretch per pipe-activity event: long enough to see, short
/// enough that a busy pipe reads as flicker. Dark at idle.
const LED_PULSE_US: u32 = 30_000;

struct TxQueue {
    buf: [u8; TXQ_CAP],
    read: usize,
    len: usize,
}

impl TxQueue {
    fn new() -> Self {
        Self {
            buf: [0; TXQ_CAP],
            read: 0,
            len: 0,
        }
    }

    fn free(&self) -> usize {
        TXQ_CAP - self.len
    }

    fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Dequeue up to `dst.len()` bytes (records are a byte stream; USB
    /// packet boundaries carry no meaning).
    fn pop_into(&mut self, dst: &mut [u8]) -> usize {
        let n = dst.len().min(self.len);
        for item in dst[..n].iter_mut() {
            *item = self.buf[self.read];
            self.read = (self.read + 1) % TXQ_CAP;
        }
        self.len -= n;
        n
    }
}

impl RecordSink for TxQueue {
    fn record(&mut self, record: &[u8]) {
        // SAFETY-net, not flow control: the PUMP_HEADROOM gate makes a
        // full queue unreachable; a breach would corrupt record framing,
        // so the whole record drops instead (and asserts in dev builds).
        debug_assert!(record.len() <= self.free(), "txq overrun");
        if record.len() > self.free() {
            return;
        }
        let mut write = (self.read + self.len) % TXQ_CAP;
        for &b in record {
            self.buf[write] = b;
            write = (write + 1) % TXQ_CAP;
        }
        self.len += record.len();
    }
}

/// Bring the adapter up and serve forever.
pub fn run() -> ! {
    if !init::bringup() {
        fail_blink();
    }

    let mut usb = UsbDevice::new();
    let mut server = LinkServer::new();
    let mut txq = TxQueue::new();
    // Latched out once reached: a bare tick comparison would re-fire every
    // half wrap (~119 s) of the tick domain and strobe the idle LED.
    let mut led_until: Option<u32> = None;

    loop {
        usb.poll();
        // Keep the edge-capture lap accounting honest (main-loop cadence
        // is the overflow detector's sampling clock).
        Edges::poll_accumulate();

        // Inbound pipe bytes -> server -> engine. Held NAK-parked until
        // the queue can absorb the worst-case reply burst.
        if txq.free() >= PUMP_HEADROOM
            && let Some(bytes) = usb.rx()
        {
            critical_section::with(|_| {
                // SAFETY: main-loop bus access under CS (registry doc).
                let bus = unsafe { Drivers::bus() };
                server.on_pipe(bytes, bus, &mut txq);
            });
            usb.rx_consume();
            led_until = Some(systick::ticks().wrapping_add(LED_PULSE_US * systick::TICKS_PER_US));
        }

        match server.take_adapter_request() {
            Some(AdapterRequest::EnterBootloader) => {
                flush_pipe(&mut usb, &mut txq);
                iap::enter_bootloader();
            }
            Some(AdapterRequest::SetRails { v3v3, v5 }) => {
                pins::rail_3v3(v3v3);
                pins::rail_5v(v5);
            }
            None => {}
        }

        // Engine events -> records, same headroom gate.
        if txq.free() >= PUMP_HEADROOM {
            critical_section::with(|_| {
                // SAFETY: main-loop bus access under CS (registry doc).
                let bus = unsafe { Drivers::bus() };
                server.pump(bus, &mut txq);
            });
        }

        if usb.tx_ready() && !txq.is_empty() {
            let mut pkt = [0u8; 512];
            let mps = usb.mps().min(pkt.len());
            let n = txq.pop_into(&mut pkt[..mps]);
            usb.tx(&pkt[..n]);
            led_until = Some(systick::ticks().wrapping_add(LED_PULSE_US * systick::TICKS_PER_US));
        }

        if let Some(until) = led_until
            && tick_reached(systick::ticks(), until)
        {
            led_until = None;
        }
        pins::led(led_until.is_some());
    }
}

/// Drain the queued records (the bootloader ack among them) out the IN
/// endpoint, bounded -- then the port is allowed to die.
fn flush_pipe(usb: &mut UsbDevice, txq: &mut TxQueue) {
    let start = systick::ticks();
    let bound = FLUSH_BOUND_US.saturating_mul(systick::TICKS_PER_US);
    while systick::ticks().wrapping_sub(start) < bound {
        usb.poll();
        if usb.tx_ready() {
            if txq.is_empty() {
                return;
            }
            let mut pkt = [0u8; 512];
            let mps = usb.mps().min(pkt.len());
            let n = txq.pop_into(&mut pkt[..mps]);
            usb.tx(&pkt[..n]);
        }
    }
}

/// Dead crystal: triple pulse forever. Timing runs off whatever clock the
/// loader left (visibly wrong scale is fine -- the pattern is the signal).
fn fail_blink() -> ! {
    loop {
        for _ in 0..3 {
            blink_delay(120_000);
            pins::led(true);
            blink_delay(120_000);
            pins::led(false);
        }
        blink_delay(800_000);
    }
}

fn blink_delay(us: u32) {
    let start = systick::ticks();
    let ticks = us.saturating_mul(systick::TICKS_PER_US);
    while systick::ticks().wrapping_sub(start) < ticks {
        core::hint::spin_loop();
    }
}

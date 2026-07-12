//! USART3 vector — the one wire-side ISR. Entry tick first (it is the
//! boundary recorder's capture), then TC release, break detect, IDLE
//! bookkeeping. This path never reads DATAR (a CPU DATAR read kills a
//! mid-reception byte in the shifter; task #7 / `osc-servo-transport.md`
//! §6 A4).

use ch32_metapac::USART3;
use portable_atomic::{AtomicU32, Ordering};
use qingke_rt::interrupt;

use crate::rx::{boundary, rings};
use crate::tick::read_tick32;

/// `rx_total` at the last progress-making idle service — the zero-
/// progress discriminator's memory (see the idle branch).
static LAST_IDLE_RX_TOTAL: AtomicU32 = AtomicU32::new(0);

/// Clear LBD with a plain all-ones-except-LBD write — never an RMW
/// (STATR flags are rc_w0 and RX runs concurrently; the clear_tc_only
/// argument). LBD is the one boundary flag with a DATAR-free clear,
/// which is why it can retire inside its own service with no storm
/// machinery.
fn clear_lbd_only() {
    USART3.statr().write(|w| {
        w.0 = u32::MAX;
        w.set_lbd(false);
    });
}

#[interrupt]
fn USART3() {
    let now = read_tick32();
    let statr = USART3.statr().read();
    // TCIE is armed per-send by the TX paths; TC here means the last
    // stop bit cleared the shifter — hand the wire back (PB10 → OD).
    if statr.tc() && USART3.ctlr1().read().tcie() {
        crate::tx::on_tx_complete();
    }
    // Break detect (LBD, LIN engine off — see tx::init_usart3): 10
    // dominant bits — every break on the wire, own echoes included
    // (they frame as valid 9-bit characters under the break TX's M=1;
    // foreign breaks frame as FE'd 0x00s — both ring, so records are
    // always attached in normal mode). Cleared before the body so a
    // break landing during the service pends a fresh event.
    if statr.lbd() && USART3.ctlr2().read().lbdie() {
        clear_lbd_only();
        crate::dbg::mark_break();
        boundary::on_break(now);
    }
    if statr.idle() && USART3.ctlr1().read().idleie() {
        // The FLAG stays latched — clearing it here is forbidden (the
        // SR→DR pair's CPU DATAR read kills a mid-reception byte in the
        // shifter, same WCH USART IP as the V006). Mid-stream the flag
        // drain-self-clears (this entry's STATR read arms the SR half;
        // the next byte's RX-DMA DATAR access completes the pair), and
        // every idle DETECTION follows an RXNE.
        let rx = rings::refresh_rx_total();
        // Zero-progress discriminator: an entry with no new RX bytes
        // since the previous idle entry cannot be a fresh detection
        // (those need an RXNE in between) — it is the latched flag
        // level-pending the vector. Mask the event so it cannot storm;
        // the send paths retire the flag under their own drive and
        // re-enable. On edge-pended silicon this branch never runs.
        if rx == LAST_IDLE_RX_TOTAL.load(Ordering::Relaxed) {
            USART3.ctlr1().modify(|w| w.set_idleie(false));
        } else {
            LAST_IDLE_RX_TOTAL.store(rx, Ordering::Relaxed);
            // TC gates the after_idle trigger: a genuine trailing idle
            // has our transmitter drained (TC latches at the last stop
            // bit and stays up), while a service running mid-send (a
            // poll-fed stretch latched the flag early) has a byte still
            // shifting — without the gate it would consume a pending
            // injection mid-request.
            if statr.tc() {
                crate::tx::on_idle(now);
            }
        }
    }
    crate::led::signal();
}

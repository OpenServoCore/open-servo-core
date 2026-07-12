//! Wire-side capture: byte ring + break-boundary recorder.
//!
//! The per-byte edge-stamp pipeline (TIM2/TIM3 input-capture rings, the
//! predict-and-snap walker, the IC filter, the trace ring) is deleted.
//! Capture is two hardware-fed rings and one short ISR:
//!
//! - `rings` — USART3 RX bytes via DMA1_CH3, circular, host-facing.
//! - `boundary` — `(tick32, byte index)` recorded at each USART3
//!   RX-error service: a break rings as exactly one `0x00` and raises FE
//!   1:1, so the service pins a real capture tick to the frame boundary.
//!   Service-entry latency is common-mode — every bench consumer
//!   differences same-flavor boundary ticks (turnaround, chain gaps,
//!   seam spans), so it cancels; only sub-µs service jitter survives.
//! - `stamp` — drain-time synthesis: `BBATCH`/`BDRAIN` walk the byte
//!   ring and emit the same per-byte records as before, with real ticks
//!   on boundary bytes and nominal bit-time strides between them. Zero
//!   CPU per byte at capture time; the wire-side ISR cost is ~1 µs per
//!   frame.
//!
//! One sticky-fatal condition remains: `stamp_overflow` — the host let
//! more than a ring of bytes accumulate undrained, so DMA lapped unread
//! data. Host commands error out until `RESET` (or `BAUD`, which
//! implicitly resets) clears it.

mod boundary;
mod desync;
mod isr;
mod rings;
mod stamp;

use ch32_metapac::Interrupt;

use crate::parse::brr_for;
use crate::tx::{APB1_HZ, BaudError, DEFAULT_BAUD};

pub use desync::{DesyncCause, desync_cause};
pub use stamp::{ByteRecord, drain_batch, drain_byte, stamps_available};

pub fn init() {
    rings::init();
    stamp::set_bit_ticks(APB1_HZ / DEFAULT_BAUD);
    // Drop any boot-transient bytes/flags so the first send anchors clean.
    reset();

    unsafe {
        qingke::pfic::enable_interrupt(Interrupt::USART3 as u8);
    }
}

/// Track a USART baud change: the stride synthesizer's bit time follows
/// the wire. Caller already retuned the USART itself via `tx::set_baud`.
pub fn set_baud(bps: u32) -> Result<(), BaudError> {
    let brr = brr_for(APB1_HZ, bps).ok_or(BaudError::OutOfRange)?;
    stamp::set_bit_ticks(brr);
    // A baud change with bytes mid-flight produces garbage anyway; drop
    // whatever is pending so the next frame anchors fresh.
    reset();
    Ok(())
}

/// Drop all undrained capture state up to the current DMA head and clear
/// the desync flag. Called from init, `set_baud`, and the host `RESET`
/// command — the pending bytes are presumed stale at these points (boot
/// glitches, baud-bounce transients, or the backlog that tripped
/// `stamp_overflow`).
pub fn reset() {
    critical_section::with(|_| {
        let rx_total = rings::refresh_rx_total();
        stamp::reset_to(rx_total);
        boundary::clear(rx_total);
        desync::clear();
    });
}

/// Configured baud (= `APB1_HZ / bit_ticks`). Returns 0 before the first
/// `set_baud` completes.
pub fn current_baud() -> u32 {
    APB1_HZ.checked_div(stamp::bit_ticks()).unwrap_or(0)
}

/// Runtime LIN-mode toggle — the task #26 A/B discriminator. With LIN
/// off the boundary recorder starves (no LBD services; stamps degrade
/// to `COUNT_UNDER` placeholders) but the byte ring keeps ringing, so
/// a probe can ask whether the skew-gated reply corruption lives in
/// the LIN engine or in the plain receiver.
pub fn set_lin(enable: bool) {
    critical_section::with(|_| {
        ch32_metapac::USART3.ctlr2().modify(|w| {
            w.set_linen(enable);
            w.set_lbdie(enable);
        });
    });
}

/// Boundary-recorder health + raw wire-register ground truth for the
/// `BDIAG` probe:
/// `(services, records, head, tail, statr, ctlr1, ctlr2, standalones)`.
pub fn boundary_diag() -> (u32, u32, u32, u32, u32, u32, u32, u32) {
    let (services, records, head, tail, standalones) = boundary::diag();
    let usart = ch32_metapac::USART3;
    (
        services,
        records,
        head,
        tail,
        usart.statr().read().0,
        usart.ctlr1().read().0,
        usart.ctlr2().read().0,
        standalones,
    )
}

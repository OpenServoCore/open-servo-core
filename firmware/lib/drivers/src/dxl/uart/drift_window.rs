//! Per-byte RX wake lifecycle — the driver-owned half of the RXNE
//! cold-start drift window. The codec (`codec::span::DriftWindow`) samples
//! the stamps; this type decides WHEN the window is open and reconciles it
//! with the FAST k > 0 status-start wait, which shares the one RXNEIE bit.
//!
//! The reason set (two bools) is the crux: RXNEIE is a single hardware
//! enable, but two independent consumers want it — the FAST wait and the
//! drift window. Toggling the provider only on the OR-edge keeps a FAST
//! resolution's `unwatch` from closing the window the drift sampler still
//! needs (and vice-versa).
//!
//! Close predicate: the window closes when the integrator CLOSES A BATCH,
//! regardless of outcome — a below-deadband close is a success (drift ≈ 0
//! learned), and the per-byte IRQs must never outlive the sampling they
//! exist for (a perfectly-trimmed chip would otherwise hold ~300k
//! IRQs/sec at 3M forever). Boot bounds the open duration at one packet;
//! a staleness reopen at one steady batch of spans.

use crate::traits::dxl::RxDma;

/// Instructions with no accepted span (natural or window) after which a
/// closed window reopens. Bench/idle bus never forms natural spans, and a
/// bus carrying only pings/short reads would otherwise let temperature
/// drift go untracked once the boot batch close ended the window.
pub(super) const DRIFT_STALENESS_INSTRUCTIONS: u32 = 64;

/// Instruction packets an open window tolerates without producing a single
/// qualifying span before it gives up and closes — never leave per-byte
/// IRQs on indefinitely when the traffic can't feed the sampler (all-short
/// or all-foreign-Status bursts). Counted in instruction packets, the
/// closest cheap proxy the driver has for "packets seen".
pub(super) const DRIFT_WINDOW_MAX_PACKETS: u32 = 4;

/// Which consumer wants the per-byte RX wake held open.
#[derive(Copy, Clone)]
pub(super) enum WakeReason {
    /// A deferred FAST slot k > 0 observing the chain Status packet's start.
    Fast,
    /// The drift sampler's cold-start / staleness window.
    Drift,
}

/// RXNEIE reason set plus the drift window's open/close bookkeeping. Holds
/// no sampling state — that lives in the codec; this only gates the wake
/// and tracks the instruction cursors the lifecycle rules read.
pub(super) struct RxWakeGate {
    fast: bool,
    drift: bool,
    /// `instruction_count` when the drift window last opened — the
    /// MAX_PACKETS give-up anchor.
    open_instr: u32,
    /// `instruction_count` at the last accepted span — the staleness
    /// reopen anchor.
    accept_instr: u32,
    /// A qualifying window span landed since the window opened; disarms the
    /// MAX_PACKETS give-up.
    qualified: bool,
}

impl RxWakeGate {
    pub(super) const fn new() -> Self {
        Self {
            fast: false,
            drift: false,
            open_instr: 0,
            accept_instr: 0,
            qualified: false,
        }
    }

    /// Set one reason on/off and toggle RXNEIE only when the OR of both
    /// reasons crosses an edge.
    fn set(&mut self, rx_dma: &mut impl RxDma, reason: WakeReason, on: bool) {
        let before = self.fast || self.drift;
        match reason {
            WakeReason::Fast => self.fast = on,
            WakeReason::Drift => self.drift = on,
        }
        let after = self.fast || self.drift;
        if after && !before {
            rx_dma.watch_status_start();
        } else if !after && before {
            rx_dma.unwatch_status_start();
        }
    }

    /// The FAST k > 0 wait wants (or no longer wants) the wake.
    pub(super) fn set_fast(&mut self, rx_dma: &mut impl RxDma, on: bool) {
        self.set(rx_dma, WakeReason::Fast, on);
    }

    /// Open the drift window at `instr` (`instruction_count`) — cold boot,
    /// applied baud change, or a staleness reopen.
    pub(super) fn open_drift(&mut self, rx_dma: &mut impl RxDma, instr: u32) {
        self.open_instr = instr;
        self.qualified = false;
        self.set(rx_dma, WakeReason::Drift, true);
    }

    fn close_drift(&mut self, rx_dma: &mut impl RxDma) {
        self.set(rx_dma, WakeReason::Drift, false);
    }

    #[inline(always)]
    pub(super) fn drift_open(&self) -> bool {
        self.drift
    }

    /// Note an accepted span (natural or window) at `instr`. Refreshes the
    /// staleness anchor; a `window` span also disarms the MAX_PACKETS
    /// give-up. Natural drain-ISR (`SpanTracker`) spans call this directly
    /// from the byte-batch / idle handlers.
    pub(super) fn note_accept(&mut self, instr: u32, window: bool) {
        self.accept_instr = instr;
        if window {
            self.qualified = true;
        }
    }

    /// A drift batch closed outside the poll boundary — natural spans
    /// closing a steady batch inside a drain-ISR handler. Same predicate as
    /// [`Self::service`]: any batch close ends the window's job.
    pub(super) fn on_batch_closed(&mut self, rx_dma: &mut impl RxDma) {
        if self.drift {
            self.close_drift(rx_dma);
        }
    }

    /// Reconcile the window at a poll boundary. `batch_closed` = the
    /// integrator closed a batch this poll (boot at the packet end, or a
    /// steady batch filled by the window span) — correction landed or not,
    /// the window's job is done; `window_accepted` = a qualifying window
    /// span was fed; `instr` = current `instruction_count`. Closes on a
    /// batch close or a no-qualifying-span give-up, reopens once a closed
    /// window has gone stale.
    pub(super) fn service(
        &mut self,
        rx_dma: &mut impl RxDma,
        batch_closed: bool,
        window_accepted: bool,
        instr: u32,
    ) {
        if window_accepted {
            self.note_accept(instr, true);
        }
        if self.drift {
            // Qualifying spans reset the give-up, which is fine: the batch
            // close above now bounds how long a fed window stays open.
            let gave_up =
                !self.qualified && instr.wrapping_sub(self.open_instr) >= DRIFT_WINDOW_MAX_PACKETS;
            if batch_closed || gave_up {
                self.close_drift(rx_dma);
            }
        } else if instr.wrapping_sub(self.accept_instr) >= DRIFT_STALENESS_INSTRUCTIONS {
            self.open_drift(rx_dma, instr);
        }
    }
}

#[cfg(test)]
impl RxWakeGate {
    /// Whether the FAST reason currently holds the wake open — the signal
    /// FAST composite tests assert on, since a concurrent drift window can
    /// keep RXNEIE (the mock's `status_start_watched`) high independently.
    pub(super) fn fast_for_test(&self) -> bool {
        self.fast
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::MockRxDma;
    use std::cell::Cell;
    use std::rc::Rc;

    /// RXNEIE state plus watch/unwatch call counts, so tests assert both the
    /// resulting enable AND that the OR-edge toggled the provider exactly
    /// when it should.
    #[derive(Clone, Default)]
    struct WakeState {
        on: Rc<Cell<bool>>,
        watches: Rc<Cell<u32>>,
        unwatches: Rc<Cell<u32>>,
    }

    fn mk_rx_dma() -> (MockRxDma, WakeState) {
        let state = WakeState::default();
        let mut m = MockRxDma::new();
        {
            let (on, n) = (state.on.clone(), state.watches.clone());
            m.expect_watch_status_start().returning_st(move || {
                on.set(true);
                n.set(n.get() + 1);
            });
        }
        {
            let (on, n) = (state.on.clone(), state.unwatches.clone());
            m.expect_unwatch_status_start().returning_st(move || {
                on.set(false);
                n.set(n.get() + 1);
            });
        }
        (m, state)
    }

    #[test]
    fn cold_boot_open_watches_once() {
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.open_drift(&mut rx, 0);
        assert!(s.on.get());
        assert_eq!(s.watches.get(), 1);
        assert!(g.drift_open());
    }

    #[test]
    fn fast_toggles_are_inert_while_drift_holds_the_wake() {
        // The coexistence guarantee: a FAST exchange opening and resolving
        // over an open drift window never toggles RXNEIE.
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.open_drift(&mut rx, 0);
        g.set_fast(&mut rx, true);
        g.set_fast(&mut rx, false);
        assert!(s.on.get(), "drift still holds the wake");
        assert_eq!(s.watches.get(), 1, "no re-watch on the FAST edge");
        assert_eq!(s.unwatches.get(), 0, "FAST resolve did not close it");
    }

    #[test]
    fn fast_alone_toggles_on_its_own_edges() {
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.set_fast(&mut rx, true);
        assert!(s.on.get());
        assert_eq!(s.watches.get(), 1);
        g.set_fast(&mut rx, false);
        assert!(!s.on.get());
        assert_eq!(s.unwatches.get(), 1);
    }

    #[test]
    fn service_closes_the_window_on_a_batch_close() {
        // Any batch close ends the window — the below-deadband (nominal
        // HSI, no correction landed) close included; per-byte IRQs must
        // not outlive the sampling.
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.open_drift(&mut rx, 0);
        g.service(&mut rx, true, true, 1);
        assert!(!s.on.get());
        assert!(!g.drift_open());
        assert_eq!(s.unwatches.get(), 1);
    }

    #[test]
    fn service_keeps_the_window_open_mid_batch() {
        // Steady reopen: qualifying spans accumulate toward the 20-span
        // batch; until it closes the window stays open (the max-packets
        // give-up is disarmed by the qualifying spans, and the batch close
        // bounds the duration).
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.open_drift(&mut rx, 0);
        for instr in 1..=8 {
            g.service(&mut rx, false, true, instr);
        }
        assert!(s.on.get());
        assert!(g.drift_open());
        assert_eq!(s.unwatches.get(), 0);
    }

    #[test]
    fn drain_isr_batch_close_ends_the_window() {
        // Natural spans can fill a steady batch inside `on_rx_advance` /
        // `on_rx_idle`, outside any poll — the window must close there too.
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.open_drift(&mut rx, 0);
        g.on_batch_closed(&mut rx);
        assert!(!s.on.get());
        assert!(!g.drift_open());
    }

    #[test]
    fn service_gives_up_after_max_packets_without_a_qualifying_span() {
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.open_drift(&mut rx, 0);
        // No qualifying span; the instruction cursor reaches the give-up.
        g.service(&mut rx, false, false, DRIFT_WINDOW_MAX_PACKETS - 1);
        assert!(s.on.get(), "not yet at the give-up bound");
        g.service(&mut rx, false, false, DRIFT_WINDOW_MAX_PACKETS);
        assert!(!s.on.get());
        assert!(!g.drift_open());
    }

    #[test]
    fn service_reopens_a_stale_closed_window() {
        let (mut rx, s) = mk_rx_dma();
        let mut g = RxWakeGate::new();
        g.open_drift(&mut rx, 0);
        g.service(&mut rx, true, true, 1); // land + close, accept at instr 1.
        assert!(!s.on.get());
        // Not yet stale.
        g.service(&mut rx, false, false, 1 + DRIFT_STALENESS_INSTRUCTIONS - 1);
        assert!(!s.on.get());
        // Stale → reopen.
        g.service(&mut rx, false, false, 1 + DRIFT_STALENESS_INSTRUCTIONS);
        assert!(s.on.get());
        assert!(g.drift_open());
    }
}

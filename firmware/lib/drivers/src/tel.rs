//! TEL streamer -- serializes the kernel's per-fast-tick sample into the
//! core `tel` frame layout and hands it to a [`TelTx`] transport. Mechanism
//! only: what enables the stream and which fields are selected is table
//! policy upstream.
//!
//! Seq increments per ATTEMPTED frame, sent or not, so a transport-busy
//! drop surfaces at the host as a seq gap instead of silently compressing
//! the timebase; `drops` counts the same events for bench introspection.

use crate::traits::TelTx;
use osc_servo_core::tel;
use osc_servo_core::{TelSample, TelStream};

pub struct Tel<T: TelTx> {
    tx: T,
    enabled: bool,
    mask: u16,
    seq: u8,
    drops: u32,
}

impl<T: TelTx> Tel<T> {
    pub fn new(tx: T) -> Self {
        Self {
            tx,
            enabled: false,
            mask: 0,
            seq: 0,
            drops: 0,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Rejects invalid masks (reserved bits / over budget), keeping the
    /// current one; valid masks take effect at the next tick.
    pub fn set_mask(&mut self, mask: u16) -> bool {
        if !tel::mask_valid(mask) {
            return false;
        }
        self.mask = mask;
        true
    }

    pub fn drops(&self) -> u32 {
        self.drops
    }
}

impl<T: TelTx> TelStream for Tel<T> {
    /// Per-tick table sync (kernel contract): invalid masks are rejected by
    /// `set_mask`, retaining the last valid one - belt only, the table rule
    /// already gates reserved bits at write time.
    fn configure(&mut self, enabled: bool, mask: u16) {
        self.set_enabled(enabled);
        self.set_mask(mask);
    }

    fn on_tick(&mut self, sample: &TelSample) {
        if !self.enabled || self.mask == 0 {
            return;
        }
        let mut buf = [0u8; tel::FRAME_BUDGET];
        let n = tel::encode(self.mask, self.seq, sample, &mut buf);
        self.seq = self.seq.wrapping_add(1);
        if !self.tx.send(&buf[..n]) {
            self.drops = self.drops.saturating_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::mocks::MockTelTx;
    use alloc::vec::Vec;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    #[derive(Clone, Default)]
    struct TxState {
        frames: Rc<RefCell<Vec<Vec<u8>>>>,
        busy: Rc<Cell<bool>>,
    }
    impl TxState {
        fn frames(&self) -> Vec<Vec<u8>> {
            self.frames.borrow().clone()
        }
        fn set_busy(&self, busy: bool) {
            self.busy.set(busy);
        }
    }
    fn mk_tx() -> (MockTelTx, TxState) {
        let state = TxState::default();
        let mut m = MockTelTx::new();
        {
            let frames = state.frames.clone();
            let busy = state.busy.clone();
            m.expect_send().returning_st(move |bytes: &[u8]| {
                if busy.get() {
                    false
                } else {
                    frames.borrow_mut().push(bytes.to_vec());
                    true
                }
            });
        }
        (m, state)
    }

    fn sample() -> TelSample {
        TelSample {
            pos: 2048,
            current: -33,
            current_trough: 400,
            duty_q15: 8000,
            vdiff: 1500,
            vbus: 1822,
            window_valid: true,
        }
    }

    fn streaming_tel(mask: u16) -> (Tel<MockTelTx>, TxState) {
        let (tx, state) = mk_tx();
        let mut t = Tel::new(tx);
        assert!(t.set_mask(mask));
        t.set_enabled(true);
        (t, state)
    }

    #[test]
    fn frames_match_core_encode() {
        for mask in [tel::MASK_ALL, tel::BIT_POS | tel::BIT_VDIFF] {
            let (mut t, state) = streaming_tel(mask);
            let s = sample();
            t.on_tick(&s);
            t.on_tick(&s);
            let mut want = [0u8; tel::FRAME_BUDGET];
            let n = tel::encode(mask, 0, &s, &mut want);
            assert_eq!(state.frames()[0], want[..n]);
            tel::encode(mask, 1, &s, &mut want);
            assert_eq!(state.frames()[1], want[..n]);
        }
    }

    #[test]
    fn disabled_or_mask_zero_sends_nothing() {
        let (tx, state) = mk_tx();
        let mut t = Tel::new(tx);
        t.on_tick(&sample());
        t.set_enabled(true);
        t.on_tick(&sample());
        assert!(t.set_mask(tel::BIT_POS));
        t.set_enabled(false);
        t.on_tick(&sample());
        assert!(state.frames().is_empty());
        assert_eq!(t.drops(), 0);
    }

    #[test]
    fn invalid_mask_rejected_old_mask_retained() {
        let (mut t, state) = streaming_tel(tel::BIT_POS);
        assert!(!t.set_mask(1 << 6));
        assert!(!t.set_mask(tel::MASK_ALL | 1 << 15));
        t.on_tick(&sample());
        assert_eq!(state.frames()[0].len(), tel::frame_len(tel::BIT_POS));
    }

    #[test]
    fn busy_drop_counts_and_leaves_seq_gap() {
        let (mut t, state) = streaming_tel(tel::BIT_POS);
        t.on_tick(&sample());
        state.set_busy(true);
        t.on_tick(&sample());
        state.set_busy(false);
        t.on_tick(&sample());
        assert_eq!(t.drops(), 1);
        let seqs: Vec<u8> = state.frames().iter().map(|f| f[1]).collect();
        assert_eq!(seqs, [0, 2], "dropped frame shows as a seq gap");
    }

    #[test]
    fn seq_wraps() {
        let (mut t, state) = streaming_tel(tel::BIT_POS);
        for _ in 0..=256 {
            t.on_tick(&sample());
        }
        let f = state.frames();
        assert_eq!(f[255][1], 255);
        assert_eq!(f[256][1], 0);
    }
}

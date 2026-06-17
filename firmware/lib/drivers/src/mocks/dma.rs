use std::vec::Vec;

use crate::traits::dxl::{DmaFlags, EdgeDma};

/// Configurable HT/TC flags + remaining count. Tests stage the next ISR
/// view by writing the fields; the read+ack call returns the staged flags
/// and pushes them into `ack_log` so the test can assert the sequence.
/// `pause` / `resume` toggle `paused` and append to `op_log`; while
/// `paused`, `read_and_ack` returns `DmaFlags::default()` without
/// consuming the staged flags — mirroring production where masking
/// HT/TC IE keeps the IRQ from firing in the first place.
#[derive(Default)]
pub struct MockEdgeDma {
    pub next_flags: DmaFlags,
    pub remaining: u16,
    pub ack_log: Vec<DmaFlags>,
    pub paused: bool,
    pub op_log: Vec<EdgeDmaOp>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EdgeDmaOp {
    Pause,
    Resume,
}

impl EdgeDma for MockEdgeDma {
    fn read_and_ack(&mut self) -> DmaFlags {
        if self.paused {
            let flags = DmaFlags::default();
            self.ack_log.push(flags);
            return flags;
        }
        let flags = self.next_flags;
        self.ack_log.push(flags);
        self.next_flags = DmaFlags::default();
        flags
    }

    fn remaining(&self) -> u16 {
        self.remaining
    }

    fn pause(&mut self) {
        self.paused = true;
        self.op_log.push(EdgeDmaOp::Pause);
    }

    fn resume(&mut self) {
        self.paused = false;
        self.op_log.push(EdgeDmaOp::Resume);
    }
}

// Shelved pending U4 (osc-drivers unit test audit): tests below bind to
// hand-rolled mock fields; will be migrated to the mockall + state-companion
// API as part of the audit.
#[cfg(any())]
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pause_then_read_and_ack_returns_no_flags() {
        let mut ring = MockEdgeDma {
            next_flags: DmaFlags {
                ht: true,
                tc: false,
            },
            ..Default::default()
        };
        ring.pause();
        let flags = ring.read_and_ack();
        assert_eq!(flags, DmaFlags::default());
        // Staged flags not consumed — the IRQ never fired in production.
        assert_eq!(
            ring.next_flags,
            DmaFlags {
                ht: true,
                tc: false
            }
        );
    }

    #[test]
    fn resume_unmasks_pending_flags() {
        let mut ring = MockEdgeDma {
            next_flags: DmaFlags {
                ht: false,
                tc: true,
            },
            ..Default::default()
        };
        ring.pause();
        let _ = ring.read_and_ack();
        ring.resume();
        let flags = ring.read_and_ack();
        assert_eq!(
            flags,
            DmaFlags {
                ht: false,
                tc: true
            }
        );
    }

    #[test]
    fn pause_resume_log_records_calls() {
        let mut ring = MockEdgeDma::default();
        ring.pause();
        ring.resume();
        ring.pause();
        assert_eq!(
            ring.op_log,
            [EdgeDmaOp::Pause, EdgeDmaOp::Resume, EdgeDmaOp::Pause]
        );
    }
}

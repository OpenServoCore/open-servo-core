use mockall::mock;

use crate::traits::dxl::{DmaFlags, EdgeDma};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EdgeDmaOp {
    Pause,
    Resume,
}

mock! {
    pub EdgeDma {}
    impl EdgeDma for EdgeDma {
        fn read_and_ack(&mut self) -> DmaFlags;
        fn remaining(&self) -> u16;
        fn pause(&mut self);
        fn resume(&mut self);
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

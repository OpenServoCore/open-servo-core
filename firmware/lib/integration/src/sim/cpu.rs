//! Per-servo PFIC occupancy model. The transport vectors share PFIC HIGH on
//! the chip, so a handler body occupies the CPU and every event landing
//! meanwhile *pends* — a flag per vector, not a queue — and a burst of same-
//! vector events coalesces into one late delivery, exactly as pended IRQs do
//! on silicon. Ring bytes are DMA and always land at their wire tick; only
//! handler invocations defer. Handler effects land at entry: the model
//! charges occupancy, not intra-body effect timing.

use super::core::TICKS_PER_US;

/// Sim-time cost of each `ServoBus` handler body, µs. Zero (the default)
/// delivers every event at its wire tick — the ideal-CPU model the logical
/// suites pin.
#[derive(Copy, Clone, Default)]
pub struct HandlerCost {
    pub on_break_us: u32,
    pub on_deadline_us: u32,
    pub on_tx_complete_us: u32,
}

/// The transport vectors, in same-priority arbitration order (lowest
/// interrupt number delivers first: SysTick, then USART1 — whose real body
/// drains RX errors before TC).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Vector {
    Compare,
    Break,
    TxDone,
}

#[derive(Default)]
pub struct Cpu {
    pub cost: HandlerCost,
    busy_until: u64,
    pend_compare: bool,
    pend_break: bool,
    pend_tx: bool,
    /// A `CpuFree` wake is in flight; at most one outstanding per servo.
    pub free_scheduled: bool,
    delivered_breaks: u64,
}

impl Cpu {
    pub fn busy(&self, now: u64) -> bool {
        now < self.busy_until
    }

    pub fn busy_until(&self) -> u64 {
        self.busy_until
    }

    pub fn pend(&mut self, v: Vector) {
        match v {
            Vector::Compare => self.pend_compare = true,
            Vector::Break => self.pend_break = true,
            Vector::TxDone => self.pend_tx = true,
        }
    }

    pub fn any_pend(&self) -> bool {
        self.pend_compare || self.pend_break || self.pend_tx
    }

    /// Pop the highest-arbitration pended vector, if any.
    pub fn take_pend(&mut self) -> Option<Vector> {
        if self.pend_compare {
            self.pend_compare = false;
            Some(Vector::Compare)
        } else if self.pend_break {
            self.pend_break = false;
            Some(Vector::Break)
        } else if self.pend_tx {
            self.pend_tx = false;
            Some(Vector::TxDone)
        } else {
            None
        }
    }

    /// Occupy the CPU with `v`'s body starting at `now`.
    pub fn charge(&mut self, now: u64, v: Vector) {
        let us = match v {
            Vector::Compare => self.cost.on_deadline_us,
            Vector::Break => {
                self.delivered_breaks += 1;
                self.cost.on_break_us
            }
            Vector::TxDone => self.cost.on_tx_complete_us,
        };
        self.busy_until = now + us as u64 * TICKS_PER_US;
    }

    /// `on_break` invocations actually delivered — the coalescing observable
    /// (wire FE events minus this = pends that merged).
    pub fn delivered_breaks(&self) -> u64 {
        self.delivered_breaks
    }
}

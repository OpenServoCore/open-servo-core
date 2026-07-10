//! Reply sequencing for every reply (`docs/osc-native-protocol.md` §6, §7).
//!
//! Pure state machine: the composite owns the deadline provider and drives
//! this with plain ticks. Every reply is deadline-triggered ≥ reply gap after the
//! frame it answers; a GREAD chain slot `k > 0` additionally waits for `k`
//! predecessor status frames, each with a reclaim window so a silent
//! predecessor can't collapse the tail (§6). A unicast reply is just slot 0.

/// What the chain wants from the composite after an event.
pub enum ChainOut {
    None,
    /// Arm the chain deadline at this absolute tick.
    Wait(u32),
    /// Start the staged reply now. `predecessor_silent` sets the status error
    /// field (§6): true when any reclaim window expired during this chain.
    Trigger {
        predecessor_silent: bool,
    },
}

enum State {
    Idle,
    /// Reply staged; the trigger deadline is armed. `silent` carries whether a
    /// reclaim fired earlier in the chain.
    Pending {
        silent: bool,
    },
    /// A chain slot awaiting `remaining` predecessor status frames. `silent`
    /// latches once any reclaim window expires.
    Waiting {
        remaining: u8,
        silent: bool,
    },
}

pub struct Chain {
    state: State,
    // Baud is fixed for a chain, so these are captured once at staging (§7).
    reply_gap: u32,
    reclaim: u32,
    allowance: u32,
}

impl Default for Chain {
    fn default() -> Self {
        Self::new()
    }
}

impl Chain {
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            reply_gap: 0,
            reclaim: 0,
            allowance: 0,
        }
    }

    pub fn active(&self) -> bool {
        !matches!(self.state, State::Idle)
    }

    /// Awaiting predecessor status frames (§6): a predecessor's break is
    /// expected, so the composite suspends its reclaim window on that break
    /// instead of killing the staged reply.
    pub fn waiting(&self) -> bool {
        matches!(self.state, State::Waiting { .. })
    }

    /// Own reply started, or a new instruction superseded the chain.
    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    /// Own reply staged for `slot` (0 = unicast or first chain slot) after the
    /// instruction frame ended at `end`. `reclaim` covers a predecessor's
    /// trigger → break lead only (§6 keys reclaim off the break, so the
    /// default stays baud-independent); `allowance` bounds how long an
    /// observed break suspends reclaim while its frame plays out.
    pub fn on_reply_staged(
        &mut self,
        slot: u8,
        end: u32,
        reply_gap: u32,
        reclaim: u32,
        allowance: u32,
    ) -> ChainOut {
        self.reply_gap = reply_gap;
        self.reclaim = reclaim;
        self.allowance = allowance;
        if slot == 0 {
            // §7: reply ≥ reply gap after the instruction end, like a unicast read.
            self.state = State::Pending { silent: false };
            ChainOut::Wait(end.wrapping_add(reply_gap))
        } else {
            // §6: wait for `slot` predecessors; the reclaim guards slot 0's own
            // trigger (its trigger is end + reply_gap, so its window ends one
            // reclaim later).
            self.state = State::Waiting {
                remaining: slot,
                silent: false,
            };
            ChainOut::Wait(end.wrapping_add(reply_gap).wrapping_add(reclaim))
        }
    }

    /// A break landed on the wire while we hold a staged reply. §6: reclaim
    /// fires only when a predecessor produces *no break* within its window —
    /// a break means it is alive, so the window suspends for the bounded
    /// frame allowance while its frame plays out. The frame's completion
    /// re-sequences via [`Self::on_status_end`]; a frame that garbles or
    /// wedges instead lets the suspended deadline fire as the reclaim.
    pub fn on_break_observed(&mut self, now: u32) -> ChainOut {
        match self.state {
            State::Waiting { .. } => ChainOut::Wait(now.wrapping_add(self.allowance)),
            _ => ChainOut::None,
        }
    }

    /// A status frame (someone else's — own TX never rings, F9) ended at `end`.
    pub fn on_status_end(&mut self, end: u32) -> ChainOut {
        match self.state {
            State::Waiting { remaining, silent } => {
                let remaining = remaining.saturating_sub(1);
                if remaining == 0 {
                    self.state = State::Pending { silent };
                    ChainOut::Wait(end.wrapping_add(self.reply_gap))
                } else {
                    self.state = State::Waiting { remaining, silent };
                    ChainOut::Wait(end.wrapping_add(self.reply_gap).wrapping_add(self.reclaim))
                }
            }
            // Idle or already pending: a stale snoop, normal (§6).
            _ => ChainOut::None,
        }
    }

    pub fn on_deadline(&mut self, now: u32) -> ChainOut {
        match self.state {
            State::Pending { silent } => {
                self.state = State::Idle;
                ChainOut::Trigger {
                    predecessor_silent: silent,
                }
            }
            State::Waiting { remaining, .. } => {
                // Reclaim expiry: this predecessor was silent (§6).
                let remaining = remaining.saturating_sub(1);
                if remaining == 0 {
                    self.state = State::Idle;
                    ChainOut::Trigger {
                        predecessor_silent: true,
                    }
                } else {
                    // Each silent predecessor's window cascades from its own
                    // missed trigger (now), not the original anchor.
                    self.state = State::Waiting {
                        remaining,
                        silent: true,
                    };
                    ChainOut::Wait(now.wrapping_add(self.reclaim))
                }
            }
            State::Idle => ChainOut::None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const END: u32 = 1000;
    const REPLY_GAP: u32 = 60;
    const RECLAIM: u32 = 600;
    const ALLOWANCE: u32 = 5000;

    fn wait_tick(out: ChainOut) -> u32 {
        match out {
            ChainOut::Wait(t) => t,
            _ => panic!("expected Wait"),
        }
    }

    fn trigger_silent(out: ChainOut) -> bool {
        match out {
            ChainOut::Trigger { predecessor_silent } => predecessor_silent,
            _ => panic!("expected Trigger"),
        }
    }

    #[test]
    fn slot0_triggers_after_reply_gap() {
        let mut c = Chain::new();
        assert_eq!(
            wait_tick(c.on_reply_staged(0, END, REPLY_GAP, RECLAIM, ALLOWANCE)),
            END + REPLY_GAP
        );
        assert!(c.active());
        assert!(!trigger_silent(c.on_deadline(END + REPLY_GAP)));
        assert!(!c.active());
    }

    #[test]
    fn slot2_two_statuses_then_trigger() {
        let mut c = Chain::new();
        // reclaim guards slot 0's trigger.
        assert_eq!(
            wait_tick(c.on_reply_staged(2, END, REPLY_GAP, RECLAIM, ALLOWANCE)),
            END + REPLY_GAP + RECLAIM
        );
        // predecessor 0 replies.
        assert_eq!(wait_tick(c.on_status_end(1200)), 1200 + REPLY_GAP + RECLAIM);
        // predecessor 1 replies → our slot pends, trigger reply gap after it.
        assert_eq!(wait_tick(c.on_status_end(1400)), 1400 + REPLY_GAP);
        assert!(!trigger_silent(c.on_deadline(1400 + REPLY_GAP)));
    }

    #[test]
    fn slot1_reclaim_triggers_silent() {
        let mut c = Chain::new();
        assert_eq!(
            wait_tick(c.on_reply_staged(1, END, REPLY_GAP, RECLAIM, ALLOWANCE)),
            END + REPLY_GAP + RECLAIM
        );
        // No status arrives: the reclaim window expires and we take the slot.
        assert!(trigger_silent(c.on_deadline(END + REPLY_GAP + RECLAIM)));
        assert!(!c.active());
    }

    #[test]
    fn slot3_one_status_then_two_reclaims() {
        let mut c = Chain::new();
        assert_eq!(
            wait_tick(c.on_reply_staged(3, END, REPLY_GAP, RECLAIM, ALLOWANCE)),
            END + REPLY_GAP + RECLAIM
        );
        // predecessor 0 replies (real).
        assert_eq!(wait_tick(c.on_status_end(1200)), 1200 + REPLY_GAP + RECLAIM);
        // predecessor 1 goes silent: reclaim cascades from now.
        assert_eq!(wait_tick(c.on_deadline(1860)), 1860 + RECLAIM);
        // predecessor 2 goes silent: last one → trigger, silent.
        assert!(trigger_silent(c.on_deadline(2460)));
    }

    #[test]
    fn break_suspends_reclaim_for_frame_allowance() {
        let mut c = Chain::new();
        c.on_reply_staged(1, END, REPLY_GAP, RECLAIM, ALLOWANCE);
        // Predecessor's break lands inside its reclaim window: alive — the
        // window suspends for the frame allowance instead of expiring.
        assert_eq!(wait_tick(c.on_break_observed(1100)), 1100 + ALLOWANCE);
        // Its frame completes: normal sequencing resumes.
        assert_eq!(wait_tick(c.on_status_end(1500)), 1500 + REPLY_GAP);
        assert!(!trigger_silent(c.on_deadline(1500 + REPLY_GAP)));
    }

    #[test]
    fn wedged_after_break_reclaims_at_allowance() {
        let mut c = Chain::new();
        c.on_reply_staged(1, END, REPLY_GAP, RECLAIM, ALLOWANCE);
        c.on_break_observed(1100);
        // The frame never resolves (garbled/wedged): the suspended deadline
        // fires as the reclaim.
        assert!(trigger_silent(c.on_deadline(1100 + ALLOWANCE)));
    }

    #[test]
    fn break_while_idle_or_pending_is_none() {
        let mut c = Chain::new();
        assert!(matches!(c.on_break_observed(500), ChainOut::None));
        c.on_reply_staged(0, END, REPLY_GAP, RECLAIM, ALLOWANCE);
        // Pending our own trigger: a break is not a predecessor signal.
        assert!(matches!(c.on_break_observed(1010), ChainOut::None));
    }

    #[test]
    fn status_while_idle_is_none() {
        let mut c = Chain::new();
        assert!(matches!(c.on_status_end(500), ChainOut::None));
        assert!(matches!(c.on_deadline(500), ChainOut::None));
    }

    #[test]
    fn reset_mid_chain_goes_idle() {
        let mut c = Chain::new();
        c.on_reply_staged(2, END, REPLY_GAP, RECLAIM, ALLOWANCE);
        assert!(c.active());
        c.reset();
        assert!(!c.active());
        assert!(matches!(c.on_deadline(9999), ChainOut::None));
    }
}

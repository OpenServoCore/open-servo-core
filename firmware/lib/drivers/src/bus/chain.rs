//! Reply sequencing for every reply (`docs/osc-native-protocol.md` §6, §7).
//!
//! Pure state machine: the composite owns the deadline provider and drives
//! this with plain ticks. Every reply is deadline-triggered ≥ T_turn after the
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
    t_turn: u32,
    reclaim: u32,
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
            t_turn: 0,
            reclaim: 0,
        }
    }

    pub fn active(&self) -> bool {
        !matches!(self.state, State::Idle)
    }

    /// Own reply started, or a new instruction superseded the chain.
    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    /// Own reply staged for `slot` (0 = unicast or first chain slot) after the
    /// instruction frame ended at `end`.
    pub fn on_reply_staged(&mut self, slot: u8, end: u32, t_turn: u32, reclaim: u32) -> ChainOut {
        self.t_turn = t_turn;
        self.reclaim = reclaim;
        if slot == 0 {
            // §7: reply ≥ T_turn after the instruction end, like a unicast read.
            self.state = State::Pending { silent: false };
            ChainOut::Wait(end.wrapping_add(t_turn))
        } else {
            // §6: wait for `slot` predecessors; the reclaim guards slot 0's own
            // trigger (its trigger is end + t_turn, so its window ends one
            // reclaim later).
            self.state = State::Waiting {
                remaining: slot,
                silent: false,
            };
            ChainOut::Wait(end.wrapping_add(t_turn).wrapping_add(reclaim))
        }
    }

    /// A status frame (someone else's — own TX never rings, F9) ended at `end`.
    pub fn on_status_end(&mut self, end: u32) -> ChainOut {
        match self.state {
            State::Waiting { remaining, silent } => {
                let remaining = remaining.saturating_sub(1);
                if remaining == 0 {
                    self.state = State::Pending { silent };
                    ChainOut::Wait(end.wrapping_add(self.t_turn))
                } else {
                    self.state = State::Waiting { remaining, silent };
                    ChainOut::Wait(end.wrapping_add(self.t_turn).wrapping_add(self.reclaim))
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
    const T_TURN: u32 = 60;
    const RECLAIM: u32 = 600;

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
    fn slot0_triggers_after_t_turn() {
        let mut c = Chain::new();
        assert_eq!(
            wait_tick(c.on_reply_staged(0, END, T_TURN, RECLAIM)),
            END + T_TURN
        );
        assert!(c.active());
        assert!(!trigger_silent(c.on_deadline(END + T_TURN)));
        assert!(!c.active());
    }

    #[test]
    fn slot2_two_statuses_then_trigger() {
        let mut c = Chain::new();
        // reclaim guards slot 0's trigger.
        assert_eq!(
            wait_tick(c.on_reply_staged(2, END, T_TURN, RECLAIM)),
            END + T_TURN + RECLAIM
        );
        // predecessor 0 replies.
        assert_eq!(wait_tick(c.on_status_end(1200)), 1200 + T_TURN + RECLAIM);
        // predecessor 1 replies → our slot pends, trigger T_turn after it.
        assert_eq!(wait_tick(c.on_status_end(1400)), 1400 + T_TURN);
        assert!(!trigger_silent(c.on_deadline(1400 + T_TURN)));
    }

    #[test]
    fn slot1_reclaim_triggers_silent() {
        let mut c = Chain::new();
        assert_eq!(
            wait_tick(c.on_reply_staged(1, END, T_TURN, RECLAIM)),
            END + T_TURN + RECLAIM
        );
        // No status arrives: the reclaim window expires and we take the slot.
        assert!(trigger_silent(c.on_deadline(END + T_TURN + RECLAIM)));
        assert!(!c.active());
    }

    #[test]
    fn slot3_one_status_then_two_reclaims() {
        let mut c = Chain::new();
        assert_eq!(
            wait_tick(c.on_reply_staged(3, END, T_TURN, RECLAIM)),
            END + T_TURN + RECLAIM
        );
        // predecessor 0 replies (real).
        assert_eq!(wait_tick(c.on_status_end(1200)), 1200 + T_TURN + RECLAIM);
        // predecessor 1 goes silent: reclaim cascades from now.
        assert_eq!(wait_tick(c.on_deadline(1860)), 1860 + RECLAIM);
        // predecessor 2 goes silent: last one → trigger, silent.
        assert!(trigger_silent(c.on_deadline(2460)));
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
        c.on_reply_staged(2, END, T_TURN, RECLAIM);
        assert!(c.active());
        c.reset();
        assert!(!c.active());
        assert!(matches!(c.on_deadline(9999), ChainOut::None));
    }
}

//! Reply sequencing for every reply (`docs/osc-native-protocol.md` sec 6, sec 7).
//!
//! Pure state machine: the composite owns the deadline provider and drives
//! this with plain ticks. Every reply is deadline-triggered >= reply gap after the
//! frame it answers; a GREAD chain slot `k > 0` additionally waits for `k`
//! predecessor status frames, each with a reclaim window so a silent
//! predecessor can't collapse the tail (sec 6). A unicast reply is just slot 0.

use osc_protocol::wire;

use crate::traits::bus::tick_reached;

/// An error status carries no payload (sec 5.3), whatever the read asked for.
const ERROR_FOOTPRINT: u16 = wire::footprint(wire::len_for(0)) as u16;

/// What the chain wants from the composite after an event.
pub enum ChainOut {
    None,
    /// Arm the chain deadline at this absolute tick.
    Wait(u32),
    /// Start the staged reply now. `predecessor_silent` sets the status error
    /// field (sec 6): true when any reclaim window expired during this chain.
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
    // Baud is fixed for a chain, so these are captured once at staging (sec 7).
    reply_gap: u32,
    reclaim: u32,
    allowance: u32,
    peer_footprint: Option<u16>,
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
            peer_footprint: None,
        }
    }

    pub fn active(&self) -> bool {
        !matches!(self.state, State::Idle)
    }

    /// Awaiting predecessor status frames (sec 6): a predecessor's break is
    /// expected, so the composite suspends its reclaim window on that break
    /// instead of killing the staged reply.
    pub fn waiting(&self) -> bool {
        matches!(self.state, State::Waiting { .. })
    }

    /// Own reply started, or a new instruction superseded the chain.
    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    /// Own reply staged at `now` for `slot` (0 = unicast or first chain
    /// slot) after the instruction frame ended at `end`. `reclaim` covers a
    /// predecessor's trigger -> break lead only (sec 6 keys reclaim off the
    /// break, so the default stays baud-independent); `allowance` bounds how
    /// long an observed break suspends reclaim while its frame plays out.
    /// `peer_footprint` sizes a predecessor's OK status when the GREAD does.
    #[allow(clippy::too_many_arguments)]
    pub fn on_reply_staged(
        &mut self,
        slot: u8,
        end: u32,
        now: u32,
        reply_gap: u32,
        reclaim: u32,
        allowance: u32,
        peer_footprint: Option<u16>,
    ) -> ChainOut {
        self.reply_gap = reply_gap;
        self.reclaim = reclaim;
        self.allowance = allowance;
        self.peer_footprint = peer_footprint;
        if slot == 0 {
            // sec 7: reply >= reply gap after the instruction end, like a unicast read.
            self.state = State::Pending { silent: false };
            ChainOut::Wait(end.wrapping_add(reply_gap))
        } else {
            // sec 6: wait for `slot` predecessors; the reclaim guards slot 0's own
            // trigger, counted from the later of that trigger (end +
            // reply_gap) and this slot's readiness: a backlog delays every
            // servo alike, so a predecessor ready as late as this slot still
            // gets its whole window.
            self.state = State::Waiting {
                remaining: slot,
                silent: false,
            };
            let due = end.wrapping_add(reply_gap);
            let from = if tick_reached(now, due) { now } else { due };
            ChainOut::Wait(from.wrapping_add(reclaim))
        }
    }

    /// A break landed on the wire while we hold a staged reply. sec 6: reclaim
    /// fires only when a predecessor produces *no break* within its window --
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

    /// A predecessor status of this footprint fits the GREAD: an OK status of
    /// the span it sized, or an error status. Its LEN moves the ladder and
    /// times the slot, so one the GREAD rules out was garbled.
    pub fn admits(&self, footprint: u16) -> bool {
        match self.peer_footprint {
            Some(ok) => footprint == ok || footprint == ERROR_FOOTPRINT,
            None => true,
        }
    }

    /// A status frame (someone else's -- own TX never rings, F9) ended at `end`.
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
            // Idle or already pending: a stale snoop, normal (sec 6).
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
                // Reclaim expiry: this predecessor was silent (sec 6).
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
            wait_tick(c.on_reply_staged(0, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None)),
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
            wait_tick(c.on_reply_staged(2, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None)),
            END + REPLY_GAP + RECLAIM
        );
        // predecessor 0 replies.
        assert_eq!(wait_tick(c.on_status_end(1200)), 1200 + REPLY_GAP + RECLAIM);
        // predecessor 1 replies -> our slot pends, trigger reply gap after it.
        assert_eq!(wait_tick(c.on_status_end(1400)), 1400 + REPLY_GAP);
        assert!(!trigger_silent(c.on_deadline(1400 + REPLY_GAP)));
    }

    #[test]
    fn slot1_window_counts_from_a_late_readiness() {
        let mut c = Chain::new();
        let ready = END + REPLY_GAP + RECLAIM + 400;
        assert_eq!(
            wait_tick(c.on_reply_staged(1, END, ready, REPLY_GAP, RECLAIM, ALLOWANCE, None)),
            ready + RECLAIM
        );
        assert_eq!(
            wait_tick(c.on_status_end(ready + 200)),
            ready + 200 + REPLY_GAP
        );
        assert!(!trigger_silent(c.on_deadline(ready + 200 + REPLY_GAP)));
    }

    #[test]
    fn slot1_reclaim_triggers_silent() {
        let mut c = Chain::new();
        assert_eq!(
            wait_tick(c.on_reply_staged(1, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None)),
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
            wait_tick(c.on_reply_staged(3, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None)),
            END + REPLY_GAP + RECLAIM
        );
        // predecessor 0 replies (real).
        assert_eq!(wait_tick(c.on_status_end(1200)), 1200 + REPLY_GAP + RECLAIM);
        // predecessor 1 goes silent: reclaim cascades from now.
        assert_eq!(wait_tick(c.on_deadline(1860)), 1860 + RECLAIM);
        // predecessor 2 goes silent: last one -> trigger, silent.
        assert!(trigger_silent(c.on_deadline(2460)));
    }

    #[test]
    fn break_suspends_reclaim_for_frame_allowance() {
        let mut c = Chain::new();
        c.on_reply_staged(1, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None);
        // Predecessor's break lands inside its reclaim window: alive -- the
        // window suspends for the frame allowance instead of expiring.
        assert_eq!(wait_tick(c.on_break_observed(1100)), 1100 + ALLOWANCE);
        // Its frame completes: normal sequencing resumes.
        assert_eq!(wait_tick(c.on_status_end(1500)), 1500 + REPLY_GAP);
        assert!(!trigger_silent(c.on_deadline(1500 + REPLY_GAP)));
    }

    #[test]
    fn wedged_after_break_reclaims_at_allowance() {
        let mut c = Chain::new();
        c.on_reply_staged(1, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None);
        c.on_break_observed(1100);
        // The frame never resolves (garbled/wedged): the suspended deadline
        // fires as the reclaim.
        assert!(trigger_silent(c.on_deadline(1100 + ALLOWANCE)));
    }

    #[test]
    fn break_while_idle_or_pending_is_none() {
        let mut c = Chain::new();
        assert!(matches!(c.on_break_observed(500), ChainOut::None));
        c.on_reply_staged(0, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None);
        // Pending our own trigger: a break is not a predecessor signal.
        assert!(matches!(c.on_break_observed(1010), ChainOut::None));
    }

    #[test]
    fn sized_gread_admits_its_span_and_error_statuses_only() {
        let mut c = Chain::new();
        c.on_reply_staged(1, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, Some(14));
        assert!(c.admits(14));
        assert!(c.admits(ERROR_FOOTPRINT));
        assert!(!c.admits(13));
        assert!(!c.admits(15));
        c.on_reply_staged(1, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None);
        assert!(c.admits(15));
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
        c.on_reply_staged(2, END, END, REPLY_GAP, RECLAIM, ALLOWANCE, None);
        assert!(c.active());
        c.reset();
        assert!(!c.active());
        assert!(matches!(c.on_deadline(9999), ChainOut::None));
    }
}

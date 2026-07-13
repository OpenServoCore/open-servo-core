//! Single-board GREAD chain timing (`docs/osc-native-protocol.md` sec 6). We put
//! the DUT at a non-zero chain slot by listing a phantom id ahead of it, then
//! watch how it sequences its reply on real silicon -- the timing the DES sim,
//! with its zero-cost handlers, cannot measure.
//!
//! The phantom slot-0 servo does not exist, so this exercises the RECLAIM path:
//! the DUT waits the full `response_deadline_us` window for the silent
//! predecessor, then fires its own reply flagged `PredecessorSilent` -- the
//! chain tail survives a silent predecessor (protocol sec 6, "error replies keep the chain
//! alive") instead of collapsing. The reply must NOT come before the reclaim
//! window elapses, or the DUT gave up on its predecessor too early.
//!
//! The complementary snoop-ADVANCE path (a live predecessor status the DUT sees
//! and advances on) needs a break-framed injection inside the ~60 us reclaim
//! window -- beyond host timing and the pirate's current scheduled-send (no
//! break); it is left to a pirate-firmware follow-up.

use bench::osc::{build_instruction, gread_uniform_payload};
use bench::run::Stats;
use osc_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_core::regions::config::addr::identity::MODEL_NUMBER;
use osc_protocol::wire::{Opcode, ResultCode};
use serial_test::serial;

use crate::support::bench;

const BCAST: u8 = 0xFE;

/// The DUT at chain slot 1 behind a silent phantom must reclaim the predecessor
/// and reply `PredecessorSilent`, and only after waiting out the reclaim window.
#[test]
#[serial]
fn gread_slot1_reclaims_silent_predecessor() {
    let mut b = bench();
    let id = b.id();
    // Top of the unicast range (protocol sec 3.1): never assigned on any bench or
    // fleet, so the slot-0 predecessor is guaranteed silent.
    const PHANTOM_ID: u8 = 249;

    // GREAD listing [phantom, DUT]: the DUT parses its own position as slot 1.
    let wire = build_instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform_payload(MODEL_NUMBER, 2, &[PHANTOM_ID, id]),
    );

    let hz = b.hz_per_us() as f64;
    let mut turnarounds = Vec::new();
    for _ in 0..20 {
        let ex = b.xfer(&wire).expect("chain reply");
        assert_eq!(ex.status.id, id, "the DUT (slot 1) is the responder");
        assert_eq!(
            ex.status.result,
            Some(ResultCode::PredecessorSilent),
            "a silent predecessor must reclaim, not collapse the tail"
        );
        turnarounds.push(ex.turnaround_ticks as f64 / hz);
    }

    let s = Stats::from(&turnarounds).expect("samples");
    s.print();
    // The reply fires at instruction-end + reply gap + reclaim; it must clear the
    // reclaim window (the DUT genuinely waited for its predecessor), and stay in
    // a sane envelope above it.
    let reclaim = DEFAULT_RESPONSE_DEADLINE_US as f64;
    assert!(
        s.min > reclaim,
        "slot-1 reply at {:.1} us came before the {reclaim:.0} us reclaim window",
        s.min
    );
    assert!(
        s.mean < reclaim + 90.0,
        "slot-1 reclaim reply mean {:.1} us far over the reclaim window",
        s.mean
    );
}

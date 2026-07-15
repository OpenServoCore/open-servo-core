//! The wire instrument: raw TX verbs plus hardware-timestamped edge
//! capture, the adapter's 0x6x record family. Deliberate protocol-breaking
//! lives here and only here -- the SUBMIT surface stays valid-by-
//! construction, these verbs bypass the engine's shape validation on
//! purpose (malformed frames are the point of an instrument). One wire
//! owner at a time: a wire op while a command is outstanding answers
//! `Rejected(Busy)`, and vice versa.
//!
//! Capture is continuous from adapter boot, consumer-paced: drain at least
//! every few ms of wire activity or the sticky overflow flag goes up.
//! Edge ticks arrive as the engine domain's low 16 bits, one chronological
//! ring per polarity. The rings merge by alternation (wire edges strictly
//! alternate) and compose onto ONE monotone anchor chain: each tick places
//! at the first u32 at or after its predecessor -- exact for arbitrarily
//! late drains. Unwrapping against the drain moment instead would alias
//! any batch older than a u16 wrap (~3.6 ms at 18 MHz) by whole wraps;
//! multi-ms host scheduling stalls hit that in long suite runs and
//! scrambled ~0.2% of zero-gap burst decodes. Absolute placement across a
//! quiet stretch stays exact as long as some EMPTY drain lands within a
//! wrap of it -- empty drains lift the anchor along `now`.

use crate::client::{Client, desync};
use crate::error::{Error, LinkError, RejectReason};
use crate::pipe::Pipe;
use crate::session::{Record, Session};

/// Line level after a captured edge: `Low` is a falling edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Low,
    High,
}

/// One captured wire edge in the engine tick domain
/// ([`Client::ticks`] converts spans).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WireEdge {
    pub tick: u32,
    pub level: Level,
}

/// One edge drain, time-ordered across both polarities.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeDrain {
    pub edges: Vec<WireEdge>,
    /// A capture ring lapped undrained since the last reset -- ticks before
    /// this drain are untrustworthy. Sticky until [`Client::reset_capture`].
    pub overflow: bool,
    /// The 32-bit engine tick at the drain (the unwrap's anchor floor).
    pub now: u32,
}

impl<P: Pipe> Client<P> {
    /// One law break plus `bytes` verbatim -- no validation, any shape.
    /// Returns the wire-release tick.
    pub async fn wire_send(&mut self, bytes: &[u8]) -> Result<u32, Error> {
        let mut out = Vec::new();
        let seq = self.session.encode_wire_send(&mut out, bytes);
        self.pipe.send(&out).await?;
        self.await_wire_done(seq).await
    }

    /// Break-framed frames back-to-back at bus pace (inter-frame gap under
    /// a character time). Frames must each be 1..=255 bytes -- the train's
    /// length-prefix format cannot carry more, so that much is checked
    /// here; everything else the adapter validates.
    pub async fn wire_burst(&mut self, frames: &[&[u8]]) -> Result<u32, Error> {
        let mut stream = Vec::new();
        for f in frames {
            if f.is_empty() || f.len() > 255 {
                return Err(Error::Link(LinkError::Rejected(RejectReason::Malformed)));
            }
            stream.push(f.len() as u8);
            stream.extend_from_slice(f);
        }
        let mut out = Vec::new();
        let seq = self.session.encode_wire_burst(&mut out, &stream);
        self.pipe.send(&out).await?;
        self.await_wire_done(seq).await
    }

    /// Hold the wire dominant-low for `us`, any width. A pulse >= the
    /// rescue floor (protocol sec 9.1, 300 us) on a powered fleet IS a
    /// rescue declaration -- every servo drops to the rescue baud.
    pub async fn wire_pulse_low(&mut self, us: u16) -> Result<u32, Error> {
        let mut out = Vec::new();
        let seq = self.session.encode_wire_pulse(&mut out, us);
        self.pipe.send(&out).await?;
        self.await_wire_done(seq).await
    }

    /// `announce` as a break-framed frame, then `breaks` bare law breaks
    /// spaced `gap_us` apart on the adapter's crystal grid. The gap is
    /// deliberately decoupled from the announce payload -- a lying CAL
    /// announce injects a known clock-offset reading (protocol sec 9.3
    /// trains through the engine always pace what they announce).
    pub async fn wire_train(
        &mut self,
        announce: &[u8],
        gap_us: u16,
        breaks: u8,
    ) -> Result<u32, Error> {
        let mut out = Vec::new();
        let seq = self
            .session
            .encode_wire_train(&mut out, announce, gap_us, breaks);
        self.pipe.send(&out).await?;
        self.await_wire_done(seq).await
    }

    /// Arbitrary host UART rate, raw bps -- off-catalog divisors allowed
    /// (a detuned host is the clock-tracker's drift injector). Catalog
    /// rate changes belong to [`Client::host_baud`].
    pub async fn wire_baud(&mut self, bps: u32) -> Result<u32, Error> {
        let mut out = Vec::new();
        let seq = self.session.encode_wire_baud(&mut out, bps);
        self.pipe.send(&out).await?;
        self.await_wire_done(seq).await
    }

    /// One capture drain (up to the adapter's per-polarity cap; loop until
    /// empty to exhaust a backlog).
    pub async fn drain_edges(&mut self) -> Result<EdgeDrain, Error> {
        let mut out = Vec::new();
        Session::encode_edge_drain(&mut out, u8::MAX);
        self.pipe.send(&out).await?;
        match self.next_record().await? {
            Record::Edges {
                overflow,
                now,
                falls,
                rises,
            } => Ok(EdgeDrain {
                edges: unwrap_drain(&mut self.anchor, now, &falls, &rises),
                overflow,
                now,
            }),
            other => Err(desync("EDGES", &other)),
        }
    }

    /// Drop undrained captures and the sticky overflow flag.
    pub async fn reset_capture(&mut self) -> Result<(), Error> {
        let mut out = Vec::new();
        Session::encode_capture_reset(&mut out);
        self.pipe.send(&out).await?;
        match self.next_record().await? {
            Record::CaptureAck => {
                self.anchor = EdgeAnchor::default();
                Ok(())
            }
            other => Err(desync("CAPTURE ack", &other)),
        }
    }

    async fn await_wire_done(&mut self, seq: u16) -> Result<u32, Error> {
        match self.next_record().await? {
            Record::WireDone { seq: s, tick } if s == seq => Ok(tick),
            Record::Rejected { seq: s, reason } if s == seq => Err(Error::Link(
                LinkError::Rejected(RejectReason::from_byte(reason)),
            )),
            other => Err(desync("WIRE_DONE", &other)),
        }
    }
}

/// The single unwrap chain over the merged edge stream, cleared by
/// [`Client::reset_capture`]. One wire has one timeline: separate per-ring
/// anchors can land the two polarities on different wrap pages whenever a
/// stalled gap straddles a page boundary between a fall and its rise (a
/// split timeline decodes every character as zero), so both rings feed
/// one chain. `tick` is the composed anchor; `lead` is the polarity the
/// wire produces next -- Low after a reset, because the bus idles high.
/// `carry` holds falls whose partner rises were still undrained at the
/// batch end, deferred until those rises arrive.
pub(crate) struct EdgeAnchor {
    tick: Option<u32>,
    lead: Level,
    carry: Vec<u16>,
}

impl Default for EdgeAnchor {
    fn default() -> Self {
        Self {
            tick: None,
            lead: Level::Low,
            carry: Vec::new(),
        }
    }
}

/// How far behind a drain's `now` the anchor floor sits, capture ticks. An
/// edge can land in the ring after the drain's snapshot yet before its
/// `now` latch, so it arrives one batch later with a tick slightly before
/// that `now` -- the floor must clear the whole snapshot-to-latch window.
/// ~230 us at 18 MHz, orders above the server's us-scale dispatch and
/// well under the 3.6 ms wrap.
const ANCHOR_GUARD: u32 = 4096;

/// Compose one drain's 16-bit ticks into the 32-bit timeline. The two
/// rings merge by ALTERNATION -- edges on a physical wire strictly
/// alternate polarity, and `lead` carries the phase across batches -- then
/// every tick places at the first u32 at or after its predecessor on the
/// single anchor chain (each ring is chronological, and the merge makes
/// the pair chronological). Correct for arbitrarily late drains: a batch
/// delayed past a u16 wrap keeps its exact relative timeline instead of
/// aliasing by whole wraps, and it cannot split across polarities because
/// there is only one chain. Absolute placement needs the anchor within a
/// wrap of the next edge: ONLY an empty drain lifts it (to
/// `now - ANCHOR_GUARD`) -- empty proves the rings held nothing older,
/// while a clamped non-empty drain may still hide backlog a floor must
/// not overtake. Live polling drains empty through quiet stretches, so
/// real gaps stay exact; only a wire gap over a wrap with no empty drain
/// inside it compresses, coherently. Tears (the server snapshots rises
/// before falls, so edges landing between the snapshots ship as surplus
/// trailing falls with their partner rises a batch behind) DEFER: those
/// falls cannot be ordered against rises not yet drained, and the
/// forward-only chain turns any order inversion into a whole-wrap fling.
fn unwrap_drain(anchor: &mut EdgeAnchor, now: u32, falls: &[u16], rises: &[u16]) -> Vec<WireEdge> {
    let falls: Vec<u16> = anchor
        .carry
        .drain(..)
        .chain(falls.iter().copied())
        .collect();
    let (mut fi, mut ri) = (0, 0);
    let mut out = Vec::with_capacity(falls.len() + rises.len());
    let mut a = anchor.tick;
    while fi < falls.len() || ri < rises.len() {
        let level = match (anchor.lead, fi < falls.len(), ri < rises.len()) {
            (Level::Low, true, _) => Level::Low,
            // A rise is due but the rises ring ran dry: the falls left
            // over sit AFTER that undrained rise on the wire. Defer them
            // to merge with the rises of the next drain.
            (Level::High, _, false) => break,
            // Rises available at Low lead: a fall went missing (capture
            // loss); emit the rise so the stream re-syncs.
            _ => Level::High,
        };
        let t = match level {
            Level::Low => {
                fi += 1;
                falls[fi - 1]
            }
            Level::High => {
                ri += 1;
                rises[ri - 1]
            }
        };
        let tick = match a {
            // First edge after a reset seeds at or before `now`.
            None => now.wrapping_sub((now as u16).wrapping_sub(t) as u32),
            Some(prev) => prev.wrapping_add(t.wrapping_sub(prev as u16) as u32),
        };
        a = Some(tick);
        out.push(WireEdge { tick, level });
        anchor.lead = match level {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };
    }
    anchor.carry.extend_from_slice(&falls[fi..]);
    // A pending carry blocks the floor too: the deferred fall is an edge
    // older than `now` still in flight.
    if out.is_empty() && anchor.carry.is_empty() {
        let floor = now.wrapping_sub(ANCHOR_GUARD);
        if a.is_none_or(|prev| floor.wrapping_sub(prev) < u32::MAX / 2) {
            a = Some(floor);
        }
    }
    anchor.tick = a;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const WRAP: u32 = 0x1_0000;

    fn ticks(edges: &[WireEdge]) -> Vec<u32> {
        edges.iter().map(|e| e.tick).collect()
    }

    #[test]
    fn seed_anchors_at_or_before_now() {
        // The head fall one wrap below now's page seeds the chain; its
        // rise chains forward onto the next page.
        let mut a = EdgeAnchor::default();
        let e = unwrap_drain(&mut a, 0x0001_0010, &[0xFFF0], &[0x000C]);
        assert_eq!(e[0].tick, 0x0000_FFF0);
        assert_eq!(e[0].level, Level::Low);
        assert_eq!(e[1].tick, 0x0001_000C);
        assert_eq!(e[1].level, Level::High);
    }

    #[test]
    fn merge_alternates_across_polarities() {
        let mut a = EdgeAnchor::default();
        let e = unwrap_drain(&mut a, 1000, &[100, 300], &[200, 400]);
        assert_eq!(ticks(&e), [100, 200, 300, 400]);
        assert_eq!(e[1].level, Level::High);
        assert_eq!(e[2].level, Level::Low);
    }

    #[test]
    fn seed_survives_now_page_boundary() {
        // now just past a wrap; an edge captured just before it.
        let mut a = EdgeAnchor::default();
        let e = unwrap_drain(&mut a, 0x0002_0003, &[0xFFFE], &[]);
        assert_eq!(e[0].tick, 0x0001_FFFE);
    }

    #[test]
    fn pair_straddling_a_page_boundary_stays_paired() {
        // The break fall sits at the top of a page, its rise just past
        // it. Per-ring unwrapping seeded each polarity independently and
        // could land them a whole wrap apart (a split timeline decodes
        // every char as zero); the single chain makes the split
        // impossible by construction.
        let mut a = EdgeAnchor::default();
        let e = unwrap_drain(&mut a, 0x0002_0004, &[0xFFF8], &[0x0008]);
        assert_eq!(e[0].tick, 0x0001_FFF8);
        assert_eq!(e[0].level, Level::Low);
        assert_eq!(e[1].tick, 0x0002_0008);
        assert_eq!(e[1].level, Level::High);
    }

    #[test]
    fn stalled_batch_keeps_its_true_timeline() {
        // The wrap-alias regression: a batch drained 3+ wraps after
        // capture (host stall). Unwrapped against `now` these edges would
        // land +3 wraps late and sort after later traffic; the chain
        // places them where they happened.
        let mut a = EdgeAnchor {
            tick: Some(1_000),
            lead: Level::Low,
            carry: Vec::new(),
        };
        let now = 1_000 + 3 * WRAP + 200;
        let e = unwrap_drain(&mut a, now, &[2_000, 30_000], &[2_500, 30_500]);
        assert_eq!(ticks(&e), [2_000, 2_500, 30_000, 30_500]);
    }

    #[test]
    fn batch_spanning_wrap_pages_chains_through() {
        // Chronological capture across a u16 page: low bits fold, the
        // chain keeps climbing.
        let mut a = EdgeAnchor {
            tick: Some(0x0000_F000),
            lead: Level::Low,
            carry: Vec::new(),
        };
        let e = unwrap_drain(&mut a, 0x0001_2000, &[0xF800, 0x0100], &[0xFC00, 0x0900]);
        assert_eq!(
            ticks(&e),
            [0x0000_F800, 0x0000_FC00, 0x0001_0100, 0x0001_0900]
        );
    }

    #[test]
    fn empty_drains_carry_the_anchor_across_quiet() {
        // A long quiet stretch (SAVE flash stall) spanned by live polls:
        // the empty drains lift the anchor, so the edge on the far side
        // places at its true absolute tick instead of chaining off the
        // pre-quiet batch and compressing the gap by whole wraps.
        let mut a = EdgeAnchor::default();
        assert_eq!(ticks(&unwrap_drain(&mut a, 1_500, &[1_000], &[])), [1_000]);
        for quiet_now in [WRAP, 2 * WRAP, 3 * WRAP] {
            assert!(unwrap_drain(&mut a, quiet_now, &[], &[]).is_empty());
        }
        let far = 3 * WRAP + 5_000;
        let e = unwrap_drain(&mut a, far + 100, &[], &[far as u16]);
        assert_eq!(ticks(&e), [far]);
        assert_eq!(e[0].level, Level::High, "the quiet span was a low hold");
    }

    #[test]
    fn backlogged_batches_chain_without_the_floor() {
        // A drain clamped at the per-drain cap leaves older edges in the
        // rings while `now` keeps moving. The floor must not rise over
        // that backlog: the next batch has to chain off the last drained
        // edge, not alias up by whole wraps (the 3M hot-loop reply-tail
        // regression -- its ~110 falls span two clamped drains).
        let mut a = EdgeAnchor::default();
        let b1 = unwrap_drain(&mut a, 40_000, &[10_000, 12_000], &[11_000, 13_000]);
        assert_eq!(ticks(&b1), [10_000, 11_000, 12_000, 13_000]);
        let b2 = unwrap_drain(&mut a, 40_500, &[14_000, 20_000], &[15_000, 21_000]);
        assert_eq!(ticks(&b2), [14_000, 15_000, 20_000, 21_000]);
    }

    #[test]
    fn anchor_never_advances_past_in_flight_edges() {
        // An edge captured just after a drain's ring snapshot but before
        // its `now` latch arrives one batch late with a tick slightly
        // before that `now`; the guard keeps the floor below it.
        let mut a = EdgeAnchor::default();
        assert!(unwrap_drain(&mut a, 50_000, &[], &[]).is_empty());
        let late = (50_000 - ANCHOR_GUARD / 2) as u16;
        let e = unwrap_drain(&mut a, 50_100, &[late], &[]);
        assert_eq!(ticks(&e), [late as u32]);
    }

    #[test]
    fn torn_pair_resyncs_across_batches() {
        // The server snapshots the two rings at slightly different
        // instants, so a pair can tear: the fall ships this batch, its
        // rise the next. The lead phase carries over and the rise chains
        // onto the same timeline.
        let mut a = EdgeAnchor::default();
        let b1 = unwrap_drain(&mut a, 1_200, &[1_000, 1_100], &[1_050]);
        assert_eq!(ticks(&b1), [1_000, 1_050, 1_100]);
        assert_eq!(b1[2].level, Level::Low, "dangling fall ends the batch");
        let b2 = unwrap_drain(&mut a, 1_400, &[], &[1_150]);
        assert_eq!(ticks(&b2), [1_150]);
        assert_eq!(b2[0].level, Level::High, "the torn rise pairs back up");
    }

    #[test]
    fn double_tear_defers_the_unpaired_fall() {
        // A rise AND the next fall both land between the server's two
        // ring snapshots: the batch ends fall-fall with the rise a batch
        // behind. Consuming that second fall in place orders it ahead of
        // the missing rise and the chain flings the late rise a whole
        // wrap forward (the debug-suite reply-tail regression); it must
        // wait and merge with the next batch's rises.
        let mut a = EdgeAnchor::default();
        let b1 = unwrap_drain(&mut a, 1_200, &[1_000, 1_040, 1_080], &[1_020]);
        assert_eq!(ticks(&b1), [1_000, 1_020, 1_040]);
        let b2 = unwrap_drain(&mut a, 1_400, &[], &[1_060, 1_100]);
        assert_eq!(ticks(&b2), [1_060, 1_080, 1_100]);
        assert_eq!(b2[0].level, Level::High);
        assert_eq!(b2[1].level, Level::Low, "the deferred fall slots back in");
        assert_eq!(b2[2].level, Level::High);
    }

    #[test]
    fn pending_carry_blocks_the_empty_drain_floor() {
        // A deferred fall is an edge older than `now` still in flight;
        // quiet drains while it waits must not lift the anchor over it.
        let mut a = EdgeAnchor::default();
        let b1 = unwrap_drain(&mut a, 1_200, &[1_000, 1_040, 1_080], &[1_020]);
        assert_eq!(ticks(&b1), [1_000, 1_020, 1_040]);
        assert!(unwrap_drain(&mut a, 3 * WRAP, &[], &[]).is_empty());
        let b3 = unwrap_drain(&mut a, 3 * WRAP + 200, &[], &[1_060, 1_100]);
        assert_eq!(
            ticks(&b3),
            [1_060, 1_080, 1_100],
            "true timeline, not floor-relative"
        );
    }
}

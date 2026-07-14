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
//! Edge ticks arrive as the engine domain's low 16 bits and unwrap against
//! the drain moment -- sound within one u16 wrap (~3.6 ms at 18 MHz), which
//! covers any single exchange; older undrained edges alias.

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
    /// The 32-bit engine tick at the drain, the unwrap reference.
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
                edges: merge_edges(now, &falls, &rises),
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
            Record::CaptureAck => Ok(()),
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

/// Unwrap each 16-bit capture tick to the most recent 32-bit tick at or
/// before `now` with those low bits, then time-order across polarities.
fn merge_edges(now: u32, falls: &[u16], rises: &[u16]) -> Vec<WireEdge> {
    let unwrap = |t: u16| now.wrapping_sub((now as u16).wrapping_sub(t) as u32);
    let mut edges: Vec<WireEdge> = falls
        .iter()
        .map(|&t| (t, Level::Low))
        .chain(rises.iter().map(|&t| (t, Level::High)))
        .map(|(t, level)| WireEdge {
            tick: unwrap(t),
            level,
        })
        .collect();
    edges.sort_by_key(|e| e.tick);
    edges
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unwrap_anchors_at_or_before_now() {
        let e = merge_edges(0x0001_0010, &[0x000C], &[0xFFF0]);
        // 0xFFF0 sits one wrap below now's page; 0x000C on it.
        assert_eq!(e[0].tick, 0x0000_FFF0);
        assert_eq!(e[0].level, Level::High);
        assert_eq!(e[1].tick, 0x0001_000C);
        assert_eq!(e[1].level, Level::Low);
    }

    #[test]
    fn merge_orders_across_polarities() {
        let e = merge_edges(1000, &[100, 300], &[200, 400]);
        let ticks: Vec<u32> = e.iter().map(|e| e.tick).collect();
        assert_eq!(ticks, [100, 200, 300, 400]);
        assert_eq!(e[1].level, Level::High);
        assert_eq!(e[2].level, Level::Low);
    }

    #[test]
    fn unwrap_survives_now_page_boundary() {
        // now just past a wrap; an edge captured just before it.
        let e = merge_edges(0x0002_0003, &[0xFFFE], &[]);
        assert_eq!(e[0].tick, 0x0001_FFFE);
    }
}

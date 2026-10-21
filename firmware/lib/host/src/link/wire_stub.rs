//! `wire` without the `bench` feature: the server never advertises
//! INSTRUMENT, so it answers UNSUPPORTED before reaching here.

use crate::engine::HostBus;
use crate::traits::Providers;

use super::record;
use super::server::RecordSink;

pub(super) fn handle<P: Providers>(
    kind: u8,
    seq: u16,
    _body: &[u8],
    _bus: &mut HostBus<P>,
    out: &mut [u8],
    sink: &mut impl RecordSink,
) -> bool {
    sink.record(record::error(out, seq, kind, record::ERR_UNSUPPORTED));
    false
}

//! `wire` without the `bench` feature: no instrument record is served.

use crate::engine::HostBus;
use crate::traits::Providers;

use super::server::RecordSink;

pub(super) fn handle<P: Providers>(
    _rec: &[u8],
    _bus: &mut HostBus<P>,
    _active: &mut Option<u16>,
    _out: &mut [u8],
    _sink: &mut impl RecordSink,
) -> bool {
    false
}

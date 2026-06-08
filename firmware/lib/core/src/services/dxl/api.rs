use dxl_protocol::CrcUmts;
use dxl_protocol::decoder::{Decoder, Step};
use dxl_protocol::packet::Packet;

use crate::traits::{DxlBus, ServicesIo};
use crate::{Shared, StagedWrites};

use super::dispatcher::Dispatcher;

/// Streaming-decoder accumulator size. Sized to comfortably hold the
/// longest unstuffed frame the chip will ever see (header(8) + max-RW
/// payload + a margin); decoupled from any chip RX-ring size.
const DXL_DECODER_CAP: usize = 256;

pub struct Dxl<C: CrcUmts> {
    staged: StagedWrites,
    decoder: Decoder<DXL_DECODER_CAP, C>,
}

impl<C: CrcUmts> Dxl<C> {
    pub fn new() -> Self {
        Self {
            staged: StagedWrites::new(),
            decoder: Decoder::new(),
        }
    }

    pub fn poll<I>(&mut self, shared: &Shared, io: &mut I)
    where
        I: ServicesIo,
        I::Bus: DxlBus<Crc = C>,
    {
        let (bus, events) = io.parts();
        let got = match bus.rx_window() {
            None => return,
            Some((head, tail)) => self.feed_window(head, tail),
        };
        if !got {
            return;
        }
        // pkt borrows self.decoder, which is disjoint from `bus` —
        // dispatcher gets a fresh `&mut bus` for `bus.send` etc.
        let pkt = self.decoder.dispatch_packet();
        if !matches!(pkt, Packet::Status(_)) {
            bus.snoop();
        }
        let mut d = Dispatcher::new(shared, bus, events, &mut self.staged);
        d.dispatch(pkt);
    }

    fn feed_window(&mut self, head: &[u8], tail: &[u8]) -> bool {
        self.decoder.reset();
        if !head.is_empty() {
            let (step, _) = self.decoder.feed(head);
            if matches!(step, Step::Packet(_)) {
                return true;
            }
        }
        if !tail.is_empty() {
            let (step, _) = self.decoder.feed(tail);
            if matches!(step, Step::Packet(_)) {
                return true;
            }
        }
        false
    }
}

impl<C: CrcUmts> Default for Dxl<C> {
    fn default() -> Self {
        Self::new()
    }
}

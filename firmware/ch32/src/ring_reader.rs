use heapless::Vec;

pub struct RingReader<const N: usize> {
    scratch: Vec<u8, N>,
    last_read_pos: u16,
}

impl<const N: usize> RingReader<N> {
    pub const fn new() -> Self {
        Self {
            scratch: Vec::new(),
            last_read_pos: 0,
        }
    }

    /// Append bytes from `ring[last_read_pos..write_pos]` (mod `ring.len()`) into
    /// scratch. On backpressure (scratch full) the oldest byte is dropped per
    /// pushed byte so the type can't deadlock on persistent garbage.
    ///
    /// `write_pos` must be in `[0, ring.len())`. Circular DMA satisfies this:
    /// NDTR reloads from 0 to N on each cycle, so `N - remaining` ∈ [0, N).
    pub fn ingest(&mut self, ring: &[u8], write_pos: u16) {
        if write_pos == self.last_read_pos {
            return;
        }
        let cap = ring.len() as u16;
        let new_bytes = write_pos.wrapping_sub(self.last_read_pos) % cap;
        for i in 0..new_bytes {
            let idx = (self.last_read_pos.wrapping_add(i) % cap) as usize;
            if self.scratch.is_full() {
                self.drop_front(1);
            }
            let _ = self.scratch.push(ring[idx]);
        }
        self.last_read_pos = write_pos;
    }

    pub fn peek(&self) -> &[u8] {
        self.scratch.as_slice()
    }

    pub fn consume(&mut self, n: usize) {
        self.drop_front(n);
    }

    fn drop_front(&mut self, n: usize) {
        let n = n.min(self.scratch.len());
        if n == 0 {
            return;
        }
        let len = self.scratch.len();
        self.scratch.copy_within(n..len, 0);
        self.scratch.truncate(len - n);
    }
}

impl<const N: usize> Default for RingReader<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_new_bytes_is_noop() {
        let mut r: RingReader<16> = RingReader::new();
        let ring = [0u8; 8];
        r.ingest(&ring, 0);
        assert!(r.peek().is_empty());
    }

    #[test]
    fn contiguous_run() {
        let mut r: RingReader<16> = RingReader::new();
        let ring = [10, 20, 30, 40, 0, 0, 0, 0];
        r.ingest(&ring, 4);
        assert_eq!(r.peek(), &[10, 20, 30, 40]);
    }

    #[test]
    fn straddles_wrap() {
        let mut r: RingReader<16> = RingReader::new();
        let ring = [1, 2, 3, 4, 5, 6, 7, 8];
        r.ingest(&ring, 6);
        r.consume(6);
        // last_read_pos = 6. Producer wraps past 8 back to 2 — new bytes are
        // ring[6], ring[7], ring[0], ring[1].
        let ring2 = [90, 91, 30, 40, 50, 60, 7, 8];
        r.ingest(&ring2, 2);
        assert_eq!(r.peek(), &[7, 8, 90, 91]);
    }

    #[test]
    fn consume_advances() {
        let mut r: RingReader<16> = RingReader::new();
        let ring = [1, 2, 3, 4, 0, 0, 0, 0];
        r.ingest(&ring, 4);
        r.consume(2);
        assert_eq!(r.peek(), &[3, 4]);
    }

    #[test]
    fn consume_more_than_available_clamps() {
        let mut r: RingReader<8> = RingReader::new();
        let ring = [1, 2, 3, 0, 0, 0, 0, 0];
        r.ingest(&ring, 3);
        r.consume(99);
        assert!(r.peek().is_empty());
    }

    #[test]
    fn backpressure_drops_oldest() {
        let mut r: RingReader<4> = RingReader::new();
        let ring = [1, 2, 3, 4, 5, 6, 7, 8];
        r.ingest(&ring, 4);
        r.ingest(&ring, 7);
        assert_eq!(r.peek(), &[4, 5, 6, 7]);
    }

    #[test]
    fn incremental_ingest() {
        let mut r: RingReader<16> = RingReader::new();
        let ring = [1, 2, 3, 4, 5, 6, 7, 8];
        r.ingest(&ring, 3);
        assert_eq!(r.peek(), &[1, 2, 3]);
        r.ingest(&ring, 6);
        assert_eq!(r.peek(), &[1, 2, 3, 4, 5, 6]);
    }
}

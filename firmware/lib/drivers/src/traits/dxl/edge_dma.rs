/// Edge-capture DMA channel handle — the channel that lands RX falling-edge
/// timestamps into the driver-owned `edges` buffer. The buffer itself lives
/// on the driver (`EdgeCapture::edges`); this trait surfaces only the
/// remaining-transfer count (NDTR) the edge publish path derives its
/// producer head from. The driver borrows one through its [`Providers`]
/// bundle.
///
/// [`Providers`]: super::Providers
pub trait EdgeDma {
    fn remaining(&self) -> u16;

    /// Consume the "hardware restarted the ring" flag. A chip that
    /// time-shares the edge-capture channel with the TX kickoff
    /// (`docs/dxl-hw-timed-transport.md` §5/§6) re-arms it from the
    /// buffer base after each scheduled send — NDTR reloads to full and
    /// writes restart at slot 0. Returns `true` exactly once per restart;
    /// the driver's publish path resets its ring bookkeeping before
    /// reading the live head. While the time-share window is open (armed
    /// but not yet restored-and-consumed) [`Self::remaining`] must return
    /// the value latched at arm time — the channel captures nothing in
    /// that window, so the freeze is exact.
    ///
    /// Default `false` for chips whose edge channel is never repurposed.
    fn take_ring_restart(&mut self) -> bool {
        false
    }
}

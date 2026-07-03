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
}

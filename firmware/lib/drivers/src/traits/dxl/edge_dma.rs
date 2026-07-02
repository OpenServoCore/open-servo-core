/// Edge-capture DMA channel handle — controller for the DMA channel
/// that lands TIM2_CH4 IC timestamps into the driver-owned `edges`
/// buffer. The buffer itself lives on the driver (`Rx::edges`); this
/// trait owns the channel's ISR-side surface: read+ack HT/TC flags and
/// read remaining-transfer count (NDTR). The driver borrows one through
/// its type parameter; the production adapter binds to DMA1_CH7.
pub trait EdgeDma {
    fn remaining(&self) -> u16;
}

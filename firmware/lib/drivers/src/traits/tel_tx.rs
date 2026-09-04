/// Hand one encoded TEL frame to the transmit path. `false` = the previous
/// frame is still in flight and this one is dropped; the driver never
/// waits, drop-on-busy is the stream's contract.
pub trait TelTx {
    fn send(&mut self, bytes: &[u8]) -> bool;
}

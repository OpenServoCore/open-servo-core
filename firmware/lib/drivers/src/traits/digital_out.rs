/// Two-state digital line level. Shared across driver trait surfaces
/// ([`DigitalOut::set`], LED patterns, board wiring active-level).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Level {
    Low,
    High,
}

/// Drive a single digital output. Owned by the adapter; the driver holds
/// one `P: DigitalOut` and calls `set` to change the wire state.
pub trait DigitalOut {
    fn set(&mut self, level: Level);
}

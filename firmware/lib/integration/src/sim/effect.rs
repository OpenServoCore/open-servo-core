use crate::sim::SimTime;

/// An effect an actor produces from [`EventSource::advance`](crate::sim::EventSource).
/// The conductor (`Sim::settle`) stamps the producing `DeviceId` when it
/// routes the effect through the bus, so actors don't need to know their
/// own handle.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Effect {
    WireEdge { at: SimTime, rising: bool },
}

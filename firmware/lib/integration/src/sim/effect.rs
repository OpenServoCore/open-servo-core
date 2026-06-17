use crate::sim::{DeviceId, SimTime};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Effect {
    WireEdge {
        source: DeviceId,
        at: SimTime,
        rising: bool,
    },
}

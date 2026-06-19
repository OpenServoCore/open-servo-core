//! Shared transport + sim clock. `Bus` is what [`Sim`](crate::sim::Sim)
//! owns alongside the device registry. Actors never see `Sim` directly —
//! their effects are routed through `Bus` by the conductor.

use crate::sim::{DeviceId, Effect, SimTime, Wire};

pub struct Bus {
    wire: Wire,
    now: SimTime,
}

impl Bus {
    pub fn new() -> Self {
        Self {
            wire: Wire::new(),
            now: SimTime::ZERO,
        }
    }

    pub fn now(&self) -> SimTime {
        self.now
    }

    pub fn set_now(&mut self, t: SimTime) {
        assert!(
            t >= self.now,
            "sim time regressed: {:?} -> {:?}",
            self.now,
            t
        );
        self.now = t;
    }

    pub fn wire(&self) -> &Wire {
        &self.wire
    }

    pub fn wire_mut(&mut self) -> &mut Wire {
        &mut self.wire
    }

    /// Stage each [`Effect::WireEdge`] produced by `source`'s `advance`
    /// onto the wire. Stamping the source here is what lets actors stay
    /// ignorant of their own `DeviceId`.
    pub fn schedule_edges(&mut self, source: DeviceId, effects: Vec<Effect>) {
        for effect in effects {
            match effect {
                Effect::WireEdge { at, rising } => {
                    self.wire.schedule_edge(source, at, rising);
                }
            }
        }
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

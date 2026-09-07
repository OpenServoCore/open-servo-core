//! The fake adapter: the production LinkServer + HostBus over the DES sim
//! with production servo stacks, behind the [`Pipe`] trait. Full-stack CI
//! with no hardware; time is sim time, so every engine window resolves
//! instantly and deterministically.

use osc_integration::sim::{Sim, WireFrame};
use osc_protocol::wire::BaudRate;

use crate::pipe::{Pipe, PipeError};

pub struct FakePipe {
    sim: Sim,
    frames: Vec<WireFrame>,
}

impl FakePipe {
    pub fn new(rate: BaudRate, servo_ids: &[u8]) -> Self {
        let mut sim = Sim::new(rate);
        sim.attach_host();
        sim.attach_link();
        for &id in servo_ids {
            sim.add_servo(id);
        }
        Self {
            sim,
            frames: Vec::new(),
        }
    }

    /// Reach into the rig (seed UIDs, peek tables, read diag counters).
    pub fn sim_mut(&mut self) -> &mut Sim {
        &mut self.sim
    }

    /// Drain the wire frames recorded across every `send`'s sim run --
    /// deterministic timing probes for injection tests.
    pub fn take_frames(&mut self) -> Vec<WireFrame> {
        std::mem::take(&mut self.frames)
    }
}

impl Pipe for FakePipe {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        self.sim.link_send(bytes);
        // Play the whole exchange out; records accumulate for recv.
        self.frames.extend(self.sim.run());
        Ok(())
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        let out = self.sim.link_recv();
        if out.is_empty() {
            // Every send resolves its records synchronously (sim time), so
            // an empty recv means the client awaits something that will
            // never come -- fail fast instead of pending into the guard.
            return Err(PipeError::Io("fake adapter has nothing to deliver".into()));
        }
        Ok(out)
    }

    async fn pause(&mut self, d: std::time::Duration) {
        self.sim.idle(d.as_micros() as u64);
    }
}

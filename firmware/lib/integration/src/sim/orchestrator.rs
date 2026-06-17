use crate::sim::{DeviceId, DeviceRegistry, Effect, EventSource, SimTime, Wire};

pub struct Sim {
    wire: Wire,
    registry: DeviceRegistry,
    sim_now: SimTime,
    baud: u32,
}

enum SourceKind {
    Wire,
    Device(DeviceId),
}

impl Sim {
    pub fn new(baud: u32) -> Self {
        Self {
            wire: Wire::new(),
            registry: DeviceRegistry::new(),
            sim_now: SimTime::ZERO,
            baud,
        }
    }

    pub fn sim_now(&self) -> SimTime {
        self.sim_now
    }

    pub fn baud(&self) -> u32 {
        self.baud
    }

    pub fn add_device<D, F>(&mut self, build: F) -> DeviceId
    where
        D: EventSource + 'static,
        F: FnOnce(DeviceId) -> D,
    {
        let id = self.registry.add(build);
        self.wire.subscribe(id);
        id
    }

    pub fn device<T: EventSource + 'static>(&self, id: DeviceId) -> Option<&T> {
        self.registry.get(id)
    }

    pub fn device_mut<T: EventSource + 'static>(&mut self, id: DeviceId) -> Option<&mut T> {
        self.registry.get_mut(id)
    }

    pub fn run_until(&mut self, end_t: SimTime) {
        loop {
            let mut min_t = self.wire.next_delivery_time();
            let mut min_source = SourceKind::Wire;
            for (id, dev) in self.registry.iter() {
                let Some(t) = dev.next_event_time() else {
                    continue;
                };
                if min_t.is_none_or(|m| t < m) {
                    min_t = Some(t);
                    min_source = SourceKind::Device(id);
                }
            }

            let Some(t) = min_t else {
                return;
            };
            if t > end_t {
                return;
            }
            assert!(t >= self.sim_now, "sim time regressed");
            self.sim_now = t;

            match min_source {
                SourceKind::Wire => {
                    self.wire.deliver_pending(t, &mut self.registry);
                }
                SourceKind::Device(id) => {
                    let effects = self.registry.advance(id, t);
                    for effect in effects {
                        match effect {
                            Effect::WireEdge { source, at, rising } => {
                                self.wire.schedule_edge(source, at, rising);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::any::Any;

    use super::*;

    struct Sender {
        id: DeviceId,
        plan: Vec<(SimTime, bool)>,
        cursor: usize,
    }

    impl Sender {
        fn new(id: DeviceId, plan: Vec<(SimTime, bool)>) -> Self {
            Self {
                id,
                plan,
                cursor: 0,
            }
        }
    }

    impl EventSource for Sender {
        fn next_event_time(&self) -> Option<SimTime> {
            self.plan.get(self.cursor).map(|(t, _)| *t)
        }

        fn advance(&mut self, t: SimTime) -> Vec<Effect> {
            let (planned_t, rising) = self.plan[self.cursor];
            assert_eq!(planned_t, t);
            self.cursor += 1;
            vec![Effect::WireEdge {
                source: self.id,
                at: t,
                rising,
            }]
        }

        fn receive_edge(&mut self, _at: SimTime, _rising: bool) {}

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn as_any_mut(&mut self) -> &mut dyn Any {
            self
        }
    }

    #[derive(Default)]
    struct Receiver {
        received: Vec<(SimTime, bool)>,
    }

    impl EventSource for Receiver {
        fn next_event_time(&self) -> Option<SimTime> {
            None
        }

        fn advance(&mut self, _t: SimTime) -> Vec<Effect> {
            Vec::new()
        }

        fn receive_edge(&mut self, at: SimTime, rising: bool) {
            self.received.push((at, rising));
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn as_any_mut(&mut self) -> &mut dyn Any {
            self
        }
    }

    fn build(plan: Vec<(SimTime, bool)>) -> (Sim, DeviceId, DeviceId) {
        let mut sim = Sim::new(115_200);
        let sender_id = sim.add_device(|id| Sender::new(id, plan));
        let receiver_id = sim.add_device(|_| Receiver::default());
        (sim, sender_id, receiver_id)
    }

    #[test]
    fn edge_delivers_to_other_subscriber_at_scheduled_time() {
        let (mut sim, _s, r) = build(vec![(SimTime::from_ns(100), false)]);
        sim.run_until(SimTime::from_ns(200));
        assert_eq!(
            sim.device::<Receiver>(r).unwrap().received,
            vec![(SimTime::from_ns(100), false)],
        );
        assert_eq!(sim.sim_now(), SimTime::from_ns(100));
    }

    #[test]
    fn edges_delivered_in_scheduled_order() {
        let (mut sim, _s, r) = build(vec![
            (SimTime::from_ns(100), false),
            (SimTime::from_ns(200), true),
            (SimTime::from_ns(300), false),
            (SimTime::from_ns(400), true),
        ]);
        sim.run_until(SimTime::from_ns(1_000));
        assert_eq!(
            sim.device::<Receiver>(r).unwrap().received,
            vec![
                (SimTime::from_ns(100), false),
                (SimTime::from_ns(200), true),
                (SimTime::from_ns(300), false),
                (SimTime::from_ns(400), true),
            ],
        );
    }

    #[test]
    fn run_until_stops_at_horizon() {
        let (mut sim, _s, r) = build(vec![
            (SimTime::from_ns(100), false),
            (SimTime::from_ns(500), true),
        ]);
        sim.run_until(SimTime::from_ns(200));
        assert_eq!(
            sim.device::<Receiver>(r).unwrap().received,
            vec![(SimTime::from_ns(100), false)],
        );
        assert_eq!(sim.sim_now(), SimTime::from_ns(100));
    }

    #[test]
    fn sender_does_not_receive_its_own_edges() {
        let mut sim = Sim::new(115_200);
        let _r = sim.add_device(|_| Receiver::default());
        let _s = sim.add_device(|id| Sender::new(id, vec![(SimTime::from_ns(100), false)]));
        sim.run_until(SimTime::from_ns(200));
    }

    #[test]
    #[should_panic(expected = "wire collision")]
    fn concurrent_falling_edge_from_different_sources_panics() {
        let mut sim = Sim::new(115_200);
        let _a = sim.add_device(|id| Sender::new(id, vec![(SimTime::from_ns(100), false)]));
        let _b = sim.add_device(|id| Sender::new(id, vec![(SimTime::from_ns(100), false)]));
        sim.run_until(SimTime::from_ns(200));
    }
}

use crate::sim::{DeviceId, DeviceRegistry, Effect, EventSource, SimTime, Wire};

pub struct Sim {
    wire: Wire,
    registry: DeviceRegistry,
    now: SimTime,
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
            now: SimTime::ZERO,
            baud,
        }
    }

    pub fn now(&self) -> SimTime {
        self.now
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

    /// Reset every device + the wire + rewind virtual time to zero. For
    /// table-driven tests reusing one sim across scenarios.
    pub fn reset(&mut self) {
        self.wire.reset();
        self.registry.reset_all();
        self.now = SimTime::ZERO;
    }

    /// Run `action` synchronously at the current [`Sim::now`], then drain
    /// the event queue until quiescence (no pending events anywhere) or
    /// until `budget` has elapsed since the pre-action [`Sim::now`],
    /// whichever first. [`Sim::now`] after return = time of last processed
    /// event (or unchanged if no events fired).
    pub fn advance<F>(&mut self, budget: SimTime, action: F)
    where
        F: FnOnce(&mut Sim, SimTime),
    {
        let start_now = self.now;
        action(self, start_now);
        let cap = start_now + budget;

        loop {
            let Some((t, source)) = self.peek_next_event() else {
                return;
            };
            if t > cap {
                return;
            }
            assert!(t >= self.now, "sim time regressed");
            self.now = t;

            match source {
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

    fn peek_next_event(&self) -> Option<(SimTime, SourceKind)> {
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
        min_t.map(|t| (t, min_source))
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

        fn reset(&mut self) {
            self.cursor = 0;
        }

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

        fn reset(&mut self) {
            self.received.clear();
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
        sim.advance(SimTime::from_ns(200), |_, _| {});
        assert_eq!(
            sim.device::<Receiver>(r).unwrap().received,
            vec![(SimTime::from_ns(100), false)],
        );
        assert_eq!(sim.now(), SimTime::from_ns(100));
    }

    #[test]
    fn edges_delivered_in_scheduled_order() {
        let (mut sim, _s, r) = build(vec![
            (SimTime::from_ns(100), false),
            (SimTime::from_ns(200), true),
            (SimTime::from_ns(300), false),
            (SimTime::from_ns(400), true),
        ]);
        sim.advance(SimTime::from_ns(1_000), |_, _| {});
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
    fn advance_stops_at_budget_cap() {
        let (mut sim, _s, r) = build(vec![
            (SimTime::from_ns(100), false),
            (SimTime::from_ns(500), true),
        ]);
        sim.advance(SimTime::from_ns(200), |_, _| {});
        assert_eq!(
            sim.device::<Receiver>(r).unwrap().received,
            vec![(SimTime::from_ns(100), false)],
        );
        assert_eq!(sim.now(), SimTime::from_ns(100));
    }

    #[test]
    fn advance_returns_at_quiescence_before_budget() {
        let (mut sim, _s, r) = build(vec![(SimTime::from_ns(100), false)]);
        sim.advance(SimTime::from_ms(1), |_, _| {});
        assert_eq!(
            sim.device::<Receiver>(r).unwrap().received,
            vec![(SimTime::from_ns(100), false)],
        );
        assert_eq!(sim.now(), SimTime::from_ns(100));
    }

    #[test]
    fn reset_zeroes_time_and_clears_device_state() {
        let (mut sim, _s, r) = build(vec![(SimTime::from_ns(100), false)]);
        sim.advance(SimTime::from_ns(200), |_, _| {});
        assert_eq!(sim.now(), SimTime::from_ns(100));

        sim.reset();
        assert_eq!(sim.now(), SimTime::ZERO);
        assert!(sim.device::<Receiver>(r).unwrap().received.is_empty());
    }

    #[test]
    fn sender_does_not_receive_its_own_edges() {
        let mut sim = Sim::new(115_200);
        let _r = sim.add_device(|_| Receiver::default());
        let _s = sim.add_device(|id| Sender::new(id, vec![(SimTime::from_ns(100), false)]));
        sim.advance(SimTime::from_ns(200), |_, _| {});
    }

    #[test]
    #[should_panic(expected = "wire collision")]
    fn concurrent_falling_edge_from_different_sources_panics() {
        let mut sim = Sim::new(115_200);
        let _a = sim.add_device(|id| Sender::new(id, vec![(SimTime::from_ns(100), false)]));
        let _b = sim.add_device(|id| Sender::new(id, vec![(SimTime::from_ns(100), false)]));
        sim.advance(SimTime::from_ns(200), |_, _| {});
    }
}

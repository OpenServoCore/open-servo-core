use std::any::type_name;

use crate::sim::{Bus, DeviceId, DeviceRegistry, EventSource, Host, Servo, SimTime};

pub struct Sim {
    bus: Bus,
    devices: DeviceRegistry,
}

enum SourceKind {
    Wire,
    Device(DeviceId),
}

impl Sim {
    pub fn new() -> Self {
        Self {
            bus: Bus::new(),
            devices: DeviceRegistry::new(),
        }
    }

    pub fn now(&self) -> SimTime {
        self.bus.now()
    }

    pub fn bus(&self) -> &Bus {
        &self.bus
    }

    pub fn add_device<D, F>(&mut self, build: F) -> DeviceId
    where
        D: EventSource + 'static,
        F: FnOnce(DeviceId) -> D,
    {
        let id = self.devices.add(build);
        self.bus.wire_mut().subscribe(id);
        id
    }

    /// Read-only handle to a device. Panics if `id` is unknown or the
    /// device isn't of type `T` — both are programmer errors, never
    /// recovered from at runtime.
    pub fn device<T: EventSource + 'static>(&self, id: DeviceId) -> &T {
        self.devices
            .get(id)
            .unwrap_or_else(|| panic!("Sim::device: no {:?} of type {}", id, type_name::<T>()))
    }

    /// Mutable handle to a device. Bumps the device's `now` cursor to
    /// [`Sim::now`] before returning so callers queuing work see current
    /// sim time instead of the device's stale cursor from the previous
    /// quiescence. Panics under the same conditions as [`Sim::device`].
    pub fn device_mut<T: EventSource + 'static>(&mut self, id: DeviceId) -> &mut T {
        let now = self.bus.now();
        self.devices.set_now_one(id, now);
        self.devices
            .get_mut(id)
            .unwrap_or_else(|| panic!("Sim::device_mut: no {:?} of type {}", id, type_name::<T>()))
    }

    pub fn host(&self, id: DeviceId) -> &Host {
        self.device::<Host>(id)
    }

    pub fn host_mut(&mut self, id: DeviceId) -> &mut Host {
        self.device_mut::<Host>(id)
    }

    pub fn servo(&self, id: DeviceId) -> &Servo {
        self.device::<Servo>(id)
    }

    pub fn servo_mut(&mut self, id: DeviceId) -> &mut Servo {
        self.device_mut::<Servo>(id)
    }

    /// Mutate a device through `f`, then drive the simulation forward to
    /// quiescence. Use for the dominant test pattern: queue a host send +
    /// open a `wait_for_reply` predicate in one closure, let `settle`
    /// drain the wire.
    pub fn with_device<T, F>(&mut self, id: DeviceId, f: F)
    where
        T: EventSource + 'static,
        F: FnOnce(&mut T),
    {
        f(self.device_mut::<T>(id));
        self.settle();
    }

    /// Sugar over `with_device::<Host>(...)`. The host-side exchange
    /// pattern (`send_X` + `wait_for_reply`) is by far the most common
    /// test shape, so it's worth the dedicated entry point.
    pub fn with_host<F>(&mut self, id: DeviceId, f: F)
    where
        F: FnOnce(&mut Host),
    {
        self.with_device::<Host, _>(id, f);
    }

    /// Drain the event queue until every scheduled event is processed.
    /// Each iteration picks the earliest scheduled event (wire delivery
    /// or actor wake) and routes it. Termination is driven by actors
    /// exhausting their `next_event_time` — `Host::wait_for_reply` keeps
    /// settle alive via the dynamic wait deadline; once the deadline
    /// holds and `advance` clears the predicate, the host stops
    /// reporting wakes and the loop falls through.
    pub fn settle(&mut self) {
        while let Some((t, source)) = self.peek_next_event() {
            self.bus.set_now(t);

            match source {
                SourceKind::Wire => {
                    log::trace!("engine: drain wire t={:?}", t);
                    let pending = self.bus.wire_mut().take_pending(t);
                    for (target, at, rising) in pending {
                        self.devices.receive_edge(target, at, rising);
                    }
                }
                SourceKind::Device(id) => {
                    log::trace!("engine: drain device {:?} t={:?}", id, t);
                    let effects = self.devices.advance(id, t);
                    self.bus.schedule_edges(id, effects);
                }
            }
        }
    }

    fn peek_next_event(&self) -> Option<(SimTime, SourceKind)> {
        let mut min_t = self.bus.wire().next_delivery_time();
        let mut min_source = SourceKind::Wire;
        for (id, dev) in self.devices.iter() {
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

impl Default for Sim {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use std::any::Any;

    use super::*;
    use crate::sim::Effect;

    struct Sender {
        plan: Vec<(SimTime, bool)>,
        cursor: usize,
    }

    impl Sender {
        fn new(plan: Vec<(SimTime, bool)>) -> Self {
            Self { plan, cursor: 0 }
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
            vec![Effect::WireEdge { at: t, rising }]
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
        let mut sim = Sim::default();
        let sender_id = sim.add_device(|_| Sender::new(plan));
        let receiver_id = sim.add_device(|_| Receiver::default());
        (sim, sender_id, receiver_id)
    }

    #[test]
    fn edge_delivers_to_other_subscriber_at_scheduled_time() {
        let (mut sim, _s, r) = build(vec![(SimTime::from_ns(100), false)]);
        sim.settle();
        assert_eq!(
            sim.device::<Receiver>(r).received,
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
        sim.settle();
        assert_eq!(
            sim.device::<Receiver>(r).received,
            vec![
                (SimTime::from_ns(100), false),
                (SimTime::from_ns(200), true),
                (SimTime::from_ns(300), false),
                (SimTime::from_ns(400), true),
            ],
        );
    }

    #[test]
    fn sender_does_not_receive_its_own_edges() {
        let mut sim = Sim::default();
        let _r = sim.add_device(|_| Receiver::default());
        let _s = sim.add_device(|_| Sender::new(vec![(SimTime::from_ns(100), false)]));
        sim.settle();
    }

    #[test]
    #[should_panic(expected = "wire collision")]
    fn concurrent_falling_edge_from_different_sources_panics() {
        let mut sim = Sim::default();
        let _a = sim.add_device(|_| Sender::new(vec![(SimTime::from_ns(100), false)]));
        let _b = sim.add_device(|_| Sender::new(vec![(SimTime::from_ns(100), false)]));
        sim.settle();
    }

    #[test]
    #[should_panic(expected = "wire collision")]
    fn falling_edge_from_same_source_while_line_low_panics() {
        let mut sim = Sim::default();
        let _r = sim.add_device(|_| Receiver::default());
        let _s = sim.add_device(|_| {
            Sender::new(vec![
                (SimTime::from_ns(100), false),
                (SimTime::from_ns(150), false),
            ])
        });
        sim.settle();
    }
}

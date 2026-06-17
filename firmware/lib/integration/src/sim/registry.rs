use std::collections::HashMap;

use crate::sim::{Effect, EventSource, SimTime};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct DeviceId(u32);

struct DeviceEntry {
    id: DeviceId,
    src: Box<dyn EventSource>,
}

pub struct DeviceRegistry {
    devices: Vec<DeviceEntry>,
    id_to_index: HashMap<DeviceId, usize>,
    next_id: u32,
}

impl DeviceRegistry {
    pub fn new() -> Self {
        Self {
            devices: Vec::new(),
            id_to_index: HashMap::new(),
            next_id: 0,
        }
    }

    pub fn add<D, F>(&mut self, build: F) -> DeviceId
    where
        D: EventSource + 'static,
        F: FnOnce(DeviceId) -> D,
    {
        let id = DeviceId(self.next_id);
        self.next_id += 1;
        let idx = self.devices.len();
        let dev = build(id);
        self.devices.push(DeviceEntry {
            id,
            src: Box::new(dev),
        });
        self.id_to_index.insert(id, idx);
        id
    }

    pub fn get<T: EventSource + 'static>(&self, id: DeviceId) -> Option<&T> {
        let idx = *self.id_to_index.get(&id)?;
        self.devices[idx].src.as_any().downcast_ref::<T>()
    }

    pub fn get_mut<T: EventSource + 'static>(&mut self, id: DeviceId) -> Option<&mut T> {
        let idx = *self.id_to_index.get(&id)?;
        self.devices[idx].src.as_any_mut().downcast_mut::<T>()
    }

    pub fn iter(&self) -> impl Iterator<Item = (DeviceId, &dyn EventSource)> {
        self.devices.iter().map(|e| (e.id, &*e.src))
    }

    pub fn receive_edge(&mut self, id: DeviceId, at: SimTime, rising: bool) {
        let idx = self.index_of(id, "receive_edge");
        self.devices[idx].src.receive_edge(at, rising);
    }

    pub fn advance(&mut self, id: DeviceId, t: SimTime) -> Vec<Effect> {
        let idx = self.index_of(id, "advance");
        self.devices[idx].src.advance(t)
    }

    pub fn reset_all(&mut self) {
        for entry in &mut self.devices {
            entry.src.reset();
        }
    }

    fn index_of(&self, id: DeviceId, op: &str) -> usize {
        *self
            .id_to_index
            .get(&id)
            .unwrap_or_else(|| panic!("DeviceRegistry::{op}: unknown DeviceId"))
    }
}

impl Default for DeviceRegistry {
    fn default() -> Self {
        Self::new()
    }
}

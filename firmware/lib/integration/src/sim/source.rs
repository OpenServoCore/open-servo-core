use std::any::Any;

use crate::sim::{Effect, SimTime};

pub trait EventSource {
    fn next_event_time(&self) -> Option<SimTime>;
    fn advance(&mut self, t: SimTime) -> Vec<Effect>;
    fn receive_edge(&mut self, at: SimTime, rising: bool);
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

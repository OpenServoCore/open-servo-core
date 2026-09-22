//! `edges` without the `bench` feature: no capture organ, so the runtime's
//! call sites carry no cfg.

pub struct Edges;

impl Edges {
    pub fn arm() {}

    pub fn poll_accumulate() {}
}

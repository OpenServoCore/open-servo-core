//! Domain primitives shared across the driver trait surface.

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Level {
    Low,
    High,
}

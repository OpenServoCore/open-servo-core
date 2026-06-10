//! Project-wide domain types shared across layers. Lives at the crate root
//! so drivers, adapters, and HAL can all import without crossing layer
//! boundaries (cf. `BaudRate`).

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Level {
    Low,
    High,
}

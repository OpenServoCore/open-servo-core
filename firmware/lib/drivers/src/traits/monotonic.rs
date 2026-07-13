/// Free-running monotonic tick counter, used by drivers that schedule by
/// elapsed time. `TICKS_PER_US` describes the rate so the driver can
/// convert us <-> ticks without importing chip constants.
pub trait Monotonic {
    const TICKS_PER_US: u32;
    fn ticks(&self) -> u32;
}

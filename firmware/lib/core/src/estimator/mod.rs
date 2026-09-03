//! Count-domain estimator blocks the kernel runs on a `SensorFrame`.

pub mod vcal;
pub mod window;

pub use vcal::VcalLpf;
pub use window::WindowSel;

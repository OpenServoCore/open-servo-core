//! Count-domain estimator blocks the kernel runs on a `SensorFrame`.

pub mod bemf;
pub mod vcal;
pub mod window;

pub use bemf::BemfObs;
pub use vcal::VcalLpf;
pub use window::WindowSel;

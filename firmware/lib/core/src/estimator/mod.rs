//! Count-domain estimator blocks the kernel runs on a `SensorFrame`.

pub mod bemf;
pub mod bias;
pub mod fusion;
pub mod thermal;
pub mod vbus;
pub mod vcal;
pub mod window;

pub use bemf::BemfObs;
pub use bias::BiasTracker;
pub use fusion::{FusionGains, FusionObs};
pub use thermal::{ThermAnchor, ThermGates, WindingTherm};
pub use vbus::VbusEst;
pub use vcal::VcalLpf;
pub use window::WindowSel;

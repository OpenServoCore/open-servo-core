//! Count-domain estimator blocks the kernel runs on a `SensorFrame`.

pub mod bemf;
pub mod bias;
#[cfg(not(feature = "experiment"))]
pub mod fusion;
#[cfg(not(feature = "experiment"))]
pub mod omega_switch;
pub mod thermal;
#[cfg(not(feature = "experiment"))]
pub mod vbus;
pub mod window;

pub use bemf::BemfObs;
pub use bias::BiasTracker;
#[cfg(not(feature = "experiment"))]
pub use fusion::FusionObs;
#[cfg(not(feature = "experiment"))]
pub use omega_switch::{OmegaSource, OmegaSwitch};
pub use thermal::{NtcCfg, ThermCfg, WindingTherm};
#[cfg(not(feature = "experiment"))]
pub use vbus::VbusEst;
pub use window::WindowSel;

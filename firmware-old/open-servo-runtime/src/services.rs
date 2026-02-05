//! Service orchestration.
//!
//! The concrete [`Services`] bundle lives in the `open-servo-services` crate.
//!
//! This module previously contained a stub implementation. Now use:
//!
//! ```rust,ignore
//! use open_servo_services::Services;
//! ```
//!
//! The split exists because `services` depends on `runtime` for [`ShadowStorage`],
//! so `runtime` cannot depend on `services` without creating a circular dependency.
//!
//! [`Services`]: https://docs.rs/open-servo-services
//! [`ShadowStorage`]: crate::shadow_storage::ShadowStorage

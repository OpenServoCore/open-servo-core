//! Service context for per-call resource access.
//!
//! [`ServiceCtx`] bundles shared resources that service tasks need access to.
//! Instead of storing references in tasks (which creates self-referential structs),
//! the context is passed to task methods per-call.
//!
//! # Design
//!
//! ```text
//! Runtime owns:
//!   - ShadowStorage (implements HostShadow)
//!   - PersistTask (owns flash, signals, cache)
//!   - RpcTask (owns transport)
//!
//! Per-call:
//!   runtime.persist_loop() creates ServiceCtx { shadow, timer }
//!   and passes it to persist_task.handle_once(&ctx)
//! ```

use open_servo_hw::v2::AsyncTimer;
use open_servo_shadow::HostShadow;

/// Service context providing access to shared runtime resources.
///
/// Passed to service task methods per-call to avoid self-referential structs.
///
/// # Generic Parameters
///
/// - `S`: Shadow access type implementing [`HostShadow`]
/// - `T`: Async timer type for delays and timestamps
pub struct ServiceCtx<'a, S: HostShadow, T: AsyncTimer> {
    /// Shadow storage for register access.
    pub shadow: &'a S,
    /// Async timer for delays and timestamps.
    pub timer: &'a T,
}

impl<'a, S: HostShadow, T: AsyncTimer> ServiceCtx<'a, S, T> {
    /// Create a new service context.
    #[inline]
    pub fn new(shadow: &'a S, timer: &'a T) -> Self {
        Self { shadow, timer }
    }
}

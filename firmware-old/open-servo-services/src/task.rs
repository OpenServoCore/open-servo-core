//! Service task trait.
//!
//! Uniform interface for async services.
//!
//! ## Design
//!
//! Each service implements [`ServiceTask<Ctx>`] with:
//! - `init(ctx)` - one-time async initialization
//! - `handle_once(ctx)` - handle one event
//!
//! The context (`ServiceCtx`) provides access to shared runtime resources
//! (shadow storage, timer) without storing references in the task struct.
//! This avoids self-referential struct issues when Runtime owns both
//! the resources and the tasks.
//!
//! ## Executor Integration
//!
//! The executor layer (embassy, RTIC) owns the signals and calls these methods:
//! - Embassy: `init(ctx)` then `loop { signal.wait(); handle_once(ctx); }`
//! - RTIC: `init(ctx)` once, then hardware triggers call `handle_once(ctx)`

/// Async service task trait.
///
/// Implementors handle initialization and per-event processing.
/// The executor layer handles signal waiting and looping.
///
/// # Generic Parameter
///
/// - `Ctx`: Context type providing access to shared resources (e.g., `ServiceCtx`)
///
/// # Error Handling
///
/// Services handle errors internally by:
/// - Writing status to telemetry registers
/// - Logging via defmt (when available)
/// - Continuing operation (resilient design)
///
/// Methods return `()` rather than `Result` because there's no meaningful
/// caller action for service errors.
pub trait ServiceTask<Ctx> {
    /// One-time initialization.
    ///
    /// Called once at startup before any `handle_once()` calls.
    /// Use for restoring state from flash, establishing connections, etc.
    async fn init(&mut self, ctx: &Ctx);

    /// Handle one event.
    ///
    /// Called by executor after the service's signal/trigger fires.
    /// Should handle the event and return promptly.
    async fn handle_once(&mut self, ctx: &Ctx);
}

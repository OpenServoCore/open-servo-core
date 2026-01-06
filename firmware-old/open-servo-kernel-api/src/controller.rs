//! Controller contract (pure control law).
//!
//! This trait is intentionally **not** tied to any actuator concept (current vs torque vs voltage).
//! Controllers produce a **normalized command** (by default [`Effort`]).
//!
//! - A controller is **pure algorithm**: given `(dt, cfg, setpoint, feedback)` → output.
//! - How that output is turned into real actuation (PWM/timer compare, current limiting,
//!   torque budget, power limiting, etc.) belongs to an **output stage / limiter / compliance**
//!   layer in the kernel, not here.
//!
//! In other words:
//! - `Controller` answers: “how hard should I push?”
//! - An output stage answers: “what does ‘push’ mean on this board/mode?”
//!
//! ## Cascaded loops
//! This trait represents **one loop** (one update rate). Cascaded controllers are built
//! by *composition* (outer loop produces inner setpoint, inner loop produces effort).
//!
//! ## Example 1: Single-loop controller + Node wrapper
//! A controller alone is not scheduled. To schedule it, wrap it in a `Node<M>` and call
//! `controller.update(dt, ...)` from `tick()`.
//!
//! ```rust,ignore
//! use open_servo_kernel_api::{
//!     Controller, Node, CtlSlow, TickDomain, TickCtx,
//!     FaultSink, TelemetrySink,
//!     units::{MicroSecond, Effort, CentiDeg32},
//! };
//! use open_servo_kernel_api::role::{HasRole, Role};
//!
//! // --- Controller algorithm ---
//! pub struct PosPid {
//!     integ: i32,
//! }
//!
//! pub struct PosPidCfg {
//!     pub kp: i32,
//!     pub ki: i32,
//!     pub out_limit: i16, // saturate to +/- out_limit in Effort raw units
//! }
//!
//! impl Controller for PosPid {
//!     type Config = PosPidCfg;
//!     type Setpoint = CentiDeg32;
//!     type Feedback = CentiDeg32;
//!     type Output = Effort; // default would also be Effort
//!
//!     fn reset(&mut self) {
//!         self.integ = 0;
//!     }
//!
//!     fn update(
//!         &mut self,
//!         dt: MicroSecond,
//!         cfg: &Self::Config,
//!         sp: &Self::Setpoint,
//!         fb: &Self::Feedback,
//!     ) -> Self::Output {
//!         let err = sp.as_cdeg() - fb.as_cdeg();
//!         self.integ = self.integ.saturating_add(err.saturating_mul(dt.0 as i32));
//!         let p = err.saturating_mul(cfg.kp);
//!         let i = self.integ.saturating_mul(cfg.ki);
//!         let out = (p + i).clamp(-(cfg.out_limit as i32), cfg.out_limit as i32) as i16;
//!         Effort::from_raw(out)
//!     }
//! }
//!
//! // --- Node wrapper that schedules the controller ---
//! pub struct PosPidNode {
//!     pub ctl: PosPid,
//!     pub cfg: PosPidCfg,
//!     sp: CentiDeg32,
//!     fb: CentiDeg32,
//! }
//!
//! impl HasRole for PosPidNode {
//!     const ROLE: Role = Role::Control;
//! }
//!
//! impl Node<CtlSlow> for PosPidNode {
//!     type In = (CentiDeg32, CentiDeg32); // (setpoint, feedback)
//!     type Out = Effort;
//!
//!     fn update(&mut self, input: Self::In) {
//!         (self.sp, self.fb) = input;
//!     }
//!
//!     fn tick<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> Self::Out
//!     where
//!         F: FaultSink + ?Sized,
//!         T: TelemetrySink + ?Sized,
//!     {
//!         ctx.debug_assert_domain(TickDomain::ControlSlow);
//!         self.ctl.update(ctx.tick.dt, &self.cfg, &self.sp, &self.fb)
//!     }
//! }
//! ```
//!
//! ## Example 2: Cascaded controller by composition
//! Outer loop runs slower (position → velocity setpoint), inner loop runs faster
//! (velocity → effort). Each loop is still a `Controller`; scheduling is done by
//! two `Node<M>` wrappers at different domains.
//!
//! ```rust,ignore
//! use open_servo_kernel_api::{
//!     Controller, Node, CtlSlow, CtlFast, TickDomain, TickCtx,
//!     FaultSink, TelemetrySink,
//!     units::{MicroSecond, Effort, CentiDeg32, DegPerSec10},
//! };
//! use open_servo_kernel_api::role::{HasRole, Role};
//!
//! // Outer: position -> velocity setpoint
//! pub struct PosToVel;
//! pub struct PosToVelCfg { pub kp: i32, pub vel_limit_dps10: i16 }
//!
//! impl Controller for PosToVel {
//!     type Config = PosToVelCfg;
//!     type Setpoint = CentiDeg32;
//!     type Feedback = CentiDeg32;
//!     type Output = DegPerSec10;
//!
//!     fn reset(&mut self) {}
//!
//!     fn update(&mut self, _dt: MicroSecond, cfg: &Self::Config, sp: &CentiDeg32, fb: &CentiDeg32) -> DegPerSec10 {
//!         let err = sp.as_cdeg() - fb.as_cdeg();
//!         let vel = (err.saturating_mul(cfg.kp)).clamp(-(cfg.vel_limit_dps10 as i32), cfg.vel_limit_dps10 as i32) as i16;
//!         DegPerSec10::from_dps10(vel)
//!     }
//! }
//!
//! // Inner: velocity -> effort
//! pub struct VelPid { integ: i32 }
//! pub struct VelPidCfg { pub kp: i32, pub ki: i32, pub out_limit: i16 }
//!
//! impl Controller for VelPid {
//!     type Config = VelPidCfg;
//!     type Setpoint = DegPerSec10;
//!     type Feedback = DegPerSec10;
//!     type Output = Effort;
//!
//!     fn reset(&mut self) { self.integ = 0; }
//!
//!     fn update(&mut self, dt: MicroSecond, cfg: &Self::Config, sp: &DegPerSec10, fb: &DegPerSec10) -> Effort {
//!         let err = sp.as_dps10() as i32 - fb.as_dps10() as i32;
//!         self.integ = self.integ.saturating_add(err.saturating_mul(dt.0 as i32));
//!         let out = (err.saturating_mul(cfg.kp) + self.integ.saturating_mul(cfg.ki))
//!             .clamp(-(cfg.out_limit as i32), cfg.out_limit as i32) as i16;
//!         Effort::from_raw(out)
//!     }
//! }
//!
//! // Shared wire between loops (a tiny "mailbox" / state variable).
//! // In real kernel code this might live in state manager.
//! #[derive(Copy, Clone)]
//! pub struct VelSp(pub DegPerSec10);
//!
//! pub struct OuterNode {
//!     pub ctl: PosToVel,
//!     pub cfg: PosToVelCfg,
//!     sp: CentiDeg32,
//!     fb: CentiDeg32,
//!     pub out: VelSp,
//! }
//! impl HasRole for OuterNode { const ROLE: Role = Role::Control; }
//!
//! impl Node<CtlSlow> for OuterNode {
//!     type In = (CentiDeg32, CentiDeg32);
//!     type Out = ();
//!     fn update(&mut self, input: Self::In) { (self.sp, self.fb) = input; }
//!     fn tick<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> ()
//!     where F: FaultSink + ?Sized, T: TelemetrySink + ?Sized {
//!         ctx.debug_assert_domain(TickDomain::ControlSlow);
//!         let vel_sp = self.ctl.update(ctx.tick.dt, &self.cfg, &self.sp, &self.fb);
//!         self.out = VelSp(vel_sp);
//!     }
//! }
//!
//! pub struct InnerNode {
//!     pub ctl: VelPid,
//!     pub cfg: VelPidCfg,
//!     sp: DegPerSec10,
//!     fb: DegPerSec10,
//! }
//! impl HasRole for InnerNode { const ROLE: Role = Role::Control; }
//!
//! impl Node<CtlFast> for InnerNode {
//!     // caller supplies (vel_setpoint, vel_feedback) each tick/update
//!     type In = (DegPerSec10, DegPerSec10);
//!     type Out = Effort;
//!     fn update(&mut self, input: Self::In) { (self.sp, self.fb) = input; }
//!     fn tick<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> Effort
//!     where F: FaultSink + ?Sized, T: TelemetrySink + ?Sized {
//!         ctx.debug_assert_domain(TickDomain::ControlFast);
//!         self.ctl.update(ctx.tick.dt, &self.cfg, &self.sp, &self.fb)
//!     }
//! }
//! ```

use crate::units::MicroSecond;

/// Pure control-law contract.
///
/// - Default output is [`Effort`] (normalized command).
/// - If you want a controller to output something else (e.g. velocity setpoint in cascaded loops),
///   override `type Output = ...`.
pub trait Controller {
    type Config;
    type Setpoint;
    type Feedback;

    /// Output of the control law.
    ///
    /// Usually normalized [`Effort`]. Override for cascaded loops (e.g. position→velocity).
    type Output;

    /// Reset internal state (integrators, filters, observers inside the controller).
    fn reset(&mut self);

    /// Advance the controller by `dt` and compute the next output.
    ///
    /// `dt` must be provided by the scheduler/tick pipeline, not inferred.
    fn update(
        &mut self,
        dt: MicroSecond,
        cfg: &Self::Config,
        sp: &Self::Setpoint,
        fb: &Self::Feedback,
    ) -> Self::Output;
}

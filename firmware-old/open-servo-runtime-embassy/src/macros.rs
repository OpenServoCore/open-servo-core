//! Macros for spawning services.
//!
//! Provides [`spawn_all_services!`] to generate and spawn all service tasks.

/// Spawn all configured services from an EmbassyRuntime.
///
/// This macro generates embassy task functions and spawns them.
/// It must be invoked from firmware where the concrete board type is known.
///
/// # Usage
///
/// ```rust,ignore
/// use open_servo_runtime_embassy::{spawn_all_services, EmbassyRuntime, run};
///
/// fn main() -> ! {
///     // ... hardware init ...
///     let embassy_rt = init_embassy_runtime(runtime);
///
///     run(|spawner| {
///         spawn_all_services!(MyBoard, get_embassy_runtime(), spawner);
///     })
/// }
/// ```
///
/// # Arguments
///
/// - `$board_ty`: The concrete board type (e.g., `Stm32f301Board`)
/// - `$embassy_rt`: Expression returning `&'static EmbassyRuntime<$board_ty>`
/// - `$spawner`: The embassy spawner
///
/// # Generated Tasks
///
/// - `persist_task`: Runs the persist service forever (always enabled)
/// - `rpc_task`: Runs the RPC service forever (requires `osctl` feature)
#[macro_export]
macro_rules! spawn_all_services {
    ($board_ty:ty, $embassy_rt:expr, $spawner:expr) => {{
        // Get the runtime reference
        let rt: &'static $crate::EmbassyRuntime<$board_ty> = $embassy_rt;

        // Persist task (always enabled)
        {
            #[embassy_executor::task]
            async fn persist_task(rt: &'static $crate::EmbassyRuntime<$board_ty>) {
                // Wait for RTT to be initialized before accessing flash
                embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
                rt.persist_forever().await
            }
            $spawner.must_spawn(persist_task(rt));
        }

        // RPC task (osctl feature only)
        #[cfg(feature = "osctl")]
        {
            #[embassy_executor::task]
            async fn rpc_task(rt: &'static $crate::EmbassyRuntime<$board_ty>) {
                rt.rpc_forever().await
            }
            $spawner.must_spawn(rpc_task(rt));
        }
    }};
}

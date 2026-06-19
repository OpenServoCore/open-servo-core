// rstest_reuse 0.7 requires the crate to be brought into scope at the
// test binary root for `#[apply]` macro hygiene — clippy reads this as a
// redundant single-component import, but it isn't.
#[allow(clippy::single_component_path_imports, unused_imports)]
use rstest_reuse;

#[path = "../support.rs"]
mod support;

mod bulk_read;
mod bulk_write;
mod fast_bulk_read;
mod fast_sync_read;
mod ping;
mod read;
mod reboot;
mod reg_write_action;
mod sync_read;
mod sync_write;
mod write;

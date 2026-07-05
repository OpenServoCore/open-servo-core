// rstest_reuse 0.7 requires the crate to be brought into scope at the
// test binary root for `#[apply]` macro hygiene — clippy reads this as a
// redundant single-component import, but it isn't.
#[allow(clippy::single_component_path_imports, unused_imports)]
use rstest_reuse;

#[path = "../support.rs"]
mod support;

mod hsi_trim;
mod long_reply_delays;
mod ping_broadcast;
mod rdt;
mod slot_timing;

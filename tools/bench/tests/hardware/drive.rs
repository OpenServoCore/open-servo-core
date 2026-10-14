use serial_test::serial;

use crate::support::bench;

/// The drive check every closed-loop test runs first, on its own: a quick
/// screen after rewiring the motor, before the suite drives anything.
#[serial]
#[test]
fn duty_moves_the_position_its_own_way() {
    bench().require_forward_drive();
}

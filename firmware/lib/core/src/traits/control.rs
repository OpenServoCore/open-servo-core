use control_table::Enum;

use crate::Shared;

/// What the kernel control loop reads from the board hardware.
pub trait Sensors {
    /// Acquisition only: complete the scan's frame in device counts in the
    /// table's tick block (`Shared::frame`), at the tick's entry.
    fn land(&mut self, shared: &Shared);
}

/// What the kernel control loop writes to the motor driver.
pub trait Motor {
    fn write(&mut self, cmd: MotorCmd);
}

/// Bag of chip-side capabilities the control loop needs. Splits into
/// disjoint sub-trait borrows so the kernel can hold both at once.
pub trait ControlIo {
    type Sensors: Sensors;
    type Motor: Motor;

    /// Servo-wide capability flags (e.g., HAS_MOTOR_ENCODER).
    fn caps(&self) -> Capabilities {
        Capabilities::default()
    }
    fn parts(&mut self) -> (&mut Self::Sensors, &mut Self::Motor);
}

/// The kernel's per-fast-tick stream hook; core never names the transport.
pub trait TelStream {
    /// The tick block is published: the row's first chunk may be taken.
    /// `hold_start`, asked only when a burst would start: true on the
    /// kernel's configuration take phase and while a take is pending, when
    /// the burst waits a tick, so no tick carries both a take and a row.
    fn on_publish(&mut self, hold_start: impl FnOnce() -> bool);
    /// The tick runs a SLOW stage: the TX lane takes no step
    /// on it. Any time before [`Self::on_tick_end`].
    fn hold(&mut self) {}
    /// The tick's last work, after the drive is written: the row's second
    /// chunk, then the TX lane.
    fn on_tick_end(&mut self) {}
    /// A burst streams: the host is silent by design, so the experiment
    /// image's bus dead-man holds (kernel sec 2.7).
    fn streaming(&self) -> bool {
        false
    }
}

/// Stream absent (boards without a burst producer, tests).
impl TelStream for () {
    fn on_publish(&mut self, _: impl FnOnce() -> bool) {}
}

#[derive(Copy, Clone, Debug)]
pub enum MotorCmd {
    /// `drv_en` LOW. Driver powered down; outputs hi-Z. Boot/fault state.
    Disabled,
    /// `drv_en` HIGH, both half-bridges hi-Z. Motor free-wheels.
    Coast,
    /// `drv_en` HIGH, both low-side FETs on. Short-circuit braking.
    Brake,
    /// `drv_en` HIGH, PWM drive: `ticks` of the period on (`window::drive_ticks`),
    /// `reverse` in the wiring's frame, selectable off-window decay. Zero
    /// ticks coast.
    Drive {
        ticks: u16,
        reverse: bool,
        decay: DecayMode,
    },
}

/// Also CONFIG `openloop_decay`, OpenLoop's off-window decay (closed-loop
/// modes force Slow): `repr(u8)`, and a discriminant outside `ALLOWED` is
/// UB, so table writes gate on it.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum DecayMode {
    /// Off-window = BRAKE (idle leg HIGH). DRV8212P-correct for usable torque.
    #[default]
    Slow = 0,
    /// Off-window = COAST (idle leg LOW). Low EMI, near-zero avg current on DRV8212P.
    Fast = 1,
}

bitflags::bitflags! {
    /// Mirrors `capability_flags` in `ConfigCommon`. Protocol-relevant only.
    #[derive(Copy, Clone, Debug, Default)]
    pub struct Capabilities: u32 {
        const HAS_MOTOR_ENCODER = 1 << 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn motor_cmd_fits_one_register() {
        assert_eq!(size_of::<MotorCmd>(), size_of::<u32>());
    }

    #[test]
    fn capabilities_compose() {
        let none = Capabilities::default();
        assert!(none.is_empty());
        let with_enc = Capabilities::HAS_MOTOR_ENCODER;
        assert!(with_enc.contains(Capabilities::HAS_MOTOR_ENCODER));
        assert_eq!(with_enc.bits(), 1);
    }
}

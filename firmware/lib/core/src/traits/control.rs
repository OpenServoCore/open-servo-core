use crate::Shared;

/// What the kernel control loop reads from the board hardware.
pub trait Sensors {
    /// Acquisition only, at the tick's entry: the frame's fields the motor
    /// write reads (the shunt) in device counts in the table's tick block
    /// (`Shared::frame`).
    fn land(&mut self, shared: &Shared);
    /// After the motor write: the rest of the frame, once the scan that
    /// holds it has landed. False when it never did: the frame is not this
    /// period's, a sensor fault. `read`: the window this frame sampled
    /// gave a current reading, so the command before this tick's was a
    /// live drive and a flip written now makes the next window the first
    /// after a reversal, which nothing reads (kernel sec 2.3).
    fn complete(&mut self, _shared: &Shared, _read: bool) -> bool {
        true
    }
}

/// What the kernel control loop writes to the motor driver.
pub trait Motor {
    fn write(&mut self, cmd: MotorCmd);
    /// Right after a write: it landed too late to load ahead of the pulse
    /// it is centered on, so the window that pulse drives holds two
    /// commands.
    fn late(&self) -> bool {
        false
    }
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
    /// A burst streams: the host is silent by design, so the bus dead-man
    /// holds (kernel sec 2.7).
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
    /// `reverse` in the wiring's frame, the off-window a brake (slow decay,
    /// idle leg HIGH). Zero ticks coast.
    Drive { ticks: u16, reverse: bool },
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

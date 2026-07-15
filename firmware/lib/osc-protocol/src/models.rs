//! Model-number registry (protocol sec 5.4). A model number is class-structured:
//! the high byte is the device class, the low byte the model within that class.
//! It keys a table-ABI family; 0x0000 is reserved unassigned - what unseeded
//! or dev firmware reports. Low byte 0x00 is likewise unassigned within a class,
//! mirroring the global 0x0000 rule.

/// Basic-tier servo class.
pub const CLASS_SERVO: u8 = 0x01;

/// Compose a model number from its class and per-class model bytes.
pub const fn model_number(class: u8, model: u8) -> u16 {
    ((class as u16) << 8) | model as u16
}

/// Extract the device-class byte from a model number.
pub const fn model_class(n: u16) -> u8 {
    (n >> 8) as u8
}

/// Registry-adjacent display name for a class byte; "unknown" off-registry.
pub const fn class_name(class: u8) -> &'static str {
    match class {
        CLASS_SERVO => "servo",
        _ => "unknown",
    }
}

/// Reserved unassigned; unseeded or dev firmware reports it.
pub const MODEL_UNASSIGNED: u16 = 0;
/// The osc-servo-core table ABI, shared by every board flashing this firmware
/// family (osc-dev-v006 today, sg90/mg90s swap boards later); a board takes its
/// own number only if it diverges the table.
pub const MODEL_OSC_SERVO: u16 = model_number(CLASS_SERVO, 0x01);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn class_model_round_trips() {
        let n = model_number(0x02, 0x37);
        assert_eq!(model_class(n), 0x02);
        assert_eq!(n & 0xFF, 0x37);
    }

    #[test]
    fn class_name_maps_the_registry() {
        assert_eq!(class_name(CLASS_SERVO), "servo");
        assert_eq!(class_name(0xFF), "unknown");
    }

    #[test]
    fn osc_servo_number() {
        assert_eq!(MODEL_OSC_SERVO, 0x0101);
        assert_eq!(model_class(MODEL_OSC_SERVO), CLASS_SERVO);
    }
}

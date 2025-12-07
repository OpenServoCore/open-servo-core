#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartPort {
    Bus,   // Servo bus (Dynamixel protocol)
    Debug, // Debug shell
}

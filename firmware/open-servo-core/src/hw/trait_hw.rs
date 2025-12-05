#[derive(Debug, Clone, Copy)]
pub enum UartPort {
    Bus,
    Debug,
}

pub trait Hw {
    /// Current through the motor in milliamps (mA)
    fn phase_current(&self) -> u16;

    /// Raw ADC position value (0-4095 for 12-bit ADC)
    fn position(&self) -> u16;

    /// Supply/bus voltage in millivolts (mV)
    fn bus_voltage(&self) -> u16;

    /// Chip temperature in decikelvin (dK = K * 10) if available
    /// e.g., 2982 = 298.2K = 25°C
    fn temperature(&self) -> Option<u16>;

    /// Motor temperature in decikelvin (dK = K * 10) if available
    /// e.g., 3232 = 323.2K = 50°C
    fn motor_temperature(&self) -> Option<u16>;

    /// Set PWM duty cycle (-PWM_MAX to +PWM_MAX)
    /// Positive = forward, Negative = reverse
    fn set_pwm(&mut self, duty: i32);

    /// Enable/disable motor driver
    fn set_enable(&mut self, en: bool);

    /// Get current time in microseconds
    fn now_us(&self) -> u32;

    /// Write bytes to UART
    fn uart_write(&mut self, port: UartPort, buf: &[u8]);

    /// Read a byte from UART (non-blocking)
    fn uart_read_byte(&mut self, port: UartPort) -> Option<u8>;
}
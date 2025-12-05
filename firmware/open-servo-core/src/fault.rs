use open_servo_hw::MotorDriver;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultKind {
    OverCurrent,
    OverTemp,
    UnderVoltage,
    QueuePressure,
    EncoderFault,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultState {
    Ok,
    Latched(FaultKind),
}

impl FaultState {
    pub fn new() -> Self {
        FaultState::Ok
    }

    pub fn is_faulted(&self) -> bool {
        !matches!(self, FaultState::Ok)
    }

    pub fn raise(&mut self, kind: FaultKind) {
        if matches!(self, FaultState::Ok) {
            *self = FaultState::Latched(kind);
            #[cfg(feature = "defmt")]
            defmt::error!("Fault raised: {:?}", kind);
        }
    }

    pub fn clear(&mut self) {
        if self.is_faulted() {
            #[cfg(feature = "defmt")]
            defmt::info!("Fault cleared");
            *self = FaultState::Ok;
        }
    }

    /// Safety-critical fault response - disable motor immediately
    pub fn apply_safety<H: MotorDriver>(&self, hw: &mut H) {
        if self.is_faulted() {
            hw.set_pwm(0);
            hw.set_enable(false);
        }
    }
}

impl Default for FaultState {
    fn default() -> Self {
        Self::new()
    }
}

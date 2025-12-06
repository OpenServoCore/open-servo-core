/// Type-safe ADC sample structure
/// Maps the 5-channel DMA buffer to named fields
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct AdcSample {
    /// Internal reference voltage (channel 18)
    pub vrefint_raw: u16,
    
    /// Position potentiometer on PA0 (channel 1)
    pub position_raw: u16,
    
    /// Current sense on PA1 (channel 2)
    pub current_raw: u16,
    
    /// Setpoint potentiometer on PA2 (channel 3)
    pub setpoint_raw: u16,
    
    /// Internal temperature sensor (channel 16)
    pub temperature_raw: u16,
}

impl AdcSample {
    /// Create from raw DMA buffer
    pub fn from_dma_buffer(buffer: [u16; 5]) -> Self {
        AdcSample {
            vrefint_raw: buffer[0],
            position_raw: buffer[1],
            current_raw: buffer[2],
            setpoint_raw: buffer[3],
            temperature_raw: buffer[4],
        }
    }
    
    /// Convert to array (for compatibility)
    pub fn as_array(&self) -> [u16; 5] {
        [
            self.vrefint_raw,
            self.position_raw,
            self.current_raw,
            self.setpoint_raw,
            self.temperature_raw,
        ]
    }
}
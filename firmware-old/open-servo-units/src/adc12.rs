//! Raw 12-bit ADC value.

/// Raw 12-bit ADC value
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Adc12(pub u16);

impl Adc12 {
    #[inline]
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw & 0x0FFF) // Ensure 12-bit
    }

    #[inline]
    pub const fn as_raw(self) -> u16 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_masks_to_12bit() {
        assert_eq!(Adc12::from_raw(0xFFFF).as_raw(), 0x0FFF);
        assert_eq!(Adc12::from_raw(0x1234).as_raw(), 0x0234);
    }
}

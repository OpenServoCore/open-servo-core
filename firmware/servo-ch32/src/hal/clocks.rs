use ch32_metapac::rcc::vals::{Adcpre, Hpre};

pub const HSI_HZ: u32 = 24_000_000;
/// V006 RM sec 3, RCC_CTLR: each HSITRIM step shifts HSI by ~60 kHz.
pub const HSI_TRIM_STEP_HZ: u32 = 60_000;
pub const PLL_MUL: u32 = 2;
pub const SYSCLK_HZ: u32 = HSI_HZ * PLL_MUL;

pub const HPRE_DIV: u32 = 1;

pub const HCLK_HZ: u32 = SYSCLK_HZ / HPRE_DIV;
pub const PCLK_HZ: u32 = HCLK_HZ;
pub const TIM_CLK_HZ: u32 = HCLK_HZ;
/// ADC_CLK_MODE = 0 takes the divided HB clock, ADC_CLK_ADJ stays at its
/// 1/2-duty reset. fADC is rated 16-48 MHz (V006 datasheet Table 3-23) but
/// the top of that range is unusable with the low-power comparator buffer
/// (ADC_LP = 1, RM sec 9.3.14): at 48 MHz a bit trial gets 20.8 ns and the
/// buffer does not settle, so codes go missing. Measured on a pot sweep,
/// 48 MHz reports 63% of the codes the wiper crosses and dumps the range it
/// loses into a neighbour, leaving code 1024 seventy times too wide and the
/// 55 codes under it unreachable; 24 MHz reports 96% and code 1024 clean.
/// Acquisition length is not the lever - 41.5 cycles at 48 MHz measured the
/// same 63% as 3.5.
pub const ADC_PRE_DIV: u32 = 2;
pub const ADCCLK_HZ: u32 = HCLK_HZ / ADC_PRE_DIV;
pub const SYSTICK_TICKS_PER_US: u32 = HCLK_HZ / 1_000_000;
pub const SYSTICK_TICKS_PER_MS: u32 = HCLK_HZ / 1_000;

pub const fn adcpre_val() -> Adcpre {
    match ADC_PRE_DIV {
        2 => Adcpre::DIV2,
        4 => Adcpre::DIV4,
        6 => Adcpre::DIV6,
        8 => Adcpre::DIV8,
        12 => Adcpre::DIV12,
        16 => Adcpre::DIV16,
        _ => panic!("unsupported ADC_PRE_DIV"),
    }
}

pub const fn hpre_val() -> Hpre {
    match HPRE_DIV {
        1 => Hpre::DIV1,
        2 => Hpre::DIV2,
        3 => Hpre::DIV3,
        4 => Hpre::DIV4,
        5 => Hpre::DIV5,
        6 => Hpre::DIV6,
        7 => Hpre::DIV7,
        8 => Hpre::DIV8,
        16 => Hpre::DIV16,
        32 => Hpre::DIV32,
        64 => Hpre::DIV64,
        128 => Hpre::DIV128,
        256 => Hpre::DIV256,
        _ => panic!("unsupported HPRE_DIV"),
    }
}

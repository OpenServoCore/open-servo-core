use ch32_metapac::rcc::vals::Hpre;

pub const HSI_HZ: u32 = 24_000_000;
/// V006 RM sec 3, RCC_CTLR: each HSITRIM step shifts HSI by ~60 kHz.
pub const HSI_TRIM_STEP_HZ: u32 = 60_000;
pub const PLL_MUL: u32 = 2;
pub const SYSCLK_HZ: u32 = HSI_HZ * PLL_MUL;

pub const HPRE_DIV: u32 = 1;

pub const HCLK_HZ: u32 = SYSCLK_HZ / HPRE_DIV;
pub const PCLK_HZ: u32 = HCLK_HZ;
pub const TIM_CLK_HZ: u32 = HCLK_HZ;
/// ADC_CLK_MODE = 1 feeds the converter the undivided HB clock (ADCPRE is
/// ignored). fADC is rated 16-48 MHz (V006 datasheet Table 3-23); the
/// sampling-rate ceiling is the source impedance (RAIN table), set per
/// channel through its aperture, not the clock.
pub const ADCCLK_HZ: u32 = HCLK_HZ;
pub const SYSTICK_TICKS_PER_US: u32 = HCLK_HZ / 1_000_000;
pub const SYSTICK_TICKS_PER_MS: u32 = HCLK_HZ / 1_000;

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

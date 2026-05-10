use ch32_metapac::SYSTICK;
use ch32_metapac::systick::vals::Stclk;

pub fn init() {
    SYSTICK.cmp().write_value(u32::MAX);
    SYSTICK.cnt().write_value(0);
    SYSTICK.ctlr().write(|w| {
        w.set_ste(true);
        w.set_stclk(Stclk::HCLK);
    });
}

#[inline(always)]
pub fn ticks() -> u32 {
    SYSTICK.cnt().read()
}

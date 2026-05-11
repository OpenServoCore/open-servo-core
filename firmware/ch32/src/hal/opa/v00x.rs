use ch32_metapac::OPA;

const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Gain {
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
    X32 = 0b110,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum PosInput {
    PA2 = 0b00,
    PD7 = 0b01,
    PD3 = 0b10,
    PD1 = 0b11,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Bias {
    MidRail = 0,
    QuarterRail = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Output {
    PD4 = 0b00,
    PA5 = 0b01,
    /// CMP2 input + ADC channel 9, no pin.
    Internal = 0b11,
}

fn unlock() {
    OPA.opa_key().write(|w| w.set_opa_key(KEY1));
    OPA.opa_key().write(|w| w.set_opa_key(KEY2));
}

/// Caller must wait for the OPA to settle before sampling. Bring-up used
/// 500 ms; actual minimum unknown.
pub fn init_pga_single_ended(input: PosInput, gain: Gain, bias: Bias, output: Output) {
    unlock();
    OPA.ctlr1().write(|w| {
        w.set_opa_hs1(true);
        w.set_vbcmpsel(0b11);
        w.set_vbsel(matches!(bias, Bias::QuarterRail));
        w.set_vben(true);
        w.set_pgadif(false);
        w.set_fb_en1(true);
        w.set_nsel1(gain as u8);
        w.set_psel1(input as u8);
        w.set_mode1(output as u8);
        w.set_opa_en1(true);
    });
}

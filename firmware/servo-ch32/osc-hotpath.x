OVERWRITE_SECTIONS
{
    .text : ALIGN(4)
    {
        . = ALIGN(4);
        KEEP(*(SORT_NONE(.handle_reset)))
        *(.text.TIM2 .text.__qingke_rt_TIM2)
        *(.text.USART1 .text.__qingke_rt_USART1)
        *(.text.SysTick .text.__qingke_rt_SysTick)
        *(.text.*17osc_servo_drivers3bus*)
        *(.text.*14osc_servo_ch327runtime3isr*)
        *(.text.*14osc_servo_ch329providers*)
        *(.text.*12osc_protocol*)
        *(.text.*13control_table*)
        *(.text.*14osc_servo_core8services*)
        *(.text.*14osc_servo_core6shared*)
        *(.text.*14osc_servo_core10data_state*)
        *(.text.*14osc_servo_core7pos_lut*)
        *(.text.*14osc_servo_core5stamp*)
        *(.text.*14osc_servo_core7persist*)
        *(.text.memcpy .text.memset .text.memcmp .text.*17compiler_builtins3mem*)
        *(.text.DMA1_CHANNEL1 .text.__qingke_rt_DMA1_CHANNEL1)
        *(.text.*14osc_servo_ch327control*)
        *(.text.*14osc_servo_core6kernel*)
        *(.text.*14osc_servo_core4math*)
        *(.text.*17osc_servo_drivers3tel*)
        *(.text.*14osc_servo_core9estimator*)
        *(.init.rust)
        *(.text .text.*)
        *(.highcode .highcode.*);
    } >FLASH AT>FLASH
}

use control_table::{Block, Region};

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryConverted {
    #[ct_field(access = ro)]
    pub present_position: i32,
    #[ct_field(access = ro)]
    pub present_velocity: i32,
    #[ct_field(access = ro)]
    pub present_current: i16,
    #[ct_field(access = ro)]
    pub present_temp: i16,
    #[ct_field(access = ro)]
    pub present_vbus_mv: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryIntermediaries {
    #[ct_field(access = ro)]
    pub vbus_filt_mv: u16,
    #[ct_field(access = ro)]
    pub t_winding_dc: i16,
    #[ct_field(access = ro)]
    pub pwm_duty_actual: i16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
    #[ct_field(access = ro)]
    pub pid_error_last: i32,
    #[ct_field(access = ro)]
    pub pid_output_last: i32,
    #[ct_field(access = ro)]
    pub internal_goal: i32,
    #[ct_field(access = ro)]
    pub sample_tick: u32,
}

/// `fault_flags` writable-RO carve-out: host writing 0x00 clears non-latched bits.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryFault {
    #[ct_field(access = ro)]
    pub mode_active: u8,
    #[ct_field(access = ro)]
    pub fault_flags: u8,
    #[ct_field(access = ro)]
    pub fault_code: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryRaw {
    #[ct_field(access = ro)]
    pub raw_pos: u16,
    #[ct_field(access = ro)]
    pub raw_current: u16,
    #[ct_field(access = ro)]
    pub raw_temp: u16,
    #[ct_field(access = ro)]
    pub raw_vbus: u16,
    #[ct_field(access = ro)]
    pub raw_vmotor_a: u16,
    #[ct_field(access = ro)]
    pub raw_vmotor_b: u16,
    #[ct_field(access = ro)]
    pub raw_enc_a: u16,
    #[ct_field(access = ro)]
    pub raw_enc_b: u16,
}

/// Host can Write zero to any field to clear it (bench instrumentation).
/// Chip-side `report_fault` increments via raw pointer, bypassing the regmap.
/// Concurrent host clear + ISR increment may drop one update — acceptable for bench.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryDxlLink {
    #[ct_field(access = rw)]
    pub illegal_transition: u32,
    #[ct_field(access = rw)]
    pub unexpected_byte_count: u32,
    #[ct_field(access = rw)]
    pub previous_slot_timeout: u32,
    #[ct_field(access = rw)]
    pub slot_timing_miss: u32,
    #[ct_field(access = rw)]
    pub crc_patch_deadline_miss: u32,
    #[ct_field(access = rw)]
    pub edge_anchor_miss: u32,
    #[ct_field(access = rw)]
    pub dma_overrun: u32,
    #[ct_field(access = rw)]
    pub parity_error: u32,
    #[ct_field(access = rw)]
    pub framing_error: u32,
    #[ct_field(access = rw)]
    pub noise_error: u32,
}

/// Bench tuning stamps. Always zero in non-tuning chip builds; chip-side
/// stamps live under `osc-ch32`'s `tuning` feature. The block is
/// unconditional so host tooling can probe the addresses without
/// depending on chip-build features. Host can write zero to clear,
/// matching the `TelemetryDxlLink` carve-out pattern. Zero in any
/// _min field doubles as the "no sample yet" sentinel — the chip
/// treats `cur == 0` as uninitialised and writes any first delta.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryDxlTune {
    /// Min observed `(TIM2_CNT − CCR3)` at `on_tim2_cc3` IRQ entry, in
    /// TIM2 ticks. CCR3 back-dates the deadline by
    /// `TX_START_ENTRY_TICKS`, so the wire-bit lands at
    /// `deadline + (L_entry − K)`. To guarantee `wire ≥ deadline` the
    /// const must be sized to the *minimum* observed entry latency —
    /// anything above min just widens the slot (jitter-safe).
    #[ct_field(access = rw)]
    pub tx_start_entry_min: u16,
    /// Min observed `(SysTick.CNT − SysTick.CMP)` at `on_systick` entry,
    /// in HCLK ticks. Same back-date logic as `tx_start_entry_min`;
    /// the const must be sized to the minimum to keep the fold-start
    /// at-or-after the grid anchor.
    #[ct_field(access = rw)]
    pub fast_last_entry_min: u16,
    /// Max observed `(CCR3 − TIM2_CNT)` at `arm_tim2` post-CCR3-write
    /// (legitimate-window only — wrap-guard misses excluded), in TIM2
    /// ticks. Upper bound for `SCHEDULE_WRAP_GUARD_TICKS`.
    #[ct_field(access = rw)]
    pub schedule_remaining_max: u16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Region)]
#[ct_region(addr = crate::regions::TELEMETRY_BASE_ADDR, size = crate::regions::TELEMETRY_REGION_SIZE)]
pub struct TelemetryRegs {
    pub converted: TelemetryConverted,
    pub intermediaries: TelemetryIntermediaries,
    pub fault: TelemetryFault,
    pub raw: TelemetryRaw,
    pub link: TelemetryDxlLink,
    pub tune: TelemetryDxlTune,
}

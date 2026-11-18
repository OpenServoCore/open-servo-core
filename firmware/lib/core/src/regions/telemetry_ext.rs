use control_table::{Block, Section};

/// The terminal taps' floor, the companion of
/// `TelemetryLimits::window_floor_q15`.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryLimitsExt {
    /// Smallest duty whose drive window the terminal taps read
    /// (`window::floor_duty` of `v_window_min_ticks`), Q15: a drive whose
    /// fit needs `va - vb` runs at or above the higher of the two floors.
    /// Published at the kernel's first tick and at every configuration
    /// refresh.
    #[ct_field(access = ro)]
    pub window_v_floor_q15: u16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
}

/// Servo health, written by the chip side (the tick interrupt, the main
/// loop), never by the kernel. The `rw` counters follow the
/// `TelemetryCommon` clear contract: the host writes zero to clear.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryHealth {
    /// Mean share of the kernel period the tick interrupt took over the
    /// last 4000 ticks (0.2 s at 20 kHz), Q15: 32768 = the whole period.
    #[ct_field(access = ro)]
    pub tick_load_mean_q15: u16,
    /// Kernel ticks whose interrupt took longer than one period; nothing
    /// preempts the tick, so every count is the kernel overrunning by
    /// itself. Updated every 20 ticks. Wraps.
    #[ct_field(access = rw)]
    pub tick_over_count: u16,
    /// Kernel ticks that never ran: the previous tick was still running, or
    /// a flash SAVE stalled the core; updated every 20 ticks. Wraps.
    #[ct_field(access = rw)]
    pub tick_lost_count: u16,
    /// TEL stream rows dropped because both stream buffers were waiting for
    /// the wire. Wraps.
    #[ct_field(access = rw)]
    pub tel_drop_count: u16,
    /// Kernel ticks whose motor write landed after the trough, so the
    /// width loaded at the next crest and split that pulse between two
    /// commands; the window it drove read no current. Wraps.
    #[ct_field(access = rw)]
    pub write_late_count: u16,
    /// Smallest free stack seen since boot, bytes.
    #[ct_field(access = ro)]
    pub stack_free_min: u16,
}

/// Winding estimate state beside `t_winding_cc`: the board NTC as the
/// kernel converts it (centi-C, the estimate's base) and the flags
/// (`estimator::thermal::flag`: bit 0 unset, bit 1 NTC fault; bits 2-7
/// reserved 0).
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryTherm {
    #[ct_field(access = ro)]
    pub t_ntc_cc: i16,
    #[ct_field(access = ro)]
    pub therm_flags: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
}

/// The velocity loop's reference as its VELOCITY phase reads it, raw csQ16
/// as `omega_hat_cps`, published at PUBLISH beside the estimates by an
/// image built with `loop-margin`; 0 on the production and experiment
/// images.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryInject {
    #[ct_field(access = ro)]
    pub omega_ref_cps: i32,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::TELEMETRY_EXT_BASE_ADDR,
    size = crate::regions::TELEMETRY_EXT_REGION_SIZE
)]
pub struct TelemetryExtRegs {
    pub health: TelemetryHealth,
    pub limits_ext: TelemetryLimitsExt,
    pub therm: TelemetryTherm,
    pub inject: TelemetryInject,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 40],
}

use osc_units::Effort;

use crate::{Board, DecayMode, MotorCmd, SampleFrame, Shared};

/// Runs in the PWM ISR; one `on_tick` per period.
pub struct Kernel<B: Board> {
    pub board: B,
    pub state: KernelState,
    pub stream_decimation_counter: u8,
    pub stream_duration_remaining_ticks: u32,
}

#[derive(Default)]
pub struct KernelState {
    pub pid_integrator: i32,
    pub pid_prev_error: i32,
    /// Bring-up bang-bang: +1 drives toward +pos, -1 toward -pos, 0 = uninit.
    pub swing_dir: i8,
    /// Ticks left in the bring-up drive window; armed on the first tick.
    /// Reaching 0 latches the motor into Coast for the rest of the session.
    pub drive_ticks_remaining: u32,
}

// TEMP DIAG (2026-05-20): 100 % duty, locked direction, 2 s window. Stall the
// motor into one endstop so V_shunt during the drive phase is dominated by
// stall current (SG90 at 5 V stalls at ~500 mA – 1 A → 5–10 mV across the
// 10 mΩ shunt), well above the OPA's ~1.5 mV input-offset floor. If the
// current-sense chain works at all, post_pk should jump to ≥300 LSB during
// these 2 s and drop back to ~50 LSB after the window closes.
const SWING_LIMIT_URAD: i32 = 700_000;
const SWING_EFFORT: i16 = i16::MAX;
// 2 s at 20 kHz PWM (kernel ticks once per PWM period via DMA-TC).
// Scale to (2000 × pwm_freq_kHz) if PWM frequency changes.
const DRIVE_WINDOW_TICKS: u32 = 2 * 20_000;

impl<B: Board> Kernel<B> {
    pub fn new(board: B) -> Self {
        Self {
            board,
            state: KernelState::default(),
            stream_decimation_counter: 1,
            stream_duration_remaining_ticks: 0,
        }
    }

    /// Must complete well inside the kernel period (~50 µs at 20 kHz).
    pub fn on_tick(&mut self, frame: SampleFrame, _shared: &Shared) {
        self.board.pulse_tick_indicator();

        // TEMP DIAG (2026-05-20): Drive a single direction into the endstop and
        // hold there until DRIVE_WINDOW_TICKS expires — no bang-bang reversal,
        // so the motor sits in steady-state stall for the full 2 s. SWING_LIMIT
        // is unused in this mode.
        let _ = (frame.pos.0, SWING_LIMIT_URAD);
        if self.state.swing_dir == 0 {
            self.state.swing_dir = 1;
            self.state.drive_ticks_remaining = DRIVE_WINDOW_TICKS;
        }

        self.state.drive_ticks_remaining =
            self.state.drive_ticks_remaining.saturating_sub(1);

        let cmd = if self.state.drive_ticks_remaining == 0 {
            MotorCmd::Coast
        } else {
            let duty = if self.state.swing_dir >= 0 { SWING_EFFORT } else { -SWING_EFFORT };
            MotorCmd::Drive {
                duty: Effort(duty),
                decay: DecayMode::Fast,
            }
        };
        self.board.write_motor(cmd);
    }
}

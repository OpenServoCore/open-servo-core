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
    pub swing_dir: i8,
    pub drive_ticks_remaining: u32,
}

// TEMP DIAG (2026-05-20): stall into endstop for 2 s so V_shunt is dominated
// by stall current, well above the OPA input-offset floor.
const SWING_EFFORT: i16 = i16::MAX;
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
    pub fn on_tick(&mut self, _frame: SampleFrame, _shared: &Shared) {
        self.board.pulse_tick_indicator();

        if self.state.swing_dir == 0 {
            self.state.swing_dir = 1;
            self.state.drive_ticks_remaining = DRIVE_WINDOW_TICKS;
        }

        self.state.drive_ticks_remaining = self.state.drive_ticks_remaining.saturating_sub(1);

        let cmd = if self.state.drive_ticks_remaining == 0 {
            MotorCmd::Coast
        } else {
            let duty = if self.state.swing_dir >= 0 {
                SWING_EFFORT
            } else {
                -SWING_EFFORT
            };
            MotorCmd::Drive {
                duty: Effort(duty),
                decay: DecayMode::Fast,
            }
        };
        self.board.write_motor(cmd);
    }
}

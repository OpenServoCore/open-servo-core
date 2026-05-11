use crate::{Board, Shared};

/// Real-time control kernel. Runs in the PWM ISR; one `on_tick` per period.
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
}

impl<B: Board> Kernel<B> {
    pub fn new(board: B) -> Self {
        Self {
            board,
            state: KernelState::default(),
            stream_decimation_counter: 1,
            stream_duration_remaining_ticks: 0,
        }
    }

    pub fn on_tick(&mut self, _shared: &Shared) {
        self.board.pulse_tick_indicator();
    }
}

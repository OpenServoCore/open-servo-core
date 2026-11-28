//! The simulated servo behind `IdentRun.runFake`: one servo on a simulated
//! bus at the TEL rate, calibrated, its horn at rest. The sim runs no motor,
//! so its run stops at the probe's first shunt burst, as the CLI's does on
//! the same servo. Its UID is the one osc-client-web's simulated fleet
//! gives the same id, so the run folder lands under that servo's folder.

use osc_client::BaudRate;
use osc_client::fake::{FakePipe, TelSample, TelemetryTick, uid_of};
#[cfg(target_arch = "wasm32")]
use osc_client::pipe::{Pipe, PipeError};

/// The servo `id` alone on the bus.
pub fn pipe(id: u8) -> FakePipe {
    let mut pipe = FakePipe::new(BaudRate::B3000000, &[id]);
    pipe.sim_mut().seed_servo_uid(0, uid_of(id));
    pipe.seed_calibrated(0);
    pipe.set_track(0, resting());
    pipe
}

/// [`pipe`] as the page runs it: each exchange and each pause gives the
/// page a turn, as a device's transfers do, so it redraws and takes a
/// Cancel mid-step; a turn takes no sim time. The rest step makes no
/// pause, so turns at the pauses alone would leave it none.
#[cfg(target_arch = "wasm32")]
pub struct Yielding(pub FakePipe);

#[cfg(target_arch = "wasm32")]
impl Pipe for Yielding {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        let sent = self.0.send(bytes).await;
        page_turn().await;
        sent
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        self.0.recv().await
    }

    async fn pause(&mut self, d: std::time::Duration) {
        self.0.pause(d).await;
        page_turn().await;
    }
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen::prelude::wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_name = setTimeout)]
    fn set_timeout(f: &js_sys::Function, ms: i32);
}

/// A zero-delay timer: a task of its own, so the page's events, timers and
/// frames run before the run goes on. A resolved promise would not do,
/// since its microtask runs before any of them.
#[cfg(target_arch = "wasm32")]
async fn page_turn() {
    let turn = js_sys::Promise::new(&mut |go, _| set_timeout(&go, 0));
    let _ = wasm_bindgen_futures::JsFuture::from(turn).await;
}

/// A servo at rest, a count or so of noise on each channel.
fn resting() -> Vec<TelSample> {
    let noise = |i: u32, k: u32| {
        let mut h = i.wrapping_mul(0x9E37_79B9) ^ k.wrapping_mul(0x85EB_CA6B);
        h ^= h >> 15;
        h = h.wrapping_mul(0x2C1B_3C6D);
        h ^= h >> 12;
        (h % 3) as i32 - 1
    };
    (0..4096u32)
        .map(|i| {
            let at = |base: i32, k: u32| (base + noise(i, k)) as u16;
            TelSample {
                tick: TelemetryTick {
                    _scan: [0; 9],
                    pos: at(2041, 1),
                    vbus_raw: at(1241, 3),
                    ntc_raw: at(2010, 4),
                    current_raw: at(1206, 5),
                    vmotor_a: at(773, 6),
                    vmotor_b: at(772, 7),
                    duty_ticks: 0,
                    current_trough: at(1206, 8),
                    i_tick_counts: 0,
                    vdiff_counts: 0,
                    vmotor_a_trough: at(773, 9),
                    vmotor_b_trough: at(772, 10),
                    tick_flags: 0,
                    sample_tick: i,
                },
                vbus_counts: 2000,
            }
        })
        .collect()
}

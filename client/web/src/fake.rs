//! The simulated fleet behind `OscClient.fake`: the roster built into a
//! [`FakePipe`], target-independent so native tests run it.

use osc_client::boot::{Cmd, Status};
use osc_client::fake::{FakePipe, TelSample, TelemetryTick, duty_counts, tick_flag};
use osc_protocol::wire::UID_LEN;

use crate::types::{FakeFail, FakeServo, Track};

/// The pipe for `fleet`; the error names the call that refused it.
pub fn pipe(fleet: Vec<FakeServo>) -> Result<FakePipe, String> {
    if fleet.is_empty() {
        return Err("fake: the fleet needs at least one id".into());
    }
    let ids: Vec<u8> = fleet.iter().map(|s| s.id).collect();
    for (i, id) in ids.iter().enumerate() {
        if ids[..i].contains(id) {
            return Err(format!("fake: duplicate id {id}"));
        }
    }
    // The rate a servo leaves the factory at, so the fleet answers
    // before any migration.
    let mut pipe = FakePipe::new(osc_client::BaudRate::B1000000, &ids);
    for (i, s) in fleet.into_iter().enumerate() {
        pipe.sim_mut().seed_servo_uid(i, uid(s.id));
        if s.virgin {
            pipe.seed_board(i);
        } else {
            pipe.seed_calibrated_with(i, s.therm);
        }
        if let Some(track) = s.track {
            pipe.set_track(i, track_rows(track)?);
        }
        if let Some(us) = s.response_deadline_us {
            pipe.sim_mut()
                .servo_table_mut(i, |t| t.config.common.response_deadline_us = us);
        }
        let sim = pipe.sim_mut();
        if s.bootloader {
            sim.start_in_bootloader(i);
        }
        if let Some(f) = s.fail_next {
            let cmd = match f {
                FakeFail::Erase => Cmd::Erase,
                FakeFail::Write => Cmd::Write,
            };
            sim.fail_next_boot(i, cmd, Status::WriteError);
        }
    }
    Ok(pipe)
}

/// Columnar track to the sim's row form.
fn track_rows(t: Track) -> Result<Vec<TelSample>, String> {
    let n = t.pos.len();
    if n == 0 {
        return Err("fakeWithTracks: empty track".into());
    }
    let lens = [
        t.current.len(),
        t.current_trough.len(),
        t.duty_q15.len(),
        t.vdiff.len(),
        t.vbus.len(),
        t.current_raw.len(),
        t.vmotor_a.len(),
        t.vmotor_b.len(),
        t.vbus_raw.len(),
        t.ntc_raw.len(),
        t.window_valid.len(),
    ];
    if lens.iter().any(|&l| l != n) {
        return Err("fakeWithTracks: track columns differ in length".into());
    }
    Ok((0..n)
        .map(|i| TelSample {
            tick: TelemetryTick {
                i_tick_counts: t.current[i],
                vdiff_counts: t.vdiff[i],
                duty_ticks: duty_counts(t.duty_q15[i]),
                pos: t.pos[i],
                current_trough: t.current_trough[i],
                current_raw: t.current_raw[i],
                vmotor_a: t.vmotor_a[i],
                vmotor_b: t.vmotor_b[i],
                vbus_raw: t.vbus_raw[i],
                ntc_raw: t.ntc_raw[i],
                tick_flags: if t.window_valid[i] != 0 {
                    tick_flag::I_VALID
                } else {
                    0
                },
                ..Default::default()
            },
            vbus_counts: t.vbus[i],
        })
        .collect())
}

/// splitmix64 over the id: the same roster always discovers the same UIDs,
/// and they carry a real part's entropy rather than a run of zeros.
fn uid(id: u8) -> [u8; UID_LEN] {
    let mut s = 0x05C0_DE00_0000_0000 | id as u64;
    let mut out = [0u8; UID_LEN];
    for w in out.chunks_mut(8) {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        w.copy_from_slice(&(z ^ (z >> 31)).to_le_bytes());
    }
    out
}

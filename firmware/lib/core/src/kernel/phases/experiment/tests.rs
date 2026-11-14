//! The experiment image's pins: OpenLoop as written, the trip, the bus
//! dead-man, the bias tracker and the publish. On a copy of the kernel
//! tests' fake rig: the production tests compile out with the feature.

use crate::estimator::bias::{self, BiasTracker, Zero};
use crate::estimator::{bemf, window};
use crate::kernel::phases::loops::faults;
use crate::kernel::phases::loops::limits::flag;
use crate::kernel::{DECIM_MED, DECIM_SLOW, Kernel, KernelTiming, phase};
use crate::regions::control::Mode;
use crate::traits::{ControlIo, DecayMode, Motor, MotorCmd, Sensors, TelStream};
use crate::{ControlTable, RegionStorage, SensorFrame, Shared, adc};

const BIAS: u16 = 2048;
const ARR: u16 = 1200;
/// 60 % duty.
const GOAL: i16 = 19660;

// The chip-side const-eval for a 20 kHz FAST rate (MED 2 kHz).
const TIMING: KernelTiming = KernelTiming {
    pwm_arr: ARR,
    recip_arr_q24: (1 << bemf::RECIP_ARR_SHIFT) / ARR as u32,
    tick_hz: 20_000,
    dt_med_q32: ((1u64 << 32) / 2000) as u32,
    vbus_scale_q15: 34952,
    bias_brake_min_ticks: 600,
    v_trough_min_ticks: 0,
    bemf_min_ticks: 0,
    i_settle_gain: window::SettleGain::UNITY,
};

struct FakeSensors;
impl Sensors for FakeSensors {
    fn land(&mut self, _: &Shared) {}
}

struct FakeMotor {
    last: Option<MotorCmd>,
}
impl Motor for FakeMotor {
    fn write(&mut self, cmd: MotorCmd) {
        self.last = Some(cmd);
    }
}

struct FakeIo {
    sensors: FakeSensors,
    motor: FakeMotor,
}
impl ControlIo for FakeIo {
    type Sensors = FakeSensors;
    type Motor = FakeMotor;
    fn parts(&mut self) -> (&mut FakeSensors, &mut FakeMotor) {
        (&mut self.sensors, &mut self.motor)
    }
}

/// A TEL stream the test switches on and off.
#[derive(Default)]
struct FakeTel {
    streaming: bool,
}
impl TelStream for FakeTel {
    fn on_publish(&mut self, _: impl FnOnce() -> bool) {}
    fn streaming(&self) -> bool {
        self.streaming
    }
}

fn io() -> FakeIo {
    FakeIo {
        sensors: FakeSensors,
        motor: FakeMotor { last: None },
    }
}

fn kernel() -> Kernel<FakeIo, FakeTel> {
    Kernel::with_tel(io(), FakeTel::default(), TIMING)
}

fn last_cmd(k: &Kernel<FakeIo, FakeTel>) -> MotorCmd {
    k.io.motor.last.expect("a motor write happened")
}

/// The rig: a 4-window trip at 2400 counts, the shunt and the taps read
/// from 100 ticks of drive, torque off at boot.
fn seed(shared: &Shared) {
    shared.table.with_mut(|t| {
        let c = &mut t.config;
        c.loop_current.duty_max_q15 = 32767;
        c.limits.current_limit_counts = 1200;
        c.limits.drive_polarity = true;
        c.limits.oc_trip_counts = 2400;
        c.limits.oc_trip_ticks = 4;
        c.limits.openloop_decay = DecayMode::Slow;
        c.limits.openloop_zero_brake = false;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = 100;
        cal.sense.v_window_min_ticks = 100;
        t.telemetry.refs.current_bias_counts = BIAS;
        t.control.lifecycle.torque_enable = false;
        t.control.lifecycle.mode = Mode::Position;
    });
}

/// A committed CONFIG write, as the dispatcher lands it.
fn write_config(sh: &Shared, f: impl FnOnce(&mut ControlTable)) {
    sh.config_commit_begin();
    sh.table.with_mut(f);
    sh.config_commit_end();
}

/// Slow-decay frames: the trough scan lands in the brake phase, so the shunt
/// reads its offset there.
fn frame(pos: u16, current: u16) -> SensorFrame {
    SensorFrame {
        pos,
        current,
        current_trough: BIAS,
        vmotor_a: 3000,
        vmotor_a_trough: 3000,
        vmotor_b: 40,
        vmotor_b_trough: 40,
        vref_raw: 1489,
        vbus_raw: 1100,
        ntc_raw: 2048,
    }
}

/// `n` ticks of `f`.
fn scan(k: &mut Kernel<FakeIo, FakeTel>, sh: &Shared, f: SensorFrame, n: u32) {
    for _ in 0..n {
        k.on_scan(f, sh);
    }
}

/// Ticks `f` until the next tick runs medium phase `p`.
fn run_to(k: &mut Kernel<FakeIo, FakeTel>, sh: &Shared, f: SensorFrame, p: u8) {
    while k.phase != p {
        k.on_scan(f, sh);
    }
}

fn fault(k: &Kernel<FakeIo, FakeTel>) -> u8 {
    k.faults.mask()
}

/// Torque on at the goal with the mode byte Position, as a tool left it.
fn torque_on(sh: &Shared, goal: i16, deadman: u16) {
    sh.table.with_mut(|t| {
        let l = &mut t.control.lifecycle;
        l.torque_enable = true;
        l.mode = Mode::Position;
        l.goal_duty = goal;
        l.bus_deadman_ticks = deadman;
    });
}

/// The goal drives as written from the first tick: no limiter slews it, no
/// band ceilings it, the configured decay carries it, and the mode byte
/// changes nothing. `duty_max_q15` still clamps it, at the next CONTROL
/// phase after the take.
#[test]
fn openloop_duty_is_the_goal() {
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, GOAL, 0);
    sh.table
        .with_mut(|t| t.config.limits.openloop_decay = DecayMode::Fast);
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.duty_q15(), GOAL, "the first tick drives the goal");
    scan(&mut k, &sh, frame(2000, BIAS), 2 * DECIM_MED as u32);
    match last_cmd(&k) {
        MotorCmd::Drive {
            ticks,
            reverse,
            decay,
        } => {
            assert_eq!(ticks, window::drive_ticks(GOAL, ARR) as u16);
            assert!(!reverse);
            assert!(matches!(decay, DecayMode::Fast));
        }
        other => panic!("expected Drive, got {other:?}"),
    }
    assert_eq!(k.duty_q15(), GOAL);
    sh.table.with(|t| {
        assert_eq!(t.telemetry.mode.mode_active, Mode::OpenLoop as u8);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_NONE);
        assert_eq!(t.telemetry.common.fault_flags, 0);
    });
    // the other way round, in the wiring's frame
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -GOAL);
    run_to(&mut k, &sh, frame(2000, BIAS), phase::CONTROL);
    k.on_scan(frame(2000, BIAS), &sh);
    assert!(matches!(
        last_cmd(&k),
        MotorCmd::Drive { reverse: true, .. }
    ));
    assert_eq!(k.duty_q15(), -GOAL);
    // duty_max clamps the goal at the first CONTROL phase after the take
    write_config(&sh, |t| t.config.loop_current.duty_max_q15 = 5000);
    run_to(&mut k, &sh, frame(2000, BIAS), phase::CONTROL);
    assert_eq!(k.duty_q15(), -GOAL);
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.duty_q15(), -5000);
}

/// `oc_trip_ticks` consecutive valid windows over `oc_trip_counts` latch
/// `over_current`: the drive is Disabled whatever `torque_enable` says and
/// the publish names it. Torque 0 then 1 is the ack, seen at the CONTROL
/// phase after the raise, and a trip still present re-latches.
#[test]
fn the_trip_coasts_and_the_ack_rearms() {
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 16000, 0);
    let mut k = kernel();
    let over = frame(2000, BIAS + 3000);
    // tick 0 measures the boot duty of 0: an invalid window
    k.on_scan(over, &sh);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    for _ in 0..3 {
        k.on_scan(over, &sh);
        assert_eq!(fault(&k), 0);
    }
    k.on_scan(over, &sh);
    assert_eq!(fault(&k), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    for _ in 0..DECIM_MED {
        k.on_scan(over, &sh);
        assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    }
    sh.table.with(|t| {
        assert_eq!(t.telemetry.common.fault_flags, faults::BIT_OVER_CURRENT);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_OVER_CURRENT);
        assert_eq!(t.telemetry.mode.mode_active, Mode::OpenLoop as u8);
    });
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    scan(&mut k, &sh, over, DECIM_MED as u32);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    run_to(&mut k, &sh, over, phase::CONTROL);
    assert_eq!(fault(&k), faults::BIT_OVER_CURRENT);
    k.on_scan(over, &sh);
    assert_eq!(fault(&k), 0);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // the overcurrent persists: a fresh window re-latches
    scan(&mut k, &sh, over, 5);
    assert_eq!(fault(&k), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
}

/// The dead-man counts the medium periods of bus silence, DECIM_MED ticks
/// each on the RAIL phase, so a field of 400 lapses on the 40th RAIL after
/// the enable edge: tick 395 from an enable at the boot tick. A dispatched
/// instruction (`Shared::bus_touch`) reloads it from the field, a
/// streaming burst holds it, a zero field never trips, torque off clears
/// it and the enable edge restarts it from the field.
#[test]
fn the_deadman_counts_bus_silence() {
    let f = frame(2000, BIAS);
    let drives = |k: &Kernel<FakeIo, FakeTel>| {
        fault(k) == 0 && matches!(last_cmd(k), MotorCmd::Drive { .. })
    };
    let lapsed = |k: &Kernel<FakeIo, FakeTel>| {
        fault(k) == faults::BIT_DEADMAN && matches!(last_cmd(k), MotorCmd::Disabled)
    };

    // silence from the boot tick: ticks 0..394 drive, tick 395 coasts
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 400);
    let mut k = kernel();
    scan(&mut k, &sh, f, 395);
    assert!(drives(&k));
    k.on_scan(f, &sh);
    assert!(lapsed(&k));
    run_to(&mut k, &sh, f, phase::SPARE_A);
    sh.table.with(|t| {
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_DEADMAN);
        assert_eq!(t.telemetry.common.fault_flags, faults::BIT_DEADMAN);
    });

    // torque off clears the count; the enable edge at tick 410 restarts
    // 400 ticks: the 40th RAIL after it is tick 805
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    run_to(&mut k, &sh, f, phase::SLOW);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    run_to(&mut k, &sh, f, phase::CONTROL);
    assert_eq!(fault(&k), faults::BIT_DEADMAN);
    scan(&mut k, &sh, f, 395);
    assert!(drives(&k));
    k.on_scan(f, &sh);
    assert!(lapsed(&k));

    // an instruction at tick 300 reloads: the RAIL at 305 starts 400
    // afresh, so tick 705 is the lapse
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 400);
    let mut k = kernel();
    scan(&mut k, &sh, f, 300);
    sh.bus_touch();
    scan(&mut k, &sh, f, 405);
    assert!(drives(&k));
    k.on_scan(f, &sh);
    assert!(lapsed(&k));

    // a stream from tick 100 holds the 300 ticks left; released at 1100,
    // they run out on the RAIL of tick 1395
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 400);
    let mut k = kernel();
    scan(&mut k, &sh, f, 100);
    k.tel.streaming = true;
    scan(&mut k, &sh, f, 1000);
    assert!(drives(&k));
    k.tel.streaming = false;
    scan(&mut k, &sh, f, 295);
    assert!(drives(&k));
    k.on_scan(f, &sh);
    assert!(lapsed(&k));

    // a zero field never trips, however long the silence
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 0);
    let mut k = kernel();
    scan(&mut k, &sh, f, 5000);
    assert!(drives(&k));
}

/// A nonzero field written under torque arms the dead-man and trips
/// nothing, wherever the write and its touch land against the CONTROL
/// read. From 0 at the enable edge: a write of 400 landing with its touch
/// at tick 2, between CONTROL and RAIL, is carried by the CONTROL at 10
/// and loaded on the RAIL at 15, so the silence lapses at tick 415. A
/// touch at tick 2 ahead of the write, as a write frame's dispatch is
/// ahead of its commit, then the write with its own touch at tick 12,
/// loads on the RAIL at 25: tick 425.
#[test]
fn the_deadman_reloads_a_field_written_under_torque() {
    let f = frame(2000, BIAS);
    let drives = |k: &Kernel<FakeIo, FakeTel>| {
        fault(k) == 0 && matches!(last_cmd(k), MotorCmd::Drive { .. })
    };
    let lapsed = |k: &Kernel<FakeIo, FakeTel>| {
        fault(k) == faults::BIT_DEADMAN && matches!(last_cmd(k), MotorCmd::Disabled)
    };
    let write = |sh: &Shared, ticks: u16| {
        sh.table
            .with_mut(|t| t.control.lifecycle.bus_deadman_ticks = ticks);
        sh.bus_touch();
    };

    // the write and its touch at tick 2: ticks 2..414 drive, 415 coasts
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 0);
    let mut k = kernel();
    scan(&mut k, &sh, f, 2);
    write(&sh, 400);
    scan(&mut k, &sh, f, 2 * DECIM_MED as u32);
    assert!(drives(&k), "a field written under torque trips nothing");
    scan(&mut k, &sh, f, 415 - 22);
    assert!(drives(&k));
    k.on_scan(f, &sh);
    assert!(lapsed(&k));

    // a touch at tick 2 ahead of the write at 12: 425 coasts
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 0);
    let mut k = kernel();
    scan(&mut k, &sh, f, 2);
    sh.bus_touch();
    scan(&mut k, &sh, f, 10);
    write(&sh, 400);
    scan(&mut k, &sh, f, 2 * DECIM_MED as u32);
    assert!(drives(&k), "a field written under torque trips nothing");
    scan(&mut k, &sh, f, 425 - 32);
    assert!(drives(&k));
    k.on_scan(f, &sh);
    assert!(lapsed(&k));
}

/// The dead-man's loaded countdown publishes as the permit's grant, the bit
/// the burst arm reads for a seat outside the soft limits: live from the
/// enable edge, gone when the count lapses or torque goes off, never there
/// with the field at zero.
#[test]
fn the_deadman_publishes_the_permit_grant() {
    let f = frame(2000, BIAS);
    let flags = |sh: &Shared| sh.table.with(|t| t.telemetry.limits.limit_flags);

    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 400);
    let mut k = kernel();
    run_to(&mut k, &sh, f, phase::SPARE_A);
    assert_eq!(flags(&sh), flag::PERMIT, "a loaded countdown is the grant");
    scan(&mut k, &sh, f, 400);
    run_to(&mut k, &sh, f, phase::SPARE_A);
    assert_eq!(fault(&k), faults::BIT_DEADMAN);
    assert_eq!(flags(&sh), 0, "a lapsed count is no grant");

    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 400);
    let mut k = kernel();
    scan(&mut k, &sh, f, 100);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    run_to(&mut k, &sh, f, phase::SPARE_A);
    assert_eq!(flags(&sh), 0, "torque off drops the grant");

    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 0);
    let mut k = kernel();
    scan(&mut k, &sh, f, 100);
    run_to(&mut k, &sh, f, phase::SPARE_A);
    assert_eq!(flags(&sh), 0, "no field, no grant");
}

/// The image keeps the bias tracker's feed: one sample a control period on
/// the PUBLISH tick and none on the other nine phases, asleep and under
/// drive alike. Against a trough far from the zero every sample moves the
/// zero in use by a count or more, so it steps on the PUBLISH ticks alone,
/// by what one sample moves the tracker, and the same tick's publish
/// carries it.
#[test]
fn the_zero_takes_one_sample_a_period_at_publish() {
    const PERIODS: u32 = 5;
    for (torque, zero) in [(false, Zero::Asleep), (true, Zero::Awake)] {
        let sh = Shared::new();
        seed(&sh);
        if torque {
            torque_on(&sh, 8000, 0);
        }
        let mut k = kernel();
        // the zero's own value until the bridge state has settled: torque
        // off, or a Slow drive whose trough is a wide brake half
        let near = frame(2000, BIAS + 300);
        scan(&mut k, &sh, near, 10 * DECIM_MED as u32);
        assert_eq!(k.fast.bias_counts(), BIAS);
        let mut far = near;
        far.current_trough = BIAS - 1600;
        let mut model = BiasTracker::new();
        model.seed(BIAS);
        let mut samples = 0;
        for tick in 0..PERIODS * DECIM_MED as u32 {
            let next = k.phase;
            let before = k.fast.bias_counts();
            k.on_scan(far, &sh);
            let after = k.fast.bias_counts();
            if next == phase::PUBLISH {
                samples += 1;
                assert_ne!(after, before, "{zero:?} tick {tick}");
                assert_eq!(
                    after,
                    model.update(zero, (far.current_trough as u32) << adc::ADC_Q4_SHIFT),
                    "{zero:?} tick {tick}"
                );
                let published = sh.table.with(|t| t.telemetry.refs.current_bias_counts);
                assert_eq!(published, after, "{zero:?} tick {tick}");
            } else {
                assert_eq!(after, before, "{zero:?} tick {tick}");
            }
        }
        assert_eq!(samples, PERIODS, "{zero:?}");
        assert_eq!(matches!(last_cmd(&k), MotorCmd::Drive { .. }), torque);
    }
}

/// The image keeps the tracker's aging, its whole slow block: one step
/// every DECIM_SLOW periods on the SLOW phase, and an unfed zero is stale
/// after `STALE_PERIODS` of them. The brake feeds the awake zero, then
/// torque goes off. Through the last SLOW period of the horizon the rest
/// sits far off, so every sample moves the asleep zero by a count or more:
/// the awake zero in use holds until the step that ends the horizon and
/// moves with the next sample.
#[test]
fn an_unfed_zero_is_stale_after_stale_periods_of_slow() {
    let sh = Shared::new();
    seed(&sh);
    write_config(&sh, |t| t.config.limits.openloop_zero_brake = true);
    torque_on(&sh, 0, 0);
    let mut k = kernel();
    let rest = |trough: u16| SensorFrame {
        current_trough: trough,
        ..frame(2000, trough)
    };
    // the last awake sample is the PUBLISH ahead of the CONTROL phase that
    // reads torque off
    scan(&mut k, &sh, rest(BIAS), 100 * DECIM_MED as u32);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(k.phase, phase::CONTROL);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    // one tick of `f`; whether it ran the aging step
    let step = |k: &mut Kernel<FakeIo, FakeTel>, f: SensorFrame| {
        let (next, before) = (k.phase, k.medium.slow_periods);
        k.on_scan(f, &sh);
        let aged = k.medium.slow_periods < before;
        assert!(!aged || next == phase::SLOW);
        aged
    };
    let (mut aged, mut ticks) = (0, 0);
    while aged < bias::STALE_PERIODS - 1 {
        aged += step(&mut k, rest(BIAS)) as u8;
        ticks += 1;
    }
    assert!(ticks <= aged as u32 * DECIM_SLOW as u32 * DECIM_MED as u32);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    let far = rest(BIAS - 1600);
    let mut samples = 0;
    loop {
        samples += (k.phase == phase::PUBLISH) as u32;
        let last = step(&mut k, far);
        assert_eq!(k.fast.bias_counts(), BIAS, "sample {samples}");
        if last {
            break;
        }
    }
    assert_eq!(samples, DECIM_SLOW as u32);
    assert_eq!(k.phase, phase::PUBLISH);
    k.on_scan(far, &sh);
    assert!(k.fast.bias_counts() < BIAS, "{}", k.fast.bias_counts());
}

/// The image reads a wide code as the middle of its band where the
/// production image does, on what it computes: the window's shunt sample
/// and taps, the zero, and so the trip. A drive with the crest shunt and
/// each tap on a wide code reads the current and the differential the
/// codes their slots up read without a map, the raw fields stay the
/// codes, and a crest under the trip without its slot trips with it.
#[test]
fn a_wide_code_reads_mid_band_and_the_trip_follows() {
    const TRIP: u16 = 1030;
    // the held current, the differential, the fault mask and the zero in
    // use after a drive on `f` under `map`
    let run = |map: [u8; adc::ADC_WIDE_SLOTS], f: SensorFrame| {
        let sh = Shared::new();
        seed(&sh);
        write_config(&sh, |t| {
            t.config.limits.oc_trip_counts = TRIP;
            t.calib.adc.adc_wide_mid_q4 = map;
        });
        torque_on(&sh, 8000, 0);
        let mut k = kernel();
        scan(&mut k, &sh, f, 10 * DECIM_MED as u32);
        sh.table.with(|t| {
            let b = &t.telemetry.tick;
            assert_eq!(
                (b.current_raw, b.vmotor_a, b.vmotor_b, b.current_trough),
                (f.current, f.vmotor_a, f.vmotor_b, f.current_trough)
            );
            (
                b.i_tick_counts,
                b.vdiff_counts,
                fault(&k),
                t.telemetry.refs.current_bias_counts,
            )
        })
    };
    let none = [0u8; adc::ADC_WIDE_SLOTS];
    let mut map = [0xa0u8; adc::ADC_WIDE_SLOTS];
    map[0] = 0x60;
    let stuck = SensorFrame {
        current: 0x8ff,
        vmotor_a: 0xbff,
        vmotor_b: 0x0ff,
        ..frame(2000, BIAS)
    };
    let mid = SensorFrame {
        current: stuck.current + 10,
        vmotor_a: stuck.vmotor_a + 10,
        vmotor_b: stuck.vmotor_b + 6,
        ..stuck
    };
    let want = run(none, mid);
    assert_eq!(want, (0x8ff + 10 - BIAS as i16, 0xb00 + 4, 0, BIAS));
    assert_eq!(run(map, stuck), want);
    assert_eq!(run(none, stuck), (0x8ff - BIAS as i16, 0xb00, 0, BIAS));
    // the crest on 0xbff: 1023 counts over the zero, 1033 with its slot
    let hot = SensorFrame {
        current: 0xbff,
        ..frame(2000, BIAS)
    };
    assert_eq!(run(none, hot).2, 0);
    assert_eq!(run(map, hot).2, faults::BIT_OVER_CURRENT);
    // the zero on a wide code settles on the middle of its band
    let sh = Shared::new();
    seed(&sh);
    write_config(&sh, |t| t.calib.adc.adc_wide_mid_q4 = map);
    let mut k = kernel();
    let rest = SensorFrame {
        current_trough: 0x7ff,
        ..frame(2000, 0x7ff)
    };
    scan(
        &mut k,
        &sh,
        rest,
        (12 << bias::ALPHA_SHIFT) * DECIM_MED as u32,
    );
    assert_eq!(k.fast.bias_counts(), 0x7ff + 10);
}

/// The publish carries what the image computes and nothing else: the bias
/// and the window floors, the mode as OpenLoop, the fault; the estimates,
/// the limits and the thermal fields read zero through a drive.
#[test]
fn publish_is_the_truth() {
    let sh = Shared::new();
    seed(&sh);
    torque_on(&sh, 8000, 0);
    let mut k = kernel();
    scan(&mut k, &sh, frame(1234, BIAS + 300), 4 * DECIM_MED as u32);
    sh.table.with(|t| {
        assert_eq!(t.telemetry.tick.pos, 1234);
        assert_eq!(t.telemetry.tick.i_tick_counts, 300);
        assert_eq!(t.telemetry.refs.current_bias_counts, BIAS);
        assert_ne!(t.telemetry.limits.window_floor_q15, 0);
        assert_ne!(t.telemetry_ext.limits_ext.window_v_floor_q15, 0);
        assert_eq!(t.telemetry.mode.mode_active, Mode::OpenLoop as u8);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_NONE);
        assert_eq!(t.telemetry.common.fault_flags, 0);
        let e = &t.telemetry.estimates;
        assert_eq!(
            (
                e.theta_hat_q16,
                e.omega_hat_cps,
                e.tau_d_counts,
                e.i_lim_counts,
                e.t_winding_cc,
                e.vbus_counts,
                e.omega_bemf_cps
            ),
            (0, 0, 0, 0, 0, 0, 0)
        );
        assert_eq!(t.telemetry.mode.omega_hat_src, 0);
        assert_eq!(t.telemetry.limits.limit_flags, 0);
        assert_eq!(t.telemetry_ext.therm.t_ntc_cc, 0);
        assert_eq!(t.telemetry_ext.therm.therm_flags, 0);
    });
    scan(&mut k, &sh, frame(1234, BIAS + 3000), 2 * DECIM_MED as u32);
    sh.table.with(|t| {
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_OVER_CURRENT);
        assert_eq!(t.telemetry.common.fault_flags, faults::BIT_OVER_CURRENT);
        assert_eq!(t.telemetry.limits.limit_flags, 0);
    });
}

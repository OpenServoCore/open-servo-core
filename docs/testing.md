# Testing: the three gears

The transport is verified by three test gears, each proving something the others
structurally cannot. Green across all three — `scripts/gears.sh` — is what lets
us pronounce the tree good.

| gear | proves | where | gate |
|------|--------|-------|------|
| **1 — unit** | the pieces are individually correct: codec, CRC vectors, control-table rules, driver internals | in-crate `#[cfg(test)]` across `firmware/lib/*` and `tools/bench` | CI |
| **2 — DES** | the pieces compose correctly under adversarial sequencing, at every baud, deterministically | `firmware/lib/integration` (discrete-event sim) | CI |
| **3 — bench** | the real silicon meets timing and survives zero-gap load under real ISR/DMA/drift | `tools/bench/tests/hardware` (hardware-in-the-loop) | on-rig |

The division of labour is deliberate. **Gear 2 is wide but blind to time**: the
sim dispatches with a *zero* CPU-time model, so it proves logical correctness and
sequencing at every baud but cannot see ISR latency, wall-clock turnaround, HSI
drift, or the frame-N-deadline-vs-frame-N+1-break window. **Gear 3 is the deep-
on-timing, narrow-on-logic gear**: its whole job is the silicon-only failure
modes gear 2 erases. So for every property the sim is structurally blind to,
gear 3 carries one hardware assertion with a measured budget — it is complete on
silicon-only properties, not on logic.

## Running

```sh
scripts/gears.sh
```

Gears 1 and 2 are deterministic and need no hardware. Gear 3 needs an
osc-adapter with a flashed V006 on its bus; the script auto-detects the adapter
by its USB identity and **skips** gear 3 (rather than failing) when no rig is
attached. Force-skip with `SKIP_BENCH=1`; force-run with `BENCH_FORCE=1`. The
suite targets servo id 1 at the 1 M boot baud by default; override with
`BENCH_ID` / `BENCH_BAUD`.

The gears map to plain cargo invocations if you want to run one directly:

```sh
( cd firmware/lib && cargo test --workspace )            # gear 1 + 2
( cd tools/bench  && cargo test --lib )                  # gear 1 (bench units)
( cd tools/bench  && cargo test --test hardware -- --test-threads=1 )   # gear 3
```

## Torque limit pins

The OpenLoop duty limiter, the stall permit lease and the class default
limits (control-theory "Limits", protocol sec 5.8) are pinned in gears 1
and 2. The DES pins run the kernel against the MG90-scale R-L plant rig
(`integration/src/plant.rs`: 4.9 ohm winding, 7.9 V rail), so they
read the winding current the limiter only sees through the shunt window
one tick late. Unless a pin says otherwise the limit is 280 counts and
the goal is 64% duty, in both directions. Paths below are under
`firmware/lib`.

The limiter law (`core/src/kernel/duty_limit.rs`):

- `blind_goal_passes_raw` - a goal at or under the window floor comes
  back unchanged at any current, a zero limit included.
- `ceiling_rises_at_the_up_rate_from_the_floor` - below the band the
  ceiling climbs 128 per tick from the floor and stops at the goal.
- `ceiling_holds_inside_the_band` - current from `lim - lim/8` up to
  `lim` leaves the ceiling where it is.
- `ceiling_drops_by_the_overage` - 10 counts over drops the ceiling by
  80 and records a pin.
- `goal_cut_applies_at_once` - a lowered goal lands the same tick;
  raised again, it slews from the cut.
- `zero_limit_never_rises` - a zero limit neither rises nor underflows.
- `ceiling_under_the_floor_applies_the_base` - under the floor the base
  applies, never above the goal, and the rise back past the floor is
  the re-probe.
- `slew_alone_is_never_pinned` - neither a slew nor a hold at the goal
  pins, only a ceiling held under the goal does, and taking the pin
  clears it.

The floor, the observer and the defaults (`core/src/estimator/window.rs`,
`fusion.rs`, `core/src/regions/config.rs`):

- `floor_duty_is_window_valid` - the floor duty clears the current
  window for floors of 100, 160 and 240 ticks; 160 of 1200 is 4356.
- `floor_duty_is_the_smallest_valid_duty` - one Q15 step less misses
  the window, for every floor up to 1200 ticks, and up to 512 at five
  other periods.
- `floor_duty_saturates_past_full_duty` - a floor past the period asks
  for full duty.
- `unset_model_holds_tau_d_at_zero` - with the motor model unset, a
  driven and moving shaft never moves tau_d off zero while theta still
  tracks.
- `arm_b_rig_reproduces_the_count_seeds` - the class defaults land as
  184 / 111 / 55 / 184 / 1843 / 92 counts on the 33 mohm chain.
- `current_counts_scales_with_the_shunt_and_saturates` - 300 mA is 335
  counts on the 60 mohm chain.

The lease and the flags (`core/src/kernel/tests.rs`,
`core/src/services/bus/tests/mod.rs`):

- `permit_lease_expires_without_a_host` - one write opens the endstop
  for 0.99 to 1.01 s, then it stays closed; the kernel never clears the
  request byte.
- `permit_rewrite_extends_the_lease` - a rewrite at 0.8 s runs the
  lease a second from the rewrite, and a write of false revokes inside
  one medium tick.
- `torque_off_drops_the_permit` - torque off ends the lease, and
  re-enabling inside the second does not revive it.
- `permit_written_with_torque_off_never_grants` - a request written
  under torque off grants nothing once torque comes back on.
- `limit_flags_name_the_governor` - each bit alone: ceiling in OpenLoop,
  yield after a Current-mode stall, endstop past a soft wall, permit
  with the endstop dropped.
- `permit_write_bumps_the_generation_only_when_set` - only a committed
  true with torque on grants; a torque-off request, false, a span that
  misses the byte and a staged HOLD write do not, and one span over
  torque and permit is judged as committed.

The limiter on the plant rig (`integration/tests/torque_limit.rs`):

- `openloop_locked_rotor_mid_travel_holds_at_i_lim` - 2 s on a locked
  rotor: peak within 1.1x the limit, mean after the first 100 ms within
  0.85x to 1.0x, no fault.
- `openloop_stall_holds_at_i_lim` - the same, seated on a hard stop
  inside the soft limits.
- `openloop_first_edge_stays_under_i_lim` - 0 to 100% on a locked rotor
  peaks within 1.05x the limit over the first 20 ms.
- `openloop_ceiling_releases_when_the_shaft_frees` - a governed 40% goal
  is back at the goal within 100 ms of the shaft freeing and stays
  there.
- `openloop_blind_goal_is_untouched` - a 12% goal under the floor
  applies exactly, from the first tick, on a locked rotor.
- `openloop_reversal_restarts_from_the_floor` - reversing out of a held
  64% restarts at the floor and never outruns the 128-per-tick slew.
- `openloop_wall_hit_at_speed_recovers_inside_1ms` - a stop hit above
  1000 counts/s is over 1.1x the limit only within the first
  millisecond, no fault.
- `openloop_stall_yields_like_closed_loop` - Yield response: the limit
  folds to the yield value no sooner than `stall_time_ms` and within
  100 ms after it, and stays folded.
- `openloop_stall_faults_on_the_boot_response` - Fault response: STALL
  latches in the same window and the drive disables.
- `openloop_slew_never_counts_as_a_stall` - with a 1 ms stall timer, a
  blind start and a slew to 64% across the travel never trip.
- `openloop_stall_under_permit_never_trips` - a permit rewritten every
  250 ms holds a locked rotor at the limit for 2 s with no fold and no
  fault.
- `dead_host_stall_trips_within_the_lease` - one permit write, then
  silence: STALL latches 1.5 to 1.6 s after it.
- `blind_band_caps_at_i_lim_when_r_is_known` - a limit of 200, under the
  floor's 239-count stall current: most ticks sit under the floor, each
  at a duty stalling at 95 to 100% of the limit, mean within 1.1x.
- `yield_fold_reaches_the_blind_band` - after a fold to a yield of 168
  the blind band holds the yield the same way, mean within 1.2x.
- `virgin_blind_band_passes_to_the_window_floor` - with `r_q12` at 0
  the duty never goes under the floor and sits there pinned, at the
  floor's stall current.
- `window_floor_is_published_as_the_limiter_uses_it` - for 160 and 240
  ticks the published `window_floor_q15` is `floor_duty` of the board's
  period, and the duty a virgin stall pins at equals it.

A virgin servo (`integration/tests/class_defaults.rs`, boot seed on the
60 mohm chain):

- `virgin_openloop_run_never_collision_trips` - free runs at +/-64%, 30%
  and 0 raise no fault and leave tau_d at zero.
- `virgin_servo_stalls_at_the_class_limit` - locked at 64%, it holds at
  the 335-count class limit: peak within 1.1x, mean 0.85x to 1.0x, no
  fault inside the stall time.

## Identification and calibration pins

How `osc ident` and `osc cal` plan their drives under the current limit
(control-theory "Open Loop Under the Same Band" and "Calibration",
protocol sec 5.8) is pinned in the host crates' own unit tests: `ident`
and `tools/osc` at the repo root, each `cargo test` in its directory
and in CI. The drives run against a fake servo that models the limiter,
the stall timer and fold, the permit lease and the burst; the bench
MG90 on it is a 4.9 ohm winding, a 280-count limit, the 13.3% window
floor, stops at 209 and 3849 and soft limits at 432 and 3626, on a 2S
rail of 3204 vcounts or a USB rail of 1780. Paths below are under
`ident/src` unless they say otherwise.

The servo's limits and the stall-safe plan (`limits.rs`):

- `servo_limits_take_the_floor_from_the_servo` - the limits carry the
  floor a telemetry read publishes, 4356 and 6534 alike, and the stop
  ladder starts on it.
- `a_servo_without_a_floor_is_refused_in_plain_words` - a published 0
  refuses with a message naming the missing sensor floor; no board
  constant stands in.
- `board_default_limits_are_refused` - soft limits that span the pot, or
  are out of order, refuse with "run `osc cal` first".
- `guards_come_from_the_soft_limits` - the travel guard sits 100 counts
  inside each soft limit, each end overridable alone.
- `i_abort_defaults_a_quarter_over_the_current_limit` - 350 at a limit
  of 280; lower is taken, higher is refused in plain words.
- `every_stall_safe_duty_stalls_under_the_limit` - seek, hold and stop
  cap stall at or under the limit however they are rounded, over five
  limit and rail pairs; on 2S the cap is 15.5%, the hold 7.76% and a
  step over an 80-count base 9.1%.
- `bootstrap_duty_is_class_safe` - the class-safe duty is 9.5% at the
  bench limit on 2S and stalls at the limit on a 3.0 ohm winding; on
  USB it applies the same volts.
- `plan_over_the_limit_is_refused_in_plain_words` - a deliberate stall
  over the limit names the duty, the current and the highest duty the
  limit holds.
- `stall_ladder_rungs_stay_under_the_limit` - every stop-ladder dwell
  sits between the window floor and the stall-safe cap, 0.5% apart.
- `the_stop_ladder_needs_room_over_the_window_floor` - on 2S at 280 the
  band from 13.3% to 15.5% is refused; on USB it dwells at 13.3, 18.2,
  23.0 and 27.9%.
- `burst_rungs_fit_the_fit_and_the_allowance` - 25/40% at 7.9 and
  4.39 V, 25/38% at 8.4 V, 25/35% at 9.1 V, none above it; every top
  rung at most 3.2 V.
- `burst_count_is_bounded` - repeats are cut so a run arms at most 24.
- `the_lease_is_held_only_behind_torque_on` - a permit written with
  torque off is not held, a held one is due every 250 ms, torque off
  drops it, and a stream over 750 ms with it held is refused.
- `stall_settings_above_the_limit_are_warned` - a yield not under the
  limit and a collision trip over twice it are warned in plain words.

The run order and cal's sequence (`run.rs`):

- `run_all_stops_at_the_first_abort` - an abort at any stage ends the
  run there; a declined burst ends it, a declined ladder still ends
  with the closing centring.
- `default_run_never_stalls_a_stop` - the whole default run on both
  rails, and on a rail too high for the burst, never presses a stop and
  never writes the permit.
- `burst_waits_for_the_nudge` - no burst before the jam check moved the
  shaft, and none at 9.4 V.
- `nudge_cap_is_two_volts_on_the_rail` - class-safe start 9.5% and cap
  25.3% on 2S, 17.1% and 45.6% on USB.
- `burst_cfg_is_the_allowance` - 25/40% from rest, the from-a-hold
  control from the class-safe duty, repeats cut to the arm count.
- `bench_mg90_run_reaches_the_fit_on_2s_and_usb` - R and L from the
  burst, a ladder of three or more rungs, an inertia fit, at most 24
  arms, ending near mid travel with torque off.
- `r_and_l_are_measured_before_any_planned_drive` - before the burst
  nothing exceeds the jam check's cap or the 3.2 V allowance; after
  it, every stall-safe drive stays at or under the stop cap.
- `bench_inertia_base_and_steps` - seek 10%, base 15%, steps to 19.5,
  21.8 and 24.1% on 2S and 23.2, 27.3 and 31.5% on USB.
- `virgin_run_escalates_the_nudge_and_measures_r` - a shaft that needs
  more than the class-safe duty is raised until it moves, then the
  burst measures R.
- `a_mid_travel_jam_ends_the_run_at_the_first_seek` - a locked shaft
  ends the run in the jam check, at or under its cap, with no burst and
  no stream.
- `the_stop_ladder_runs_only_on_request` - a declined burst ends the
  run unless asked; asked, on 2S at 280 it refuses for want of room,
  on USB it runs in the burst's place.
- `the_stop_ladder_measures_r_when_asked` - on USB, both stops stalled
  over the floor and under the limit with the permit held, R within
  2%, the run planned from it to the end.
- `cal_order_and_its_ends` - bias, jam check, burst, stops, traverse,
  centring; a declined burst or a high rail goes on without R, an
  abort ends where it stands.
- `cal_duties_on_the_bench_servo` - a shaft that moved at 13%
  approaches at 15% on both rails and seats at 7.76% on 2S and 14.0%
  on USB, half the limit.
- `cal_approaches_at_the_duty_that_moved` - moved at 14.5% on 2S, the
  approach is capped at the stop cap; at 17.1% on USB it is 19.1%; the
  permit is held only while the stops are found.
- `cal_finds_the_stops_on_a_virgin_servo` - no stops or soft limits
  known: the same run finds 209 and 3849 and ends at mid travel.
- `cal_never_escalates_toward_a_stop` - toward a stop only the approach
  duty, the seat duty and 0 are ever written; the traverse's first pass
  and the closing centring run at the approach duty.
- `cal_refuses_when_breakaway_is_over_the_limit` - moved at 17% on 2S,
  307 counts over 280: refused before any stop drive, no permit
  written.
- `cal_without_r_seats_at_the_class_half_duty` - the burst declined:
  approach at the moving duty, seat at 4.76% on 2S.
- `cal_on_a_jammed_shaft_writes_nothing` - a locked shaft ends cal in
  the jam check; only drive fields in CONTROL were ever written.

The runway (`runway.rs`):

- `room_is_the_guard_less_the_start_band` - 2919 counts on the bench
  guard, start band edges and brake points as the rule places them.
- `bench_mg90_runway_fits_the_rungs_to_64_and_not_100` - with the 2S
  pilot envelope, 26% needs 457, 40% 992, 64% 2350 and fit; 80% 3564
  and 100% 5431 do not.
- `stale_envelope_is_ignored` - an envelope of another supply, with a
  stop more than 150 counts off, or without a coast sizes nothing.
- `self_sizing_follows_the_measured_rungs` - one speed scales in
  proportion, two make a line, a stop scales by speed squared above
  and in proportion below.
- `coast_falls_back_to_the_measured_runs` - the coast law fits the
  bench coasts; one coast scales by speed squared.
- `need_adds_climb_run_and_a_margined_stop` - need is the climb, the
  run and 1.25 stops, and a zero acceleration never fits.

Stops and stillness (`exp/seek.rs`):

- `a_mid_travel_rest_is_not_a_stop` - a rest is a stop only within 150
  counts of the stop driven at; with the stops unknown, only after 100
  counts of travel.
- `clear_of_the_stops_is_300_counts_from_both` - the raise zone starts
  300 counts in from each stop.
- `only_a_drive_off_a_stop_leaves_it` - within 150 counts of a stop and
  pointing away; never with the stops unknown.
- `stillness_is_net_travel_over_a_window` - jitter is not travel, and a
  turn inside a window is not a rest.

The jam check (`exp/centre.rs`):

- `a_centred_shaft_is_left_alone_unless_nudged` - a centred shaft gets
  no write; with the check asked for it goes out and back at the start
  duty.
- `nudge_raises_the_duty_only_clear_of_the_stops` - raised 2.5% per
  still window at mid travel and held at first travel; 250 counts from
  a stop it is blocked, never raised.
- `nudge_gives_up_at_the_cap` - a locked shaft is raised to the cap,
  never over, and ends blocked.
- `a_yield_fold_ends_the_nudge_blocked` - the firmware's stall fold is
  the verdict at once.
- `slow_seek_is_raised_not_blocked` - a shaft creeping under the
  stillness speed gets a raise, not a verdict.
- `unknown_stops_try_the_other_direction` - with no stops known, a
  still check tries the other way once.

Cal's stop finder (`exp/endstop.rs`):

- `cal_seats_at_half_the_limit_after_backing_down` - approach, seat,
  hold 300 ms, the stop is the mean of the last 8 polls, then leave;
  the permit held throughout and withdrawn at the end.
- `recal_rides_a_sticky_spot_under_the_cap` - a mid-travel sticky spot
  raises the approach under the cap, and it carries on to the stop.
- `a_sticky_spot_over_the_cap_is_blocked` - the same spot at the cap
  ends blocked.
- `a_blocked_shaft_finds_no_stops` - an approach that never travels
  ends the run, and rests that do not bracket the start 1500 counts
  apart are no stops.

The ladder (`exp/ladder.rs`):

- `free_running_rungs_fit_the_runway` - every rung on both rails sized
  inside the room and braked at its end, the top rung over twice the
  stall-safe duty.
- `ladder_stops_climbing_at_the_first_rung_that_does_not_fit` - with
  the 2S envelope, rungs to 55% run, 80% ends the ladder, and nothing
  at or over it is driven.
- `a_climb_that_never_ends_is_blocked_or_declined` - still governed
  after twice its predicted climb: blocked when still, declined as too
  heavy when moving.
- `a_governed_rung_is_declined_and_the_ladder_goes_on` - a rung with no
  clean window is declined and does not count; the next rung runs.
- `a_yield_fold_on_a_rung_ends_the_run_blocked` - the stall fold ends
  the run inside the climb budget.
- `thin_ladder_is_declined` - two rungs both ways, or a top speed under
  twice the bottom one, declines.
- `abort_threshold_rides_over_a_governed_climb` - a governed climb never
  reaches the abort a quarter over the limit.

Inertia (`exp/inertia.rs`):

- `inertia_steps_from_a_moving_base` - each step leaves the moving base
  and is sized from its running current; none reaches the limit.
- `a_governed_step_is_declined` - a step the limiter holds under its
  goal is declined, never fitted.
- `a_yield_fold_on_the_base_ends_the_run_blocked` - a base held into a
  jam ends the run before any step.
- `inertia_base_and_steps_fit_the_runway` - every base and step sized
  inside the runway, every drive braked.
- `an_inertia_step_that_does_not_fit_is_skipped` - on a short travel
  the larger steps are skipped in plain words and the run goes on.

Cal's traverse (`exp/sweep.rs`):

- `the_guard_sits_inside_the_stops_and_the_soft_guard` - 150 counts in
  from each stop, and inside the soft limits' guard.
- `cal_traverse_brakes_inside_the_stops` - the 26% run is captured and
  brakes inside the guard on both rails, never touching a stop.
- `an_unsized_traverse_does_not_run` - nothing measured and no envelope:
  no run.
- `a_jam_mid_run_is_blocked` - a shaft that locks during the capture
  ends the traverse where it stopped, torque off.
- `a_jam_on_the_way_to_the_start_is_blocked` - so does one locked on
  the way to the start band: blocked, not a start.

Governed windows, the envelope and the permit (`exp/mod.rs`):

- `governed_window_is_declined_never_fitted` - a window held under the
  goal after it was reached never reaches a fit.
- `slew_samples_are_trimmed_not_declined` - a 64% climb from rest is
  trimmed, not counted against the goal.
- `governed_climb_is_trimmed_and_the_settled_windows_fit` - a climb held
  at the limit is trimmed and every settled window fits.
- `a_free_running_governed_climb_fits_only_settled_windows` - the bench
  64% rung hands on only windows at the goal.
- `abort_default_clears_a_held_stall` - a locked shaft held at the limit
  trips an abort at the limit and never the default.
- `permit_follows_torque_on` - the permit is written after every torque
  enable and withdrawn at the end.
- `long_pause_is_sliced_under_the_lease` - a 3 s pause rewrites the
  permit every 250 ms.

The stop routes asked for (`exp/resistance.rs`, `exp/held.rs`):

- `a_mid_travel_jam_is_not_a_stop` - a seek that rests short of the stop
  ends the run before any dwell.
- `rung_with_no_clean_window_reports_declined` - dwells the limit holds
  under their duty are declined.
- `pump_refreshes_a_held_permit` - a held run outlasting the lease keeps
  the stop open, every rewrite with torque on.
- `an_abort_mid_burst_still_withdraws_the_permit` - a fault mid run ends
  with duty 0, torque off, permit off.
- `escalation_only_leaves_a_stop` - the duty rises leaving a stop; a
  seek from mid travel stays at the hold duty and aborts.

The burst handshake and the driver (`burst.rs`, `tools/osc/src/rig/pump.rs`):

- `a_rejected_arm_reports_and_releases` - a `REJECTED` arm is an error
  and the arm is released.
- `a_held_permit_refuses_a_stream_it_would_lapse_inside` - 750 ms
  passes, 751 does not, and torque off holds nothing.

## Gear 3 in detail

The hardware suite drives a flashed V006 through the osc-adapter and asserts
on the wire itself: the adapter's timer captures every edge on the bus pin
(its own TX included) and the bench decodes those edges into byte+tick stamps
host-side, so assertions run on what the wire did, not on what any UART
thinks it heard. No wlink, no chip-counter reads; the wire is the failure
surface. The suite sweeps the full baud matrix (0.5 M / 1 M / 2 M / 3 M).

- **turnaround** (`turnaround.rs`) — THE metric: instruction wire-end → status
  break fall, per baud, gated for ping AND read AND write. 1 M is the tuned
  sweet spot (~35 µs ping); the ceiling is baud-aware because turnaround rises
  at both higher and lower baud on this silicon, and the acked-WRITE gate sits
  near the ~89 µs rules-dominated dispatch floor (the production hot loop pays
  none of it — GWRITE is NOREPLY).
- **rescue** (`rescue.rs`) — the §9.1 pulse reaches a servo at any baud, and
  the full field-recovery flow (rescue → prefix-walk → ASSIGN → SAVE → reboot
  → FACTORY). Both tests end with a rescue-based recovery tail so a transient
  capture dropout never strands the bench unit.
- **trim** (`trim.rs`) - the §9.3 clock discipline on real oscillators: CAL
  trains converge the DUT's trim, the lying-train probe pins plant direction,
  and the host-detune probe (an off-catalog rate one BRR step from nominal)
  exercises the differential tracker.
- **hot loop** (`hot_loop.rs`) — the production `[GWRITE(HOLD), COMMIT, GREAD]`
  zero-gap loop, the silicon twin of the DES `hot_loop` suite. The GREAD must
  read back the just-committed value every cycle; a stale read-back is a
  silently-dropped frame.
- **plain flood** (`hot_loop.rs`) — an aggressive `[WRITE(NOREPLY) × 8, READ]`
  flood that surfaces the low-baud framer floor.
- **ping / read / write / hold_commit / chain / profile / mgmt / silence** -
  the single-servo instruction set, coordinated reads, and the management
  plane happy paths.

Longer soak: `BENCH_BURST_CYCLES=25000 scripts/gears.sh`.

### The strict burst gate

The burst tests assert **zero failures** — no stale read-backs, no missed or
malformed replies — at every baud, with no tolerance budget. That strictness
is what root-caused the former "low-baud glitch" (a phantom rescue confirm
aliasing data bits, and a CPU DATAR read killing the RX byte mid-reception —
see `osc-servo-transport.md` §7 and `osc-native-protocol.md` §9.1); the
tests went green on their own once the real bugs died, exactly as designed. Keep the gate strict: a red run prints
the exact baud and the `stale` / `no-reply` / `other` breakdown, and the
measurement helpers do not retry, so a first-exchange failure is a real
signal too.

The `tool-*` binaries in `tools/bench/src/bin` are the forensic instruments
behind these tests — `tool-burst` shares the exact cycle engine the hot-loop
test asserts on; `tool-snoop` passively dumps the decoded edge capture when
a wire artifact needs root-causing. Bus operation (discovery, provisioning,
rescue, calibration) is the `osc` CLI's job (`tools/osc`, built on
osc-client), not the bench's.

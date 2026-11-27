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

The timing pins of gear 2 that do model CPU time (the kernel lane, the bus
handler costs, the TEL lane) charge one table, `osc_servo_core::budget`, in
HCLK ticks. Gear 3's `budgets` test holds the chip to the same table, so while
it is green no modelled cost is cheaper than silicon. Every gating pin runs at
budget (every body at its maximum, every injected backlog at the worst the
table allows): the sim is a bound. The typical lane (bodies drawn below their
maxima at the bench-held mean budgets) is a forecast for reports and gates
nothing. The kernel rows go by drive arm (Open, Closed, OpenLoop): a host
hold charges the Closed rows for the measured approach after its goal moves,
then the Open rows; a moving servo, stepping or driven OpenLoop, charges per
phase the dearer of the Closed and OpenLoop rows.

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
( cd tools/bench  && cargo test --test usb_session -- --test-threads=1 ) # gear 3, adapter sessions
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

The limiter law (`core/src/kernel/phases/loops/duty_limit.rs`):

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
- `bemf_floor_voids_the_observer_and_keeps_the_differential` - a window
  one tick under the board's back-EMF floor leaves the back-EMF velocity
  at 0 while the terminal differential still lands; at the floor the
  boxcar reads its pinned value.
- `unset_model_holds_tau_d_at_zero` - with the motor model unset, a
  driven and moving shaft never moves tau_d off zero while theta still
  tracks.
- `arm_b_rig_reproduces_the_count_seeds` - the class defaults land as
  307 / 111 / 55 / 307 / 1352 / 92 counts on the 33 mohm chain.
- `current_counts_scales_with_the_shunt_and_saturates` - 500 mA is 554
  counts on the 60 mohm chain.

The current the limiter reads (`core/src/estimator/window.rs`,
`core/src/kernel/tests.rs`, `integration/tests/window_current.rs`):

- `settle_gain_bands_start_at_their_edge` - each 8-tick band takes its
  gain from its first tick to its last.
- `settle_gain_clamps_below_and_is_unity_above` - a window under the
  first band takes the first band, one past the last band and every
  window of an empty table read at unity.
- `current_gain_rounds_and_keeps_the_sign_symmetric` - 300 counts at
  33259 read 304 over the zero and -304 under it, rounded half up.
- `the_term_is_the_conductance_on_the_headroom_and_the_rail` - the
  window term is `g x V x (arr - ticks) / arr` in Q16 to the floors'
  share at every width, a fresh term's first take to half a count more,
  0 at full duty whatever the carry, 25 counts at the 64-tick floor on
  the 5 V rail at the set of record's 4.75 mS.
- `the_takes_sum_to_the_exact_term_over_a_slow_ramp` - over a duty
  ramping 0.2 counts a tick across the period and back, the carried
  takes sum to the Q16 term within half a count, where a whole-count
  term's running sum strays past 14 counts.
- `the_reading_takes_the_term_off_along_the_drive` - the published
  current is the sample less the term's whole count or the next one up,
  signed by the drive, and over 100 periods the readings take the Q16
  term within one count; with no conductance the sample's, bit for bit.
- `the_dither_slope_reads_the_exact_window_term` - on the record rig at
  the 2500 c/s cruise, worked and rested contacts, the learner's dither
  slope on the published current lands within its derived noise bound
  (five sd of the shunt code's rounding through the fit, about 0.02 ohm)
  of the slope on the exact term, and a whole-count term on the same run
  lands outside it (0.11 and 0.05 ohm high).
- `the_overcurrent_trip_reads_the_uncorrected_current` - a sample 10
  counts over the trip latches it on the fourth window while the reading
  sits under it.
- `a_held_rotor_carries_the_goal_and_reads_the_winding_current` - on the
  record rig with the plant's shunt carrying 5.09 mS, Current mode holds
  the winding within 5 counts of 150 and 400 either way, and the
  published current within 5 counts of the winding.
- `current_unity_gain_is_exact` - unity returns the bias-subtracted
  sample unchanged over the whole u16 range.
- `settle_gain_lifts_a_short_window_and_leaves_the_raw_sample` - a
  100-tick OpenLoop window reads 304 for 300 counts in the stream's
  `current` while `current_raw` stays raw; past the table it reads 300.

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
  yield after a Current-mode press over `yield_current`, endstop past a soft
  wall, permit with the endstop dropped, the goal capping a Velocity
  drive and not an OpenLoop one, a long hold over `yield_current` folding
  nothing, the rail fold binding on a sag into its band.
- `off_soft_limits_wall_at_the_physical_stops` - soft limits 0 / 0 are
  off: the take holds the physical stops in their place, at or past a
  stop the current toward it is 0 and the endstop flag set, the permit
  lifts it, and the table keeps 0 / 0.
- `off_soft_limits_follow_a_stop_write` - a write to a physical stop
  alone moves the wall while the soft limits are off.
- `off_soft_limits_clamp_goals_to_the_physical_stops` - a Position goal
  past either stop lands the profile at the stop.
- `soft_limits_take_0_0_as_off_and_every_other_pair_by_the_rules`
  (`core/src/regions/config.rs`) - 0 / 0 is accepted in one write; every
  other pair, one side 0 among them, passes or fails the stops and order
  rules as before.
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
  folds to `yield_current_counts` no sooner than `stall_time_ticks` and within
  100 ms after it, and stays folded on every tick of the next second.
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
- `yield_fold_reaches_the_blind_band` - after a fold to an `yield_current` of
  168 the blind band holds it the same way for the whole second after
  it, mean within 1.2x.
- `virgin_blind_band_passes_to_the_window_floor` - with `r_q12` at 0
  the duty never goes under the floor and sits there pinned, at the
  floor's stall current.
- `window_floor_is_published_as_the_limiter_uses_it` - for 160 and 240
  ticks the published `window_floor_q15` is `floor_duty` of the board's
  period, and the duty a virgin stall pins at equals it.
- `window_v_floor_is_published_beside_the_current_floor` - at 64 and
  160 ticks the two published floors read 1734 and 4356 from the first
  tick, torque off.

A virgin servo (`integration/tests/class_defaults.rs`, boot seed on the
60 mohm chain):

- `virgin_openloop_run_never_collision_trips` - free runs at +/-64%, 30%
  and 0 raise no fault and leave tau_d at zero.
- `virgin_servo_stalls_at_the_class_limit` - locked at 64%, it holds at
  the 554-count class limit: peak within 1.1x, mean 0.85x to 1.0x, no
  fault inside the stall time.

## Thrown hold pins

The thrown hold (control-theory "The Position Loop", `position::Thrown`) is
pinned in gear 1. The two DES pins in `core/src/kernel/tests.rs` run the
kernel against `ThrownShaft`, a kinematic plant in the shape of the free
horn's rows after a long move: the pot rests at one of two places either
side of the goal, a drive kept up for 15 ms throws it to the other in 3 or
4 ms and it stops dead there, and undriven it stays. The loops and the
observer are the development servo's stored set, the band 4 counts, the
motor model unset so no load estimate forms. A hold hunts when the motor
is driven on more than 2% of the ticks of its last 500 ms. Paths below are
under `firmware/lib`.

- `a_hold_the_play_throws_wider_than_the_zone_comes_to_rest` - ends 18
  below and 19 above the goal: the hold parks at -18 after its ninth loss,
  208.6 ms in and 10 throws, still and undriven to the end, inside five
  bands, the last drive within 0.5 s.
- `a_thrown_hold_rests_on_the_end_inside_the_zone` - ends 22 below and 5
  above: the hold parks at +5, its first rest inside the zone once thrown,
  102.6 ms in and 5 throws.
- `a_goal_a_mode_change_and_the_run_edge_clear_a_thrown_hold` - the wide
  plant thrown, then a new goal, a mode change or torque off: the count
  reads 0.
- `a_thrown_hold_never_parks_a_loaded_shaft` - the loaded rig held in the
  band with a count of nine, the widest reach's: held and driven on every
  tick of 3 s, the held current's sign kept, the count still nine.
- `a_load_ends_the_thrown_holds_park_at_once` - the same rig with the rule's
  park set on it: never braked, held on every tick, the count 0.

The loaded and yielded holds of `integration/tests/loaded_hold.rs` and
`yield_hold.rs` run unchanged with the rule in: a loaded hold rests in the
zone with the drive on, the stall latch and the yield fold come as without
it.

The rule's state (`core/src/kernel/phases/loops/position.rs`):
`a_hold_lost_twice_is_the_plain_hold`,
`a_thrown_hold_parks_after_the_rest_and_inside_the_zone_only`,
`the_reach_grows_a_band_every_second_loss_to_five`,
`the_park_is_kept_a_band_past_its_reach_and_once_lost_clears`,
`a_loss_after_the_dwell_restarts_the_count`,
`a_creeping_shaft_with_pot_noise_is_never_at_rest`,
`clear_drops_the_count_the_rest_and_the_park`, and the load test:
`a_thrown_hold_parks_only_inside_the_friction_level` (an estimate a count
over the level never parks and the count stands; no level set, only a zero
estimate is free) and
`the_park_is_kept_inside_twice_the_friction_level_and_a_load_ends_it`.
Under three losses every output is the plain hold's: the `kernel_trace`
golden does not move.

## Identification and calibration pins

`osc-ident`, the identification, calibration and tuning tool, is pinned
in two crates, each with its own `cargo test` in CI. `measure` holds the
steps, the fits, the rules that turn a run into a constant set and the
path that installs it: its unit tests, and `measure/tests` against the
rows recorded on the bench servo (`measure/testdata`) and against a
simulated servo over the fake adapter (`measure/src/steps/testkit.rs`).
`tools/osc-ident` pins the commands themselves in its own unit tests.

## Gear 3 in detail

The hardware suite drives a flashed V006 through the osc-adapter and asserts
on the wire itself: the adapter's timer captures every edge on the bus pin
(its own TX included) and the bench decodes those edges into byte+tick stamps
host-side, so assertions run on what the wire did, not on what any UART
thinks it heard. No wlink, no chip-counter reads; the wire is the failure
surface, with one opt-in exception (`budgets`, below). The suite sweeps the
full baud matrix (0.5 M / 1 M / 2 M / 3 M).

- **budgets** (`budgets.rs`, `--ignored`) - needs the bench image
  (`scripts/build-image.sh --bench`) flashed, `BENCH_ELF` naming its ELF
  and `wlink` on the probe. Quiet, hold (with and
  without an approach move), stepping, an OpenLoop sweep, six-field TEL still
  and stepping, and
  each polled frame type at 3M and 1M run in their own window, opened by a
  reboot; the budget probe's maxima and means must stay inside
  `osc_servo_core::budget`, and every frame the host sent must book. Kernel
  ticks book by drive arm, the tick's or the one before it, Closed over
  OpenLoop over Open (the openloop window must book mostly OpenLoop), TEL
  and banking ticks too (OpenLoop's on the Closed rows, as are its event
  ticks), a tick that closed a
  tick-load window apart from all of these, and SLOW-due ticks and SLOW finish ticks as events. Each
  window prints (and with `BUDGET_OUT` saves) its rows in ticks and us and
  its raw records as it ends; the window image (`scripts/build-image.sh
  --window`, `--features bench-window`) adds gamma(k) for k = 2, 4, 10, 20,
  the ticks of 30 us or more by arm and the longest entry lateness, which
  the standard bench image skips and says so; a red run names the
  window, the component, the measured ticks and us, and the budget. The
  TEL stepping window's Closed TEL row is held as its TEL ticks' mean over
  the stepping window's plain Closed ticks of the same run, the same steps
  with no TEL: its own plain ticks are the few between its bursts and on
  the centring move, and their mean moves from run to run. The report
  prints the probe's row and that figure and says which was held; with
  `BUDGET_WINDOWS` leaving the stepping window out the row is printed and
  not held. "hold
  raw pos" (a goal from raw `pos`) is report-only. "seated" runs only when
  `BUDGET_WINDOWS` names it: with the horn clamped at mid travel it holds a
  goal past the clamp, a press at the current limit on a still shaft that
  the SLOW stages run under.

- **turnaround** (`turnaround.rs`) - instruction wire-end -> status break
  fall, per baud, for ping, read and write. With the kernel above the bus a
  reply waits out the kernel body in flight, so the means sit about one body
  above the dispatch floor (ping ~62-78 us, acked WRITE ~151-167 us, the
  rules-dominated dispatch; the production hot loop pays none of it, GWRITE
  is NOREPLY). The ceilings catch regressions; the host-facing bound is
  RESPONSE_DEADLINE.
- **rescue** (`rescue.rs`) — the §9.1 pulse reaches a servo at any baud, and
  the full field-recovery flow (rescue → prefix-walk → ASSIGN → SAVE → reboot
  → FACTORY). Both tests end with a rescue-based recovery tail so a transient
  capture dropout never strands the bench unit.
- **trim** (`trim.rs`) - the protocol sec 9.3 clock discipline on real oscillators: CAL
  trains converge the DUT's trim, the lying-train probe pins plant direction,
  and the host-detune probe (an off-catalog rate one BRR step from nominal)
  pins that traffic never trims and that CAL at a detuned host lands on the
  same anchor.
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
rescue) is the job of the `osc-bus` CLI, the bus driver (`tools/osc-bus`, built on
osc-client), and identification, calibration and tuning that of `osc-ident`
(`tools/osc-ident`), not the bench's.

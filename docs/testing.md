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

The host reads the floor from the servo (`ident/src/limits.rs`, at the
repo root):

- `servo_limits_take_the_floor_from_the_servo` - the limits carry the
  floor a telemetry read publishes, 4356 and 6534 alike, and the stop
  ladder starts on it.
- `a_servo_without_a_floor_is_refused_in_plain_words` - a published 0
  refuses with a message naming the missing sensor floor; no board
  constant stands in.

A virgin servo (`integration/tests/class_defaults.rs`, boot seed on the
60 mohm chain):

- `virgin_openloop_run_never_collision_trips` - free runs at +/-64%, 30%
  and 0 raise no fault and leave tau_d at zero.
- `virgin_servo_stalls_at_the_class_limit` - locked at 64%, it holds at
  the 335-count class limit: peak within 1.1x, mean 0.85x to 1.0x, no
  fault inside the stall time.

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

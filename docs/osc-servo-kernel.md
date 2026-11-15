# Kernel and Scheduling Architecture

The scheduling architecture of the osc servo firmware on the CH32V006: the kernel tick and its phases, interrupt priorities, the budget table and the DES (discrete-event simulator), the telemetry lane, the configuration take, DMA use and the code-shape rules. The kernel lives in `firmware/lib/core/src/kernel/`; its chip binding lives in `firmware/servo-ch32/src/runtime/`. Companion pillars: [osc-native-protocol.md](osc-native-protocol.md) (the wire), [osc-servo-transport.md](osc-servo-transport.md) (the receiver), [control-theory.md](control-theory.md) (the loops) and [driver-pattern.md](driver-pattern.md) (the layering).

Code is authoritative; when this doc and the code disagree, fix the doc.

---

## 1. Overview

One core runs four execution contexts:

| Context | Trigger | Priority | Code | Job |
| --- | --- | --- | --- | --- |
| **Kernel tick** | DMA1 CH1 TC, once per PWM period | PFIC HIGH | `lib/core/src/kernel/`, `runtime/isr.rs` | sensing, control, motor write, TEL lane |
| **Bus vectors** | TIM2, USART1, SysTick | PFIC LOW | `lib/drivers/src/bus/`, `runtime/isr.rs` | frame service, dispatch, replies |
| **Main loop** | `wfi` wake | thread mode | `runtime/run.rs` | cold work |
| **DMA** | peripheral requests | DMA priority ladder | [transport sec 2](osc-servo-transport.md#2-hardware-ledger) | scans, copies, TX, CRC feed |

The platform facts the design assumes:

| Fact | Consequence |
| --- | --- |
| One core, HCLK 48 MHz (`CLOCK_HZ`) | every budget is in HCLK ticks |
| Flash fetched one 4-byte word per 3 HCLK, no cache | code bytes and taken branches are execution time ([sec 8](#8-code-shape-on-a-fetch-bound-chip)) |
| Hardware multiply, no divide | runtime divisors are compile-time constants |
| Small RAM; the stack is the binding limit ([v006-feature-budget.md](v006-feature-budget.md#budget-lines)) | no code in RAM, no per-task stacks |

---

## 2. Static phased scheduling

### 2.1 The tick

TIM1 runs center-aligned PWM at `TICK_HZ` (20 kHz). Each period it triggers two ADC scans, at the trough and at the crest; DMA1 CH1 lands both in the control table's tick block ([sec 7.2](#72-adc-scans-land-in-the-tick-block)). The TC (transfer complete) interrupt after the crest scan is the kernel tick. No software starts or re-arms it.

> **The tick index is the kernel's only clock.**

`sample_tick` counts executed ticks; each executed tick advances the kernel by one period; nothing in the kernel reads a wall clock. A tick that does not run is counted in `tick_lost_count` ([protocol sec 5.9](osc-native-protocol.md#59-health-osc-servo)) and is not compensated. Ticks are lost only to a body over two periods, a flash SAVE (torque off) or a debug halt. A shunt burst runs no tick for its window ([protocol sec 5.8](osc-native-protocol.md#58-limits-and-the-stall-permit-osc-servo)).

### 2.2 Rates and phases

| Rate | Period | Work |
| --- | --- | --- |
| **FAST** | every tick, `TICK_HZ` = 20 kHz | window measurement, overcurrent trip, back-EMF sums, current loop, motor write |
| **MEDIUM** | `DECIM_MED` = 10 ticks (2 kHz) | outer loops, estimators, limits, detectors, publishes; one phase per tick |
| **SLOW** | `DECIM_SLOW` = 32 medium periods (62.5 Hz) | stall permit lease, undervoltage, the two heats on the period's mean of the held current squared, which LIMITS sums one sample a medium period; then the winding estimate, its cutoff (overtemperature), the winding resistance at its temperature and the shunt zero's aging; one stage per period |

Medium phases, in data-flow order (`phase` in `kernel/scheduler/table.rs`):

| Phase | Name | Work |
| --- | --- | --- |
| 0 | CONTROL | CONTROL read, enable and mode edges, drive kind; the configuration on the first tick |
| 1 | OBSERVER | observer step, speed blend on the boxcar SPEED closed |
| 2 | TRAJECTORY | trajectory and position loop; the hold updates the drive kind |
| 3 | LIMITS | limits fold, Current mode's speed fold, OpenLoop base duty, stall fault, the held current's square into the heats' sum |
| 4 | VELOCITY | velocity loop or current clamp; current reference and band land |
| 5 | RAIL | rail estimate and its fold, sensor screen, position error timer |
| 6 | SLOW | the slow block's due stage when due; its finish one period later, with the shunt zero's aging step |
| 7 | PUBLISH | the shunt zero's sample; estimates, mode, limits and sensors publish; tick-load close |
| 8 | SPARE_A | configuration take after a commit |
| 9 | SPEED | back-EMF close: the boxcar the next period's OBSERVER reads |

Module layout:

| Path | Contents |
| --- | --- |
| `kernel/scheduler/mod.rs` | `Kernel`, `on_tick`, spare-phase arbitration, `take_config` |
| `kernel/scheduler/table.rs` | rates, phase indices, health-close constants, `Medium::step` dispatch |
| `kernel/phases/` | one file per phase; `fast.rs` is the FAST half; `lifecycle.rs` holds the lifecycle state (Off, Run, Parked) and the cells each edge zeroes or reseeds |
| `kernel/phases/loops/` | loops and detectors the phases run |
| `kernel/config.rs` | `KernelConfig`, the kernel's copy of CONFIG and CALIB |

Hand-off contracts: the medium half leaves the fast half a `Command` (drive kind, current reference and band, rail clamp, the winding resistance); the FAST measurement reaches the same tick's medium phase as `Measured`. Only the CONTROL phase reads CONTROL, so a host write lands at the next period's CONTROL phase. The sample-to-command delay is fixed by the table ([control-theory.md, Why a Cascade](control-theory.md#why-a-cascade)).

The speed fold: in Current mode LIMITS folds the band under the speed ceiling after the composed limit and the endstop, on the blend OBSERVER computed two phases earlier: `fold = (ceiling - |omega_hat|) x v_kp_q88 >> 8`, saturating, 0 at and over the ceiling, one multiply and no divide (`speed_fold`). The side along the blend's sign takes `min(side, fold)`, both sides at a blend of 0, and the other keeps the band, so `lo <= 0 <= hi` holds and the fold never brakes. Every other mode passes `u16::MAX` and pays the mode test and one compare. With asymmetric non-zero sides the band no longer recovers its magnitude, so the fold returns the composed limit beside it: that is `i_lim_counts` and what `CEILING` compares against; `ENDSTOP` is set by the endstop branch itself, and `SPEED` (`limit_flags` bit 4) while the fold sits under the composed limit. With `v_kp_q88` 0 the fold reads 0 and Current mode delivers nothing. Pins: `current_mode_folds_under_the_speed_ceiling`, `the_speed_fold_never_closes_past_zero_and_leaves_the_braking_side`, `the_speed_fold_is_the_gain_on_the_speed_short_of_the_ceiling`, `current_mode_with_no_speed_gain_delivers_nothing`.

The back-EMF boxcar: FAST adds each tick's window to the open half, and SPEED closes the half on the period's last tick, after that tick's own window is in. A half is therefore the ten rows of one period, phases 0 to 9, every row in exactly one half, and the boxcar is the two periods that end at the close. The next period's OBSERVER feeds it to the speed blend two ticks later, and PUBLISH lands the same boxcar as `omega_bemf_cps`, so a published source and boxcar belong together. The rows of phases 0 and 1 belong to the half that is still open when OBSERVER runs: a command that lands at CONTROL first shows in the phase 1 row, and reaches the blend with the next period's boxcar. The close reads the winding resistance and the Ke reciprocal as they stand on its own tick, behind a take on SPARE_A and a SLOW finish of the same period. In Open nothing drives, the close voids the half on its zero duty and folds no product. Pin: `the_observer_reads_the_boxcar_speed_closed_two_ticks_ahead`.

The boxcar's validity is decided at the close (`BemfObs::close`); the fast half only marks a row that read no window. A half is valid when every row in it read both windows over their floors and the drive kept one sign through it, and a boxcar needs three valid halves in a row: its own two and the one ahead of them.

- **The start-up half.** The first rows of a drive read fast while the winding current rises, so the first valid half behind a void one closes no boxcar and is in none: the first boxcar starts at least one period of rows, and less than two, behind the drive's first valid row.
- **A drive flip.** The close compares the sign of the duty that drove the half's last row with the sign at the last close that drove. A change means the half holds rows of both signs: it is void, the halves behind it start up again, and `BemfObs::flipped` names the flip for the one period behind that close, where the speed blend turns its standing part. A sign that changes and changes back between two closes is not seen.
- **A zero duty** drives nothing and has no sign. A close on it is no flip, and the sign a later drive is compared with stays the last one that drove, so a drive that stops and resumes the same way flips nothing and one that resumes the other way flips once. The half is void: its last row had no window.

All of it runs on SPEED; FAST sums as before. Pins: `a_boxcar_waits_out_the_startup_half`, `a_half_that_holds_a_drive_flip_is_void`, `a_zero_duty_has_no_sign_to_flip`.

The velocity loop's speed (`estimator/omega_blend.rs`) is the pot observer's speed plus the boxcar's fast part, never the boxcar's level. The level is not trusted: the brush contact's drop wanders, and under a driven loop the boxcar reads over the pot. The blend keeps two states and steps them at OBSERVER, behind the observer's own step:

- **The standing part.** On a period with a boxcar the blend takes the boxcar's difference from the observer's speed, limited so that nothing in the form wraps, and the standing part follows it with a corner of 2^7 such periods. It holds across the periods without a boxcar: under a closed loop a run of boxcars lasts a few periods, and a part cleared at every gap would never form.
- **The lead.** What the difference shows beyond the standing part reaches the loop through a first-order filter of four periods, the lead, so a boxcar that comes or goes moves the loop's speed by at most a quarter of that fast part a period. With no boxcar the lead's input is zero and it runs out to exactly zero: under the window floors and at rest the loop reads the observer alone.
- **A drive flip** negates the standing part. Every volts term of the boxcar lies along the drive, so its standing error turns with it.
- **The clears.** Both states clear at the run edges and every parked period. Behind a clear the standing part is zero, so the loop sees the whole difference until the part has formed again.

On any steady pattern of boxcars the loop's mean speed is therefore the observer's, whatever the boxcar's standing error. Only the velocity loop and `omega_hat_cps` read the blend. The stall verdict reads the observer's speed, and the other limits, the park and the Ke feed-forward read no speed estimate, as before. `omega_hat_src` reads 1 on a period whose boxcar fed the blend and 0 otherwise, beside that boxcar in `omega_bemf_cps`, which the blend leaves as it closed. Pins: `a_steady_cruise_reads_the_observer_in_the_mean`, `a_validity_change_never_steps_the_loops_speed`, `without_a_boxcar_the_loop_reads_the_observer`, `the_standing_part_turns_with_the_drive`, `the_extremes_never_wrap`, `the_loops_speed_is_the_observers_plus_the_boxcars_fast_part`, `the_park_clears_the_blend`, `the_standing_part_turns_at_the_observer_behind_a_flip`, `a_cruise_holds_the_asked_speed_with_a_back_emf_that_reads_over`.

The shunt zero: every current reading subtracts the zero-current output of the sense chain, which moves with board temperature, over minutes. The tracker (`estimator/bias.rs`) keeps two zeros, driver awake and driver asleep, and takes at most one sample a control period, at PUBLISH, on that tick's trough sample. The sample is named by the bridge state the previous command left: asleep, a brake or a coast once the state has held `ZERO_SETTLE_TICKS`, or a Slow-decay drive whose brake half is at least the board's minimum; any other period feeds nothing. Each zero is a first-order filter with a time constant of 1024 periods (0.51 s). A zero that has taken no sample for `STALE_PERIODS` (205) SLOW periods, 3.3 s, is stale and moves by the other zero's steps, so a drift seen in one state carries into the other; a fresh one holds, and the pair learns the awake step. The age counts down at the SLOW finish. The FAST measurement reads the awake zero every tick. Pins: `the_zero_takes_one_sample_a_period_at_publish`, `an_unfed_zero_is_stale_after_stale_periods_of_slow`, `the_awake_step_is_learned_and_rides_the_torque_off_drift`.

ADC wide codes: the converter can stick at a code just under a multiple of 256 while its input rises through the codes it then skips, so one code stands for a band of inputs on any channel. The wide-code map (CALIB `adc_wide_mid_q4`, per board since the converter is the chip's, host-written by calibration) gives slot `k` to code `(k << 8) | 0xff` and holds half the span the converter skips after that code, Q4 counts; 0 is no wide code there, and the all-zero map changes nothing. The kernel adds the slot on every channel it consumes, where the consumer reads the sample, so a stuck reading lands mid band, not at its edge. The fast samples take whole counts, the slot rounded half up (`Shared::adc_mid`); a consumer that carries fraction bits takes the slot in Q4 (`Shared::adc_mid_q4`, or the bare slot from `Shared::adc_wide_q4`). Every other code costs one compare.

| Channel | Corrected at | Unit | Rate |
| --- | --- | --- | --- |
| drive window's shunt sample | `window::i_from_frame`, ahead of the zero | whole counts | FAST |
| drive window's two taps | `window::vdiff_from_frame`, each ahead of the subtraction | whole counts | FAST |
| trough shunt as the zero | the zero tracker's feed (`Fast::track_bias`) | Q4 | MEDIUM at PUBLISH, on a period that feeds |
| rail | `VbusEst::step` | Q4: whole counts scale as a raw code does, the fraction lands on the guard bits | MEDIUM at RAIL |
| NTC | the winding estimate (`Medium::estimate`) | whole counts | SLOW finish and the boot seed |
| pot | `phases::pos_q4`, below | Q4, through the table's local gain | MEDIUM |
| Vref | no kernel consumer: the host applies the map | - | - |

Everything downstream reads the corrected value: the loops, the overcurrent trip, the duty limiter and the stall check the current; the rail clamp, the rail fold and undervoltage the rail; the winding estimate and its cutoff the NTC. Raw stays raw: the tick block's raw fields, TEL rows, the shunt burst, the sensor screen's pot and the boot rest measurements hold the converter's codes, so a TEL row can carry a raw field beside the corrected value derived from it (`i_tick_counts`, `vdiff_counts`, `current_bias_counts`, `vbus_counts`, `t_ntc_cc`). Pins: `a_wide_code_reads_mid_band_on_every_channel`, `the_zero_the_rail_and_the_ntc_read_every_wide_code_mid_band`, `the_trip_compares_the_corrected_current`, `a_written_map_leaves_every_raw_field_alone`; an all-zero map changes no reading (`kernel_trace_matches_golden`, `a_zero_map_reads_every_window_code_raw`, `a_whole_count_tap_steps_as_its_code`).

The position measurement: the observer and its seeds read the pot through one function (`phases::pos_q4`), so they never disagree. The wide-code step is taken in raw counts ahead of the table: a code whose low eight bits are all set gains its slot of the map (`Shared::adc_wide_q4`), and a live table is read again at the corrected raw count, `lin = table(raw + map[raw >> 8] / 16)` (Q4, floored), in the interval that holds it. A wide code is the last code of its interval, so once the slot reaches a count the band's middle lies in the interval above and takes that interval's gain, not the code's own; past the table's last point the reading is the identity. On the identity it is the bare slot. A stuck reading lands mid band, not at its edge, and a goal inside the band can rest. Every other code costs one compare, and the second table read runs only on a nonzero slot. The hold band keeps the gain of the code's own interval. Pin: `wide_step_reads_the_table_where_the_band_lies`. A position table should be built from readings that already carry the step, so the table holds the pot alone. The raw sample stays raw for the sensors publish, TEL `pos` and the sensor screen.

### 2.3 Tick sequence

`runtime::isr::on_adc_dma_tc` and `Kernel::on_tick` run, in order:

1. Increment `sample_tick`; publish the scan's shunt and taps in the tick block.
2. Spare-phase arbitration: boot on the first tick, else the configuration take on SPARE_A when due ([sec 6](#6-the-configuration-take)).
3. FAST measure: the window the previous command drove, published in the tick block.
4. TEL row, while a burst streams ([sec 5](#5-cut-through-tel)).
5. The medium phase.
6. FAST drive: current loop, open loop, brake or off; motor write.
7. At most one TEL lane step.
8. Shunt-burst handshake poll; tick-load accounting.

> **The tick finishes with its scan before the next scan lands.**

The next trough scan lands mid-period and the next crest scan near the period's end, in the same block. The tick moves the trough samples out by `TROUGH_DONE` and finishes crest reads and the row's DMA arm by `FRAME_DONE` (`the_tick_reads_its_scan_before_the_next_lands`).

The current loop in step 6 takes the reading step 3 measured, with two exceptions that it takes as no reading. A window under the shunt floor reports nothing. The first window read after a command the bridge drops reads far over the winding current, and on the proportional gain that one reading steps the drive across zero. A board names the narrowest commanded width that comes out as a pulse (`KernelTiming::bridge_min_ticks`); the loop skips the first read window after a command of 1 tick up to one under it, in either direction. Zero duty is not a dropped command, so a lift from rest, the park and its wake read as before, and a board constant of 0 turns the gate off. Only the loop skips the reading: the overcurrent trip, the held current, the tick block and the observers see every read window (`the_first_reading_after_a_dropped_pulse_never_reaches_the_current_loop`).

On a tick with no reading the loop integrates nothing. It commands the reference's volts through the winding on top of the feed-forward, `r x i_ref + u_ff`, and leaves its integrator at `r x i_ref`, so a constant reference under the floor is a constant command, a reference whose volts clear the floor opens a window on its first command, and the first reading back starts from those volts (`the_command_never_integrates_under_the_shunt_floor`, `under_the_floor_the_command_is_the_reference_through_the_winding`, `the_band_under_the_floor_holds_at_most_the_floors_volts_over_r`). `r` is the winding resistance at the estimate's temperature (below), or the loop's own `i_kp_q88`, unscaled, while CALIB `r_q12` is 0; the configuration take makes that choice ([sec 6.2](#62-the-take)) and the tick loads one value from the `Command`, so it tests nothing (`an_unidentified_winding_is_commanded_through_the_loops_own_gain`, `under_the_floor_a_warm_winding_is_commanded_through_r_hat`). Under the floor the held current is that reference: the observer, the limits and the tick block's `i_tick_counts` read it there, with `tick_flags` still naming no reading. The overcurrent trip and the back-EMF sums take a reading or nothing (`the_reference_held_under_the_floor_reaches_no_trip_and_no_back_emf_sum`). OpenLoop holds its last reading through the band as before. The rule and its limits are in [control-theory](control-theory.md#the-blind-band-under-the-floor).

The winding resistance: the control path reads one resistance, CALIB `r_q12`, the winding at 25.00 C, along copper's slope to the winding estimate's temperature, `r_hat = r_q12 x (1 + ALPHA x (t_ntc + rise - 25.00 C))`, ALPHA = 2637 / 2^26 per centi-C, the rise bounded at 150 C, saturating in its u16 ([control-theory](control-theory.md#the-winding-estimate)). The law is `r_q12`'s definition (protocol sec 5.8), so a host converts a measurement to 25.00 C with the same constant. `Medium::r_hat` computes it off the tick's straight line, on the SLOW finish right after the estimate, at the boot seed and at the configuration take, so it is never stale against the estimate or a new `r_q12`, and publishes it there as `r_hat_q12`. It is the heat model's number, not a measurement: under `r_q12` on a board below 25 C and over it above, `r_q12` itself with no NTC curve and on a failed NTC, and 0 for an unidentified winding. The factor stays in 0.745..2.081 over the NTC band and the rise bound, and its correction truncates toward zero, so an identified winding never reads 0 and nothing wraps; no divide, i32 throughout.

| Consumer | Reads | Where |
| --- | --- | --- |
| back-EMF boxcar's `R x i` | `r_hat` | SPEED, at the half's close |
| current loop on a tick with no reading | `r_hat`, through the `Command` | FAST |
| OpenLoop's stall-safe base duty | the stored `r_q12` | LIMITS |

The base duty stays on the stored value on purpose: it is a ceiling, and a winding at 25 C or warmer carries at most the limit at it. On a board below 25 C it errs high by copper's slope on the difference, 4% at 15 C and 10% at 0 C, only under the window floor, where the stall current is tens of mA. Pins: `r_hat_reads_the_25_c_resistance_at_the_estimates_temperature`, `r_hat_follows_copper_on_25_c_for_every_resistance_and_temperature`, `r_hat_rises_with_a_held_current_to_the_warm_factors_value_and_falls_as_it_cools`, `r_hat_is_the_stored_resistance_with_no_heat_unset_or_on_a_failed_ntc`, `r_hat_follows_a_new_r_q12_at_the_take_and_saturates`, `the_boxcar_reads_a_warm_winding_at_the_speed_it_read_cold`, `the_stall_safe_open_loop_duty_stays_on_the_stored_resistance`.

### 2.4 Event slots

| Event | Slot | Rule |
| --- | --- | --- |
| Configuration take | first SPARE_A after a commit | owns the phase; holds no TX lane |
| SLOW block | SLOW phase, every `DECIM_SLOW` periods | the due stage, then the finish on the next period's SLOW phase (`the_slow_stages_take_one_period_each`); each holds the TX lane (`slow_stage_ticks_hold_the_tel_lane`) |
| Tick-load close | PUBLISH (`HEALTH_CLOSE_PHASE`) | never on a stage or take phase (`health_close_lands_only_on_publish_while_streaming`) |
| TEL lane step | end of tick | at most one per tick; none on a head, bank or stage tick |

Arbitration is the first `match` in `on_tick`: boot, then the take.

> **Two rare events share a tick only where the budget table prices the combination.**

### 2.5 Scheduling rules

> **The schedule is static: no run-time queue, deadline or priority comparison.**

- Dispatch is one `match` on the phase counter.
- Kernel state has one owner, the tick, and no locks.
- A tick that does not fit is fixed by deleting work, making it cheaper or moving it to a phase with room ([sec 4.5](#45-fit-is-an-outcome)).
- The module tree mirrors the schedule; structure is admitted only at zero run-time cost.

### 2.6 The main loop

`runtime/run.rs` runs the work with no deadline inside a tick, then sleeps in `wfi`: status LED, link-counter and TEL-drop publish, clock trim, shunt-burst page copy, the data job (software CRC), deferred reboot, rescue-pulse sampler.

> **The main loop masks the bus level only (`pfic::mask_bus`), never the kernel.**

### 2.7 The experiment image

One build of the same firmware, `--features experiment`, for captures where the host owns safety: the same control table, transport, TEL and shunt burst, the same bootloader path (`osc flash` keeps META and the saved configuration). The identity block sets `capability_flags` bit 1 (`CAP_EXPERIMENT`, protocol sec 5.4), which the host tools read at connect.

| Runs | Stripped (compiled out, not switched off) |
| --- | --- |
| FAST every tick: the window measurement, the tick block publish, the overcurrent trip, the motor write | the current, velocity and position loops, the trajectory generator |
| OpenLoop only: `goal_duty` clamped to `duty_max_q15`, `openloop_decay`, `openloop_zero_brake` and `drive_polarity` honoured; the `mode` byte is accepted and ignored, `mode_active` publishes OpenLoop | the duty limiter and the limits fold: peak cap, rail fold, endstop, press fold, stall verdict |
| MEDIUM: CONTROL (the read, the enable edge, the drive kind), the bus dead-man on the RAIL phase, the shunt zero's aging step every `DECIM_SLOW` periods on the SLOW phase, PUBLISH (the shunt zero's sample, `current_bias_counts`, `mode_active`, `fault_code`, `fault_flags`), the configuration take on SPARE_A; the window floors at the first tick | the estimators (fusion observer, back-EMF boxcar, speed blend, rail estimate, winding thermal), the detectors (sensor screen, position error, stall, undervoltage, overtemperature, data-state entry check, Ke belt), the position table inside the kernel, the stall permit lease |

The fault latch has two raise points. The **raw current trip** is the production overcurrent detector: `|i_meas|` over `oc_trip_counts` for `oc_trip_ticks` consecutive valid windows latches `over_current` and the drive coasts. The **bus dead-man** is `bus_deadman_ticks` (CONTROL, protocol sec 5.8): nonzero with torque on, a countdown in kernel ticks that every instruction the bus dispatches to this servo reloads (`Shared::bus_touch`, a generation counter the kernel compares once per medium period) and a streaming TEL burst holds; at zero the kernel latches `deadman` (bit 7, code 9) and the drive coasts until the torque enable ack. Torque off clears the countdown and a fresh enable starts it from the field. The loaded countdown publishes as the permit bit of `limit_flags`, the grant the burst arm reads for a seat outside the soft limits, so a seated burst arms under the dead-man as it does under the production lease. The other stripped fields publish zero; the floors publish as today, so the host tools still refuse to drive without them.

The experiment tick is a strict subset of the production tick (no estimator, no loop, no fold), so the production rows of [sec 4](#4-budgets-and-the-des) and their DES pins bound it: it carries no pin of its own. CI builds the image and runs the three binary gates on it. `scripts/build-image.sh --experiment <rev>` builds `osc-dev-v006-app-experiment`; the pins are `kernel/phases/experiment/tests.rs` (`openloop_duty_is_the_goal`, `the_trip_coasts_and_the_ack_rearms`, `the_deadman_counts_bus_silence`, `the_deadman_reloads_a_field_written_under_torque`, `the_deadman_publishes_the_permit_grant`, `the_zero_takes_one_sample_a_period_at_publish`, `an_unfed_zero_is_stale_after_stale_periods_of_slow`, `publish_is_the_truth`).

The `deadman` feature gives the production kernel the same bus dead-man, for bench work on the closed loops: the same countdown on the RAIL phase, read ahead of CONTROL and started at the enable edge, and its fault turns the drive off through the latch like any other. The stall permit lease, `limit_flags`, the limits fold and the detectors are unchanged, and no capability bit names the build, so the host writes the field itself. The image carries no budget row and no DES pin of its own: it is a bench tool, never a release, and with the feature off the production image is unchanged. CI builds it and runs the three binary gates on it. `scripts/build-image.sh --deadman <rev>` builds `osc-dev-v006-app-deadman`; the pins are `kernel/tests.rs` (`deadman::the_deadman_lapses_bus_silence_and_the_enable_edge_acks`, `deadman::the_deadman_reloads_on_a_dispatched_instruction`, `deadman::the_deadman_holds_while_a_burst_streams`, `deadman::the_deadman_takes_a_field_written_under_torque`, and `the_deadman_field_is_ignored_without_the_feature` for the default image).

The `deadman` image also carries the loop-margin stimulus, a host's way into both closed loops at one point: a triangle summed into the velocity loop's reference in Velocity mode, right after the profile sets it at TRAJECTORY, and clamped to the speed ceiling `speed_ceiling_cps`, the bound on the reference it rides, as the Velocity goal and the position loop's output are. CONTROL `inject_accel_q88` (Q8.8 c/s per medium period, read at most 0x7FFF) is its slope and `inject_half_periods` (medium periods, read at most 512, 1.95 Hz) its half-period: at both ceilings the peak's csQ16 form stays inside i32. The block is read at CONTROL beside the lifecycle block. RAIL gates it for the next period's TRAJECTORY, behind the dead-man's step: both fields are nonzero, a TEL burst streams, the dead-man is loaded, and LIMITS set no flag but PERMIT that period, so a stimulus not asked for leaves the Velocity arm the production image's, and the stimulus lives only inside a capture and a clipped period is a paused one. Its phase steps only on a period that injects. Feed-forward rides the profile's speed, not the summed reference, so neither loop sees the stimulus through it. PUBLISH writes the summed reference as TELEMETRY-EXT `omega_ref_cps` beside `theta_hat_q16` and `omega_hat_cps`; the speed loop closes on the reference less `omega_hat`, so a row of the three reads the speed loop's gain and, Velocity mode being Position mode's loop opened at that reference, the position loop's (the margin row, protocol sec 5.6). With the stimulus off the loops compute what production's do. The production image accepts both fields and ignores them, `omega_ref_cps` reads 0, and its code is unchanged: its binary differs in one byte of the CONTROL write map. The pins are `kernel/tests.rs` (`deadman::the_stimulus_rides_the_velocity_reference_as_a_triangle`, `deadman::the_stimulus_needs_a_burst_a_loaded_deadman_and_no_limit_but_the_permit`, `deadman::a_gated_period_holds_the_stimulus_phase`, `deadman::the_stimulus_is_clamped_to_the_speed_ceiling`, `deadman::the_stimulus_reads_its_fields_at_most_at_their_ceilings`, `deadman::a_stimulus_not_asked_for_leaves_the_velocity_arm_alone`, and `the_injection_fields_are_ignored_without_the_feature` for the default image) and `integration/tests/loop_margin.rs`, which reads both loops' margins on the servo of record's rig as a host reads them on the servo and holds them to the loops' linear model.

---

## 3. The kernel above the bus

### 3.1 Priority levels

PFIC (programmable fast interrupt controller) levels, set in `runtime::isr::install_irqs`:

| Vector | Level | Job |
| --- | --- | --- |
| DMA1 CH1 TC | HIGH | kernel tick; TEL lane while a burst streams |
| TIM2 | LOW | break wake ([transport sec 7](osc-servo-transport.md#7-the-dma-priority-ladder-and-receive-side-discipline)) |
| USART1 | LOW | TX arm drained |
| SysTick | LOW, taken last | deadline mux, dispatch inline |
| main loop | thread mode | cold work |

The bus vectors share LOW and never preempt each other. SysTick is taken last within LOW, so a pending wake or TX completion resolves before a pending deadline. Vector bodies are bindings in the sense of [driver-pattern sec 8](driver-pattern.md#8-hardware-demultiplexing).

> **The kernel never waits on the bus.**

### 3.2 The bus under preemption

A bus body can exceed a tick period (the `Deadline` body and every `frame` row exceed `TICK_PERIOD`), so the kernel preempts it. Contracts:

- Bus latency stretches by 1 / (1 - U), U the kernel's CPU share; a reply may pause about one kernel body between arms.
- RESPONSE_DEADLINE ([protocol sec 7](osc-native-protocol.md#7-timing-rules)) covers the stretched path (`reply_bound_covers_turnaround_under_kernel_preemption`).
- The kernel loses no tick to traffic (`kernel_on_top_loses_no_tick_under_polling`).
- The bus closes its preemption hazards ([transport sec 2](osc-servo-transport.md#2-hardware-ledger)): each TX arm clears TC before its DMA starts, frame projections read the ring cursor before the clock, and a configuration commit is bracketed by `config_seq` ([sec 6.1](#61-the-evenodd-count)).

### 3.3 Single-writer contracts

| Data | Writer | Kernel access |
| --- | --- | --- |
| CONTROL | bus | volatile reads via `region_ptr`, once per period at CONTROL; cross-field tearing accepted |
| CONFIG, CALIB | bus | only through the take ([sec 6](#6-the-configuration-take)), the ADC wide-code map aside |
| position table | bus | volatile reads (`Shared::pos_lut_q4`) |
| ADC wide-code map (CALIB `adc_wide_mid_q4`) | bus | one volatile byte read where a consumer's sample sits on a wide code, on any channel (`Shared::adc_wide_q4`); a slot is one byte, so it never tears |
| tick block | ADC DMA and tick ISR (scan, `sample_tick`), then kernel | sole writer after the landing |
| estimates, references, mode, limits, fault byte | kernel | sole writer |
| health block | chip side (ISR wrapper, main loop) | none |
| TEL channel | split per field between `TelPost` and `TelLane` ([sec 5](#5-cut-through-tel)) | kernel half |

No reference is formed into a region another context writes. Cross-context access is volatile load and store with compiler fences where order matters; the core has no atomic read-modify-write.

---

## 4. Budgets and the DES

### 4.1 The budget table

`firmware/lib/core/src/budget/mod.rs`, in HCLK ticks; `TICK_PERIOD` = 2400.

| Rows | Scope |
| --- | --- |
| `KERNEL`, `KERNEL_MEAN` | plain tick body by regime (Open, Closed, OpenLoop) and phase, maximum and mean, every phase's stages and the tick block's work in it |
| `REFRESH`, `SLOW_DUE`, `SLOW_FINISH`, `HEALTH_CLOSE`, `CATCH_UP` | events on top of a tick |
| `R_AT_TEMPERATURE` | estimated addition inside `SLOW_FINISH`, the resistance read at the estimate's temperature, until the probe refills the finish |
| `SPEED_FOLD` | estimated addition on the LIMITS column of `KERNEL` and `KERNEL_MEAN`, every regime, Current mode's speed fold, until the probe refills the rows |
| `TEL_ROW`, `TEL_BANK`, `TEL_HEAD`, `TEL_POLL`, `TEL_FEED`, `TEL_ARM`, `TEL_PATCH`, `TEL_LANE_MEAN` | TEL row and lane steps |
| `LAYOUT` | layout allowance on every kernel row ([sec 4.2](#42-the-layout-allowance)) |
| `ENTRY_EXIT`, `ENTRY` | interrupt entry and exit |
| `TROUGH_DONE`, `FRAME_DONE`, `NEXT_TROUGH`, `NEXT_CREST` | scan deadlines ([sec 2.3](#23-tick-sequence)) |
| `body`, `body_mean`, `frame`, `frame_mean`, `CONFIG_COUNT` | bus vector bodies and frame types at `BUDGET_BAUD_HZ` (3 Mbaud) |

Users:

- the bench probe (`budget/probe.rs`) records the same components;
- the hardware test `budgets` fails a window whose maximum or mean exceeds its row;
- the DES charges the rows as its costs;
- static pins sum the rows a tick can carry against `TICK_PERIOD` (`the_take_tick_fits_at_budget_without_tel`).

Row sources:

| Kind | Value |
| --- | --- |
| **M** (measured) | probe maximum over runs plus 3%, at least 1 us; mean plus 5% |
| **E** (estimated) | named instrument (usually the release disassembly priced with the fetch model) plus stated headroom |

### 4.2 The layout allowance

`LAYOUT` (35 HCLK) sits on every kernel row, maximum and mean, and covers register-allocation and block-placement shifts between builds and between the bench and release images.

> **The allowance counts against every hard budget:** the tick period, the turnaround deadline, every DES pin at budget and the stack reserve.

### 4.3 Row changes

> **A row changes only with its evidence, in the same change.**

- A raised M row comes from the probe on the production image.
- A raised E row names its new instrument.
- The DES pins re-run against the new table.
- The PR states old and new value, the measurement and the DES result.

A red bench window or DES pin is a finding, resolved by a root cause or a redesign, never by a row raise alone. Rows also go down when a refill measures them lower.

### 4.4 DES pins

The DES (`firmware/lib/integration`, kernel lane in `sim/cpu.rs`) runs the real bus driver and dispatch against simulated providers, with the kernel lane charged from the budget table.

> **Gating pins run at budget:** every body at its maximum with `LAYOUT` and `ENTRY_EXIT`, every injected backlog or lag at the worst the table allows.

- A pin is never moved to typical mode, given a smaller backlog or a wider tolerance to pass.
- The typical lane (bodies drawn below their maxima around the mean rows) is report-only.
- The DES must predict no less than the bench: `step_soak_at_budget_drops_at_least_the_bench`, `window_load_at_budget_is_never_below_bench`.

[testing.md](testing.md) places the DES among the three test gears.

### 4.5 Fit is an outcome

> **A design that does not fit at budget changes; the pin stays.**

Remedies: ask for less (fields, rate, batch), make the work cheaper, or change the mechanism. Examples:

- TEL rows are at most 12 bytes because a 16-row frame of 13-byte rows does not clear the 3 Mbaud wire within its ticks (const assert in `firmware/lib/core/src/tel.rs`, `twelve_byte_rows_fit_the_three_megabaud_batch`).
- A burst waits for a pending take because the take tick has no room for a row ([sec 6.3](#63-tel-waits-for-a-pending-take)).

### 4.6 Health

The tick ISR is stamped at entry and exit (`runtime/tick_load.rs`); the health block reports mean load, overruns and lost ticks ([protocol sec 5.9](osc-native-protocol.md#59-health-osc-servo)). Every build carries it.

---

## 5. Cut-through TEL

`firmware/lib/drivers/src/tel/`; wire format in [protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts).

| Part | File | Side |
| --- | --- | --- |
| `TelChannel` | `tel/mod.rs` | shared storage: two frame images, arm mailbox, drop count |
| `TelPost` | `tel/post.rs` | bus: arm, lend the lane, take it back |
| `TelLane` | `tel/lane.rs`, `tel/rows.rs` | kernel: rows, frame heads, CRC, TX |

> **While a burst streams, the kernel tick owns the TX lane.**

The bus lends the lane once the arming ack is out (or at the arm when no ack follows) and takes it back in its first body after the LAST frame leaves the wire. No TX interrupt and no bus body runs for the burst.

Mechanism:

- One row per tick, 16 rows per frame, each copied once to its final wire offset in one of two frame images ([sec 7.3](#73-the-two-chunk-tel-row)).
- Cut-through: a frame breaks once its lead rows have landed, and its DMA reads later rows as the wire reaches them. The lead is the frame's wire time plus `LEAD_MARGIN_TICKS` (`tel_kernel_lane_never_underruns_at_budget`).
- The lane feeds the CRC engine as rows land and patches the CRC ahead of the DMA ([sec 7.4](#74-the-crc-beside-tx)).
- Ticks lost mid-frame fail that frame's CRC and leave a sequence hole; the burst runs on to LAST (`tel_lost_ticks_mid_frame_fail_the_crc_and_leave_a_seq_hole`).

Lane rules:

- At most one step per tick, at the end of the tick: frame start, CRC feed, DMA arm or CRC patch. Each has its own budget row.
- No step on a tick that lands a head, banks a frame or runs a slow-rate stage (`tel_lane_never_steps_on_a_stage_tick`).
- The producer never blocks: a row that finds both images waiting for the wire is dropped and counted in `tel_drop_count`.

---

## 6. The configuration take

CONTROL is read field by field. CONFIG and CALIB reach the kernel only as a whole committed set, through its copy `KernelConfig` (`kernel/config.rs`). The one exception is the ADC wide-code map ([sec 2.2](#22-rates-and-phases)), read a byte at a time, only where a consumed sample of any channel sits on a wide code: copying it would lengthen every take for a value the steady tick never loads.

### 6.1 The even/odd count

`Shared::config_seq` (`lib/core/src/shared.rs`) brackets every commit that reaches the span `SPAN_START..SPAN_END`:

| Call | Count | Fence |
| --- | --- | --- |
| `config_commit_begin` | odd, before the commit's first store | Release after |
| `config_commit_end` | even, after the commit's last store | Release before |

An even count names one whole committed set. The bus is the only writer; the pair is the `CONFIG_COUNT` row.

### 6.2 The take

On SPARE_A, `config_due` compares the count with the one last taken:

| Count | Action |
| --- | --- |
| even, not yet taken | `Kernel::take_config`: one word copy of the span (offsets const-asserted against the table), then the derived values, the winding resistance on the new `r_q12` among them, and the two speeds resolved in the copy: a ceiling of 0 reads as the move speed, a move speed of 0 as the ceiling, and the move speed runs at most the ceiling, so no phase tests either field |
| odd | keep the current set; check again at the next SPARE_A |
| equal to the last taken | nothing |

> **The take is a CPU copy inside one tick.** The kernel preempts the bus, so no commit store lands during the copy.

A write takes effect within one medium period, or one more when the take's tick lands inside the next commit (`config_apply_lands_within_its_bound_at_budget`, `dispatched_config_write_lands_at_the_take`). The first tick takes the configuration unconditionally.

### 6.3 TEL waits for a pending take

> **A take and a TEL row never share a tick** (`the_take_tick_fits_at_budget_without_tel`).

- A burst's first row waits while the count differs from the last taken, and never starts on SPARE_A; the check runs only at a burst start.
- The host is silent through a burst, so no take arrives mid-burst.
- A burst armed in the same COMMIT as a configuration write starts up to a medium period and a tick late (`a_burst_never_streams_through_a_config_take`).
- A take does not hold the TX lane; a slow-rate stage does.

---

## 7. DMA as a second execution unit

### 7.1 Rules

DMA runs in parallel with the CPU. On hot paths (the tick, the bus frame, the TEL burst):

- Large copies go to DMA, one arm per contiguous run; the control table may be reordered to make runs contiguous.
- After an arm, the CPU does independent work before anything waits on the result.
- Completion is polled, not interrupted, where possible.
- A copy below the arm cost stays on the CPU; DMA cycles also stall CPU fetch on the AHB (the internal bus).
- Cold paths use a plain small copy.

> **Never queue two arms on one channel.**

Kernel-side channels (full map and priority ladder in [transport sec 2](osc-servo-transport.md#2-hardware-ledger) and [transport sec 7](osc-servo-transport.md#7-the-dma-priority-ladder-and-receive-side-discipline)):

| Channel | Moves | Kernel use |
| --- | --- | --- |
| DMA1 CH1 | ADC scans to the tick block | sensing; its TC is the tick |
| DMA1 CH6 | memory to memory | TEL row's longer chunk (reply snapshots outside bursts) |
| DMA1 CH3 | halfwords to SPI1 | CRC feed for TEL frames |
| DMA1 CH4 | frame image to USART1 | TEL frame TX |

### 7.2 ADC scans land in the tick block

DMA1 CH1 writes both scans of each PWM period into the tick block, laid out in scan order so the crest scan's slots from `pos` on are the published fields ([protocol sec 5.6](osc-native-protocol.md#56-telemetry-stream-tel-bursts)). The terminal taps land in sample order, so the scans' shunt and tap slots are unnamed: the tick moves the trough samples out of the next scan's way, copies the crest's shunt, writes each tap under its terminal's name and stamps `sample_tick`. A bus read never sees a tap under the other terminal's name. Bus reads and TEL rows copy from the same block.

### 7.3 The two-chunk TEL row

`RowPlan` (`tel/rows.rs`), built by the bus at the arm:

| Chunk | Mover | Bound |
| --- | --- | --- |
| longer | DMA1 CH6, one memory-to-memory arm | at most the whole row |
| shorter | CPU, straight-line halfword stores | at most half a row, three halfwords |

The CPU copies its chunk, then arms CH6; the DMA copy runs during the medium phase and the drive. One arm per row, landed before the next tick's entry, so CH6 is idle at every arm (`the_tick_reads_its_scan_before_the_next_lands`). CH6 is the kernel's for the burst; no reply snapshots are taken while the lane is lent.

### 7.4 The CRC beside TX

SPI1 is the CRC engine ([protocol sec 3.2](osc-native-protocol.md#32-osc-crc-16)), fed by DMA1 CH3; the CPU computes no CRC on a hot path.

| Path | Feed | Result |
| --- | --- | --- |
| reply | per TX arm, during transmission | patched into the last arm before the DMA reaches it ([transport sec 4](osc-servo-transport.md#4-why-this-is-fast--first-principles)) |
| TEL frame | as rows land | patched into the image ahead of the wire ([sec 5](#5-cut-through-tel)) |
| receive | from the covered checkpoint | polled at the verdict ([transport sec 4](osc-servo-transport.md#4-why-this-is-fast--first-principles)) |

---

## 8. Code shape on a fetch-bound chip

The facts behind these rules, the layout traps and how to measure a change are in [v006-performance.md](v006-performance.md).

### 8.1 The fetch model

Flash is fetched one aligned 4-byte word per 3 HCLK at 48 MHz, with no cache. Every taken branch, call and return restarts the fetch; a branch not taken costs its bytes. Budget E rows are priced with this model.

### 8.2 Lean code

> **Code bytes and taken branches are execution time.**

- Delete before adding: fewer conditionals and special cases, one general path.
- The hot path is the fall-through side of its branches; small hot bodies are branchless.
- Small hot helpers are inlined.
- Constants are folded at compile time (`KernelTiming`); the kernel never divides at run time.
- A provably redundant check on a hot path may be removed with its safety argument and a test pinning the invariant. Firmware never panics.
- No dynamic dispatch and no run-time-costly layers on hot paths.

### 8.3 Hot function alignment

> **Every out-of-line hot function carries `#[rustc_align(4)]` and is listed in `scripts/check-hot-align.sh`.**

Hot means the tick interrupt and its callees, the bus vector bodies, the bus frame path and the TEL lane. The pin applies whether or not the function is aligned today; new hot code adds the attribute and the script entry in the same change. The board app builds with `-align-all-nofallthru-blocks=2` (the `osc-dev-v006-app` profile in `firmware/boards/osc-dev-v006/Cargo.toml`), which does not cover function entries. The four interrupt bodies share one `.trap.rust` section and each carries its own pin.

### 8.4 Guards

| Rule | Guard |
| --- | --- |
| Hot entries at 0 mod 4 | `scripts/check-hot-align.sh`, CI, release and bench images |
| No soft div, mod or mul, no 64-bit arithmetic | `.github/check-soft-arith.sh`, CI |
| No `memcpy`, `memset`, `memcmp`, `memmove` on runtime paths; bringup statics only, through `firmware/servo-ch32/src/runtime/mem.rs` | `.github/check-no-mem.sh`, CI, release and bench images |
| No code in RAM | review |

Replacements for `mem*` builtins:

- small fixed-size copies: typed word loads and stores, or an inlined copy not lowered to a call;
- zero fills fully overwritten before use: `MaybeUninit`;
- the rest: a dedicated routine sized for its alignment and length, pinned as a hot function;
- implicit calls (struct moves, array zeroing, `copy_from_slice`, `fill`, `==` on arrays) count too.

### 8.5 Nothing runs from RAM

> **All code runs from flash.** RAM is reserved for data and stack; a slow body is made smaller.

---

## 9. Trade-offs to accept

- **Elastic replies.** Reply latency follows kernel load; a turnaround measured at rest does not hold while moving ([transport sec 10](osc-servo-transport.md#10-measured-turnarounds)).
- **Tickless windows.** A shunt burst runs no tick ([control-theory.md, What It Does Not Cover](control-theory.md#what-it-does-not-cover)); a flash SAVE or debug halt stalls the core. SAVE and halts count in `tick_lost_count`; the burst counts nowhere.

---

## 10. Summary

1. **One tick per PWM period, started by hardware.** The tick index is the only clock; lost ticks are counted, never compensated.
2. **The schedule is a static table.** Three rates, ten medium phases in data-flow order, a fixed slot for every rare event.
3. **The kernel is above the bus.** The kernel never waits on the bus; bus latency stretches instead.
4. **Every body has a budget row.** The bench holds the chip to it; the DES charges it with every gating pin at budget, layout allowance included.
5. **Rows move only on evidence; fit is an outcome.** A raise needs a production probe and a DES re-run in the same change.
6. **The kernel owns the TX lane during a TEL burst.** Rows are cut-through, one lane step per tick at most, and the producer never blocks.
7. **Configuration reaches the tick whole.** `config_seq` brackets each commit, the take runs at SPARE_A, and a burst never starts while a take is pending.
8. **DMA is a second execution unit.** One arm per channel at a time; the CPU works beside each arm.
9. **Code size is execution time.** Hot entries at 0 mod 4, no soft arithmetic or `mem*` builtins on runtime paths, no code in RAM.

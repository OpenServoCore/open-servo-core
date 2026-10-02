# Terminal voltage floor on osc-dev-v006 rev 2A

Status: design comparison, nothing built. It compares three ways to close the gap
between the current floor and the terminal voltage floor, recommends one, and names
the bench run that decides it.

## The problem

Every current and terminal reading is gated on a minimum drive window, in TIM1 ticks of
half window (`i_window_min_ticks`, `v_window_min_ticks`, `estimator/window.rs`). ARR is
1200, so 12 ticks is 1% duty. On rev 2A the two floors are:

| floor | ticks | duty | what sets it |
|---|---|---|---|
| current | 64 | 5.3% | the amplifier's settle after the ON edge (3% tier) |
| terminal | 160 | 13.3% | tap B's sample-and-hold closes 135 ticks after the crest |

The geometry behind the terminal floor is fixed by the chip. One ADC converts
`[shunt, vmA, vmB, pos, vcal, vbus, ntc]`, triggered by TIM1 TRGO = UPDATE, so one scan
lands at the counter crest and one at the trough. Each slot is 26 ADCCLK at 24 MHz, which
is 52 timer ticks, and the S/H closes 31, 83 and 135 ticks after the crest for the shunt,
tap A and tap B. The aperture can not shrink. At a 3.3 V supply the V006 keeps the
conversion rate at or under 1 MHz (DS Table 3-23, RM sec 9.3), and 13.5 cycles is already
the shortest aperture that holds it.

The cost shows on the MG90 on 2S. It turns 600 to 1200 linearized counts per second at
8 to 11% duty, 96 to 136 ticks. That is above the current floor and under the terminal
floor, so the back-EMF boxcar is void on every half and the velocity loop runs on the pot
observer (notebook 11 sec 5). The back-EMF boxcar scatters by 0.62% against a commutation
ripple reference where a pot derivative scatters by 5.4% (control-theory, velocity
feedback). So the low speed band runs on the noisy source. The 160 tick floor also
squeezes the resistance ladder on 2S, which plans from the larger of the two floors and
gets only 13.3% to 15.5% duty under the current limit.

One caution before the candidates. The `osc ident verify` velocity legs read 3.4 to 4.7%
slow in this band, but notebook 11 shows the loop was on the pot observer there, and the
observer should track a steady ramp with no error. The cause of that 4% is still open.
Closing the voltage gap moves those legs onto the back-EMF. It is not shown to fix the 4%.

### What any route can deliver at low duty

The back-EMF is $E = v_{mean} - R\,i$. At low duty $R\,i$ is most of $v_{mean}$, so
errors in $i$ and $R$ get amplified, whatever reads the volts. For the MG90 on 2S (rail
7.38 V, 78 to 100 mA free running, $R$ = 4.56 ohm, notebook 11):

| duty | $D \cdot V$ | $R\,i$ | $E$ | 1% of $i$ or $R$, in speed | 1 count of vdiff (4.03 mV), in speed |
|---|---|---|---|---|---|
| 8% | 0.59 V | 0.37 V | 0.23 V | 1.6% | 1.8% |
| 10% | 0.74 V | 0.38 V | 0.34 V | 1.1% | 1.2% |
| 15% | 1.11 V | 0.40 V | 0.71 V | 0.6% | 0.6% |

So a useful back-EMF speed on this motor at 2S starts around 7 to 8% duty for every
candidate below. Under that, a few percent of winding heat in $R$ is already tens of
percent of speed. Getting the voltage valid down to the current floor still matters for
the resistance ladder (a stall has $E = 0$) and for the winding thermometer.

## Timing model

All candidates are scored against the same window model, in ticks from the crest, with the
commanded drive window at $[-h, +h]$:

- The chopping terminal rises about 36 ticks after the ON compare (dead time 24, propagation
  and rise, burst fold in notebook 10 sec 8) and falls 8.5 to 9 ticks after the OFF compare.
  The 100 pF tap adds a 6.4 tick time constant. So a tap sample is good from $-h + 67$
  (1% of the swing) or $-h + 90$ (1/2 LSB) up to about $h - 5$. The shipping floor keeps 25
  ticks of margin on the trailing side (160 against tap B's 135).
- In slow decay only one terminal chops. Forward drive chops A, reverse chops B, and the
  other terminal sits low through both phases, so it can be sampled anywhere.
- The shunt sample is good from $-h + 95$ at the 64 tick floor's accuracy, $-h + 156$ for 1%
  raw, $-h + 91$ (grid) or $-h + 103..115$ (MG90) for 1% with the settle gain.
- The winding current ramps through the ON phase. At low duty the slope is about
  $V/L$ = 0.21 counts per tick on 2S (L = 0.73 mH), and the sample equals the period mean
  only near +40 ticks (the real pulse's center plus the amplifier delay). Every tick a shunt
  sample sits away from that instant is 0.21 counts of error on a 70 to 110 count current.

As a check, with the shipping crest trigger the model puts the tap B cutoff at 130 ticks.
The grid ladder measured about 126.

## Candidate 1: averaging taps

Swap Cv1 and Cv2 from 100 pF to 100 nF (X7R 0603, LCSC C14663, the part already fitted as
Cb2). Keep Rv1 to Rv4. The tap time constant becomes 128 us (pole 1.24 kHz), and the tap
carries the terminal's period mean with a small ripple on it.

A single crest sample of that tap is not the mean. It sits mid-ramp and reads high by about
$25 \times (1 - D)$ counts on tap A, up to 38 counts on tap B, and the X7R tolerance moves
that by several counts. The estimator that works weights the crest and trough samples of
the same scan pair by drive and OFF ticks:

$$v_{mean} = \frac{t \cdot v_{crest} + (\mathrm{ARR} - t) \cdot v_{trough}}{\mathrm{ARR}}$$

The trough sample sits on the falling ramp with the opposite first-order error, so the pair
cancels it for any time constant. What is left is second order, at most 1.7 counts at
100 nF and about one count from 10% duty up. Both scans already convert both taps every
period, so this needs no scan change. The same formula is right on the shipping 100 pF taps as
well, where the trough term carries the brake drop ($-2 R_{ds} i$, about 16 mV here).

The tables behind these numbers are in the averaging tap study in the bringup repo,
`docs/rev2a-sense/study28-averaging-taps.md`.

- **Floors.** v floor 0. The back-EMF boxcar becomes valid wherever the current is, so from
  64 ticks.
- **Accuracy at 5 to 15%.** Residual at most 1.7 counts at 5.3% (1.7% of the 99 count
  mean), 0.8 counts at 10% (0.4%), 1.0 counts at 13 to 15%. The average also measures the
  real volt-seconds, so the ON pulse coming out 10 to 16 ticks short (notebook 11 sec 4) is
  inside the reading. The crest model can not see that loss, and notebook 11 measures it as
  most of the 2 to 4% over-read from 10 to 15% duty.
- **Loop phase.** 128 us of lag on the voltage only, 1.2 deg at 25 Hz and 9.1 deg at 200 Hz,
  on top of the 1 ms boxcar's 500 us (4.5 and 36 deg). Through a duty step the voltage lags
  the current, so the first boxcar after a step carries about 13% of the step's own
  $R\,\Delta i$. A matched IIR on the current would cancel it, but only if the bench shows
  the velocity loop cares.
- **Rework list.** Firmware: the weighted numerator in `window.rs`, `bemf.rs` and the
  thermometer in `kernel/medium.rs`, the integration plant's trough terminals, the board's
  v floor. Protocol: TEL `vdiff` (protocol sec 5.6) and the ident aggregate's `vdiff_mean`
  (protocol sec 5.10) become the period mean, so hosts drop their `duty x vdiff`. Host: the
  ident burst routes lose the terminal edges. `ident/src/exp/winding.rs` and `wavefit.rs`
  have to place edges from the timer phase and take V_on from the rail tap (VSNS keeps its
  100 pF) less $R_{ds} i$. That is the biggest single item. The coast routes in notebook 03
  skip the first 1 ms (8 time constants) after coast entry.
- **Board discriminator.** A 100 nF board and a 100 pF board need different v floors and
  different notebook entries. If every rev 2A board gets the swap and the schematic changes,
  the board crate carries the floor alone. Either way the capture meta.json `sense` block
  needs a tap kind (or `v_window_min_ticks`) so `oscnb.boards.from_meta` can tell datasets
  apart.
- **Size and risk.** Two 0603 parts per board. Around 150 lines of firmware with tests, a
  protocol meaning change, and a few hundred lines of host rework that only touch
  identification. The fast path and the scan code stay as they are. The risks are the lag
  through steps, the X7R part's DC bias and tolerance (they move the residual, not the mean),
  and fast decay, where the OFF level is not flat and the weighted pair leaves 10 to 30
  counts. Closed loop always runs slow decay, so that only reaches OpenLoop fast-decay
  telemetry.

## Candidate 2: a movable scan

### What the chip offers

From the CH32V00X RM:

- The regular group's trigger (EXTSEL, RM Table 9-3) is one of TIM1 TRGO, TIM1 CC1, TIM1
  CC2, TIM2 TRGO, TIM2 CC1, TIM2 CC2, a pin or software. Only the rising edge starts a
  conversion (RM sec 9.2.3). TIM2 belongs to the bus break wake. TIM1 CC2 is the IN2 bridge
  channel.
- The injected group's trigger (JEXTSEL, RM Table 9-4) is a CC event of TIM1, TIM2 or TIM3,
  never a TRGO.
- TIM1 MMS (CTLR2) offers reset, enable, update, the CC1 compare pulse, and OC1REF to OC4REF
  as level outputs.
- In center-aligned mode 3, the compare flag of an output channel is set on both the up and
  the down crossing (CTLR1 CMS). Modes 1 and 2 set it on one direction only.

The scan trigger options that follow:

| route | scans per period | where |
|---|---|---|
| MMS = UPDATE (shipping) | 2 | crest and trough, fixed |
| MMS = OC4REF, CH4 in PWM mode | 1 | anywhere, 20.8 ns steps |
| MMS = OC4REF, CH4 in toggle mode | 1 | REF toggles twice, so it rises once |
| EXTSEL = TIM1 CC1 (or MMS = CC1 pulse), CMS = 3 | 2 | mirrored about the crest or about the trough |
| EXTSEL = TIM1 CC1 with CCR1 alternated per half | 2 | anywhere in each half |

CH4's compare unit is free (its pin PC7 is the STAT lamp as plain GPIO, CC4E off), and the
OC4REF trigger was proven on rev B, zero stalls in 288 arms. PWM mode 2 with
CCR4 = ARR - X fires X ticks before the crest.

A per-half CCR4 change does not give two triggers. In either PWM mode OC4REF compares CNT
against CCR4, which is monotonic in each half, so REF makes one pulse and one rising edge
per period. A second rising edge needs the PWM mode itself switched in the middle of the
period, which is a CPU write against the counter.

The mirrored CC1 pair is no help on its own, both scans sit the same distance from the same
extremum. The last row is the only hardware-timed way to keep a trough scan next to a
pre-crest one. TIM1's CC1 DMA request moves to the update event with CCDS = 1, it lives on
DMA1 CH2 (RM Table 8-2), and CH2 is unused in this firmware. A two-entry circular buffer
would write CCR1 at every crest and trough. It is untested, it adds a third DMA stream whose
phase (which value lands in which half) has to be identified at every arm, the same class of
hazard the scan geometry and the burst restore already carry, and CH1's pin under Remap8 is
PC4, the VSNS input, so CC1E must never be set.

### Slot order and floors

One scan per period, triggered T ticks from the crest, slots 52 ticks apart. The two
margin sets come from the window model. "Policy" is 1/2 LSB lead and the shipping 25 tick
trailing margin, "physical" is 1% lead and the measured trailing edge.

| order | T | v floor | i floor | shunt sample | ripple error at 10% duty |
|---|---|---|---|---|---|
| shunt, A, B, crest trigger (shipping) | 0 | 160 / 130 | 64 | +31 | -2 counts |
| A, B, shunt, crest trigger | 0 | 108 / 78 | 135 or more | +135 | +20 counts |
| A, B, shunt, pre-crest | -38 to -50 | 97 / 86 | 97 / 85 | +85 to +97 | +9 to +12 counts |
| A, shunt, B, pre-crest | -47 to -50 | 110 / 83 | 59 to 62 | +33 to +36 | -1 count |

The taps-first order with the shunt last makes the two floors meet near 85 to 97 ticks, but
it does it by putting the shunt at the trailing end of the window. There it reads the
current's ramp 45 to 57 ticks past the mean, 9 to 12 counts high on a 70 to 110 count
current. At 10% duty that is 37 to 49 mV of $R\,i$ against an $E$ of 0.34 V, so the
back-EMF speed reads 11 to 14% low. The correction needs the motor's $L$, a per motor
constant, and the current floor rises from 64 to about 85 on top. That order is out.

The shunt-centered order `[A, shunt, B]` keeps the shunt where the shipping scan has it
(at +33 to +36, within a tick or two of the same ripple error) and lets the taps straddle
the crest. Forward reads A 17 ticks before the crest, reverse reads B 87 ticks after it.
The v floor lands at 83 ticks with physical margins and 110 with the shipping margin
policy, 6.9% to 9.2% duty. The current floor stays where it is. The forward floor rides the driver's turn-on delay
(500 ns dead time, typical only in the DRV8212P datasheet) and the tap's leading settle,
which no capture has measured to the 1/2 LSB level yet.

### Does it fall under the late-window rejection

Late-window sampling was rejected as architecture, as a permanent timing workaround. There the
current sample moves late in the window to give a slow amplifier more settle time. The
shunt-last order is exactly that, and the ripple error above is a second reason. The
shunt-centered order is not. Its shunt sample stays at the window's center, and the taps
move to where the terminal is valid. It is a fixed scan geometry like the shipping crest trigger,
with no per-duty or per-unit timing. It does make the forward voltage depend on the bridge's
turn-on timing, which post-crest taps never see.

### What moves with the trough scan

With one scan per period the trough scan is gone, unless the CC1 alternation above works.
Its consumers in the code:

- `estimator/window.rs`: `use_trough` for fast decay, `trough_is_brake`, and the trough
  fields in `i_from_frame` and `vdiff_from_frame`. Fast decay puts the window at the trough,
  so the trigger phase has to follow the decay mode. That is an OCM and CCR4 rewrite on every
  decay change, with the spurious first rising edge of PWM mode 1 to guard against.
- `kernel/fast.rs`: the bias tracker learns the in-drive zero from the brake trough. Without
  it the tracker learns only during commanded brake or coast, and holds through any drive.
- `SensorFrame` (`current_trough`, `vmotor_a_trough`, `vmotor_b_trough`),
  `servo-ch32/src/control/sensors/mod.rs`, `scan.rs` (buffer length, offsets, TC drain
  budget) and `control/burst.rs` restore, which re-arms MMS and decides the scan geometry
  with an ordering that is already load-bearing.
- `kernel/medium.rs` publish of `current_trough` (telemetry 0x250), `tel.rs`
  `BIT_CURRENT_TROUGH` and `TelSample`, protocol sec 5.6, and the kernel tests.
- `lib/integration/src/plant.rs` and the sim's TEL path.
- Hosts: `ident/src/frame.rs` and `regs.rs`, `tools/osc/src/rig/csvio.rs`, `sweep.rs` CSV
  columns, `capture/check.rs`, `capture/verdict.rs` (its fast-decay current is the trough),
  `client/web` telemetry types, and the notebooks that read `current_trough` (00 to 08, 11).

### Scores

- **Accuracy at 5 to 15%.** The current is unchanged. The voltage keeps the crest model,
  $D \cdot v_{on}$, with its blind spot at the pulse edges. Notebook 11 measures that model,
  with the identified constants, within 4.4% of the ripple from 10 to 30% and +1.3% at the
  10% forward rung. Its non-scaling part is +25 to +49 mV, which grows as $1/E$ toward the
  floor, so the 7 to 10% band would want notebook 11's fixed edge loss in firmware as well, a
  constant that moves with the current.
- **Loop phase.** None. The sample moves about 1 us earlier.
- **Size and risk.** Around 300 to 500 lines across the timer HAL, scan, burst restore,
  window, kernel, protocol and five host tools, all of it in the timing-critical acquisition
  path that carries the most bench scars (the stale OC4REF phase that sampled brake in both
  halves, the rotated-frame restore). No hardware.

## Candidate 3: hybrid

Candidate 1's averaging taps and weighted estimator, plus the current path's settle gain
(a board table of 21 Q15 entries indexed by drive ticks, the inverse of the amplifier's
step response at the crest sample), with no scan change.

- **Floors.** v floor 0, i floor 64. The back-EMF is valid from 64 ticks.
- **Accuracy at 5 to 15%.** Voltage as candidate 1. With the gain, the grid current lands
  within 0.16% from 60 ticks, and the MG90 within about 1% from 72 ticks, +2.4% at 60 where
  the motor's own edge charge adds counts no gain can remove. At low duty the current is the
  bigger lever on speed (the table at the top), so this is the candidate that makes 7 to 10%
  duty honest end to end. At 10% that is roughly 1% of speed from the voltage residual and
  0.3% from the current, plus whatever the winding temperature does to $R$.
- **Loop phase, rework, risk.** Candidate 1's, plus 21 table entries and one multiply on the
  fast path for the gain (+182 B of text). The gain's truth still waits on a meter
  in series with the motor.

## Side by side

| | 1. averaging taps | 2. movable scan, `[A, shunt, B]` | 3. hybrid |
|---|---|---|---|
| v floor | 0 | 83 to 110 | 0 |
| i floor | 64 | 59 to 62 | 64 |
| back-EMF valid from | 64 (5.3%) | 83 to 110 (6.9 to 9.2%) | 64 (5.3%) |
| voltage at 8 to 15% | period mean, at most 1.7 counts off, edges included | crest model, edges missed | as 1 |
| current at 64 to 125 ticks | raw, 1 to 3% low | raw, 1 to 3% low | within about 1% from 72 |
| loop phase | +1.2 deg at 25 Hz | none | as 1 |
| trough scan | kept | lost, or an untested DMA trick | kept |
| change | 2 caps, ~150 firmware lines, host burst rework | ~300 to 500 lines in the acquisition path | as 1, plus the gain table |
| proven so far | notebook 11's edge loss, coast chain agreement, model study | OC4REF trigger on rev B, tap timing | as 1, plus the gain's self-consistency on the bench |
| needs a bench run | tau, residual, lag, MG90 speed | tap lead settle, trough migration | as 1, plus the series meter |

## Recommendation

Candidate 3. It removes the voltage floor outright instead of moving it to 83 to 110 ticks,
it measures the edge volt-seconds the crest model can not see, it leaves the scan geometry
and every trough consumer alone, and its firmware change is arithmetic on data the scans
already convert. Its real costs are 128 us of voltage lag and the identification burst
rework on the host. Candidate 2 stays as the fallback if the averaging tap fails on the
bench, in the shunt-centered order only.

The firmware estimator goes first, on the shipping 100 pF taps, with the v floor still 160. It
adds the brake term notebook 11 sizes and gives the bench the period mean in telemetry. The
board swap comes after.

### The bench run that decides it

Board #1 with Cv1 and Cv2 swapped to 100 nF, 2S, the estimator build with the v floor set to
0 for the run:

1. **Grid ladder against a meter.** 3.7 ohm grid on J4, a DMM in DC volts across MOT_A and
   MOT_B. The meter reads the period mean of a 20 kHz chopped terminal, so it is a truth for
   $v_{mean}$ that needs no current and no model. Rungs every 4 ticks from 40 to 200, then
   25, 50 and 100%, both directions. Pass: the weighted pair within 2 counts of the meter from
   64 ticks.
2. **Shunt bursts with A and B** at 5, 13, 25 and 50%. Fit the tap time constant (expect
   128 us +/- 15%) and check the residual against the study's table.
3. **Duty steps** 10 to 30 to 10% on the stalled MG90 at 20 kHz TEL. Pass: the tap lag
   matches the fitted constant and the boxcar transient stays near 13% of $R\,\Delta i$ for
   one boxcar.
4. **MG90 free-shaft ladder** 5 to 20% in 1% steps, scored against the commutation ripple
   the way notebook 11 does. Pass: firmware back-EMF speed within 3% of the ripple from 8%
   duty, with no edge term in the model.
5. **Verify with the source logged.** `osc ident verify` with `omega_hat_src` streamed. The
   600 and 1200 legs should run on the back-EMF. If verify still reads 4% slow there, its
   cause is elsewhere (notebook 11 suspects the time base).

If step 1 or 2 fails, the swap goes back to 100 pF and candidate 2's go/no-go is an offline
check first. The existing three-slot grid bursts (frames of current, tap A, tap B, folded to
4 ticks) can show whether tap A settles to 1/2 LSB by 17 ticks before the crest at 83 to 110
ticks of half window.

## Open items

- How many rev 2A boards exist with 100 pF taps. That decides whether the board crate alone
  can carry the floor or a tap kind has to reach CALIB and meta.json.
- The leading edge of the terminal (turn-on 36 to 40 ticks, then the tap) is measured on the
  grid and the MG90 bursts at 4 tick resolution, not at the 1/2 LSB level that candidate 2's
  forward floor rests on.
- Fast decay with averaging taps keeps the discontinuous-conduction mismatch between the
  period-mean voltage and the mid-ON current. Closed loop never runs fast decay.
- The verify 4% is not explained by any of this.

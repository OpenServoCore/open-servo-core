# SG90 upgrade: motor-shaft encoder - PCB disc, flex IR sine pair, can NTC

Status: designed; a handful of bench checks and one flex outline remain.
This is the second planned upgrade after the pot-only baseline, and it
assumes the motor-mounted driver board (motor terminals soldered to the
board, MG90-style).

## Concept

Industrial dual-encoder architecture on a $2 servo: the output shaft gets an
absolute magnetic encoder (see `sg90-pot-encoder-upgrade.md`); the motor
shaft gets an incremental analog encoder ahead of the 234.73:1 gear train
(41078/175 exact - the fixed-point scale between motor counts and output
angle). What the motor-side sensor buys:

- Resolution multiplication: every output degree is ~235 motor degrees.
  8-10 bits per motor rev from sine interpolation becomes sub-arcminute
  output resolution for control.
- True velocity - measured, not differentiated from position.
- Motor angle x ratio minus output angle = gear-train wind-up plus backlash:
  a free, crude joint-torque estimate and a backlash observer.

The sensing principle follows adamb314/ServoProject (proven on MG90S-class
servos): two reflective IR sensors read a rotating pattern whose coverage
varies sinusoidally with angle, the two ~90°-phased analog signals are ADC
sampled, and atan2 gives a continuous angle within each motor revolution.
Direction falls out of the angle delta's sign. No edge counting, no
speed-dependent event rate; CPU cost is two ADC reads and a software-CORDIC
atan2 (~3-5 µs) per loop.

ServoProject's published benchmark pins the payoff: absolute static error
1.37° stock -> 0.04° modified. The stock figure matches this clone's error
budget (0.7-1.0° backlash from the gear geometry plus wiper deadband); the
modified figure is below the 0.088° LSB of a 12-bit output encoder - and
was reached with the stock wiper pot still doing output sensing, so it is
the floor for this design, not the ceiling. Static error is the friendly
metric (it converges to the output sensor's noise floor by construction);
the dynamic claims ride on the motor-side loop and backlash feedforward.

Two deliberate departures from the prior art: reading is axial (from below,
through windows in the gear-chamber floor), and the rotating eccentricity is
printed, not physical. ServoProject reads a physically eccentric black-PLA
disc from the side - which forces ~0.15 N of rotating imbalance at 30k RPM
and requires filing the FDM disc smooth while it spins. An axially-read disc
can instead be a balanced, centered part carrying an off-center *printed*
pattern: optical asymmetry with zero mass asymmetry, optically smooth as
fabricated.

## Encoder disc: a 7 mm PCB

The disc is a circular PCB pressed onto the motor pinion.

- 7.0 mm dia, 0.6 mm FR4, 2-layer. Black soldermask both faces, white
  silkscreen eccentric ring on the bottom (sensor-facing) side. Inverted
  contrast is deliberate: carbon-black mask absorbs near-IR and suppresses
  stray reflections; TiO2 white silk is a strong diffuse reflector. Polarity
  only shifts the atan2 phase constant, which calibration absorbs.
- No exposed copper; full inner pours for IR opacity.
- 0.4 mm was rejected on fab economics: at JLC it forces ENIG and lands in
  a rare-combo fee bracket (~10x the price). 0.6 mm with HASL is the $5
  tier - and the flush press (below) makes 0.6 mm mechanically free.
- Center hole: plain drilled round hole (drills are ±0.05 mm; a routed
  gear-profile cutout is unfabbable at module ~0.2). The hole presses over
  the pinion's teeth: the ~10 brass tooth tips broach shallow grooves into
  the FR4 bore - a splined, self-centering, rotation-keyed fit with no
  glue. First panel carries a hole-size ladder 2.05-2.25 mm against the
  2.23 mm tooth tip circle; the bench picks the interference. Press with
  the exit burr up (silk face down), supported near the hole by the jig.

Press depth: flush with the pinion top, enforced by a 3D-printed press jig
with a depth stop. Flush matters twice: pressed lower the disc hits gear 1's
spinning wheel; sitting on top of the pinion it would stack its thickness
into the clearance to gear 3. Embedded flush, the disc lives inside the
pinion's own height envelope and the thickness argument disappears.

Volume variant - flanged pinion: pinion + disc as one part is just a
compound gear whose second stage has no teeth, the same part class as
gear1-gear3, made the same ways (hobbed with a relief groove for cutter
runout, or PM/MIM in one die stroke). Gear houses in the servo/N20 supply
chain quote a custom flanged pinion - 10T module 0.20, 0.98 bore, flange
7.0 x 0.6 in the same height window - for cents at hundreds of pieces, no
mold tooling. Pattern: fiber-laser anneal marks (black oxide) on the
flange's sensor face, batch-marked on loose parts before pressing. Two
callouts a gear drawing won't carry by default: face runout ~0.05 mm
(gear faces are toleranced for mesh, not axial optical reading at a
1.4 mm gap) and a matte finish - a specular background steers the return
off the detector with tilt, so the safe polarity is dark marks on matte
ground. Deletes the disc part, the press-onto-teeth fit, and the hole
ladder; the PCB disc stays the zero-tooling bringup/kit part.

Measured gear-chamber heights (above the floor) that fix the geometry:

| feature | height |
|---|---|
| pinion (2.23 dia, press-fit on shaft) | 0.1 - 2.27 |
| gear 1 big wheel (meshes pinion) | 0.1 - 1.31 |
| gear 3 big wheel (sweeps overhead) | ~3.11 - 5.11 |
| disc, pressed flush | 1.67 - 2.27 |
| clearance below / above | 0.36 / 0.84 |
| motor shaft end play (FF-M20 datasheet) | 0.05 - 0.20 |

Worst-case end-play stacking clears in both directions. The pinion's own
press depth on the shaft is a reserve adjustment - raising it buys disc
room but spends gear 1 mesh overlap, so it stays untouched.

## Sensors and deck windows

Two ITR8307 reflective sensors on a flex PCB clamped between the motor's
front face and the underside of the 1.0 mm gear-chamber floor. The clamp is
the mechanical design:

- Sensor standoff is set by molded geometry (deck thickness minus package
  height): faces land ~0.25 mm proud of the floor. Optical gap to the
  pattern ≈ 1.4 mm (1.2-1.6 with tolerances), inside the ITR8307 response
  band, and tunable upward by pressing the disc short of flush if the bench
  prefers a longer gap.
- The motor's Φ4 front boss seats in the deck's Φ4 pinion hole - that is
  the motor's locating fit, and the gear-mesh separating force loads the
  hole rim on the anti-mesh side (~150-210° from the mesh direction). That
  arc stays intact.
- Sensor windows are two hand-drilled keyhole notches off the central hole
  at ~130° and ~220° from the mesh axis, sensing radius r ≈ 2.9 mm. Both
  angles clear gear 1's overhead sweep (needs >62° from mesh) and stay off
  the loaded rim arc. The windows are oversized clearance holes cut with a
  printed drill jig registered on deck features - quadrature phase comes
  from the flex layout (±0.05 mm), never from the hand operation.

Tooth glint: the pinion's bare brass teeth are exposed below the disc
(0.1-1.67 above the floor), directly over the central deck hole - and the
keyhole windows open into that hole, giving each sensor a slot-shaped
corridor to ten polished moving facets. The glint is LED-correlated, so
strobe subtraction cannot remove it; it is also a pure function of motor
angle (10th harmonic plus a 1/rev runout term), so whatever survives
mitigation lands in the calibration LUT. Mitigation ladder:

- Light fence (primary): one printed part - a snoot box per sensor,
  joined by a thin bridge across the 130-220° arc (clear of gear 1's
  sweep, like the windows themselves). Walls 0.4-0.5 mm black resin,
  ~1.0-1.2 mm tall under the 1.67 disc ceiling, top apertures open to
  the pattern ring, glued to the deck. The inboard walls close the
  keyhole corridors to the teeth; four walls per box also seal the
  lateral ambient path (case plastic is NIR-translucent) and isolate
  the sensors from each other - the shared strobe GPIO fires both LEDs
  at once, so cross-illumination is otherwise indistinguishable from
  signal. The bridge fixes box-to-box pitch, so the part self-jigs on
  the windows. Glue order is forced: fence onto the bare deck and full
  cure BEFORE the motor-flex sandwich - the boxes sit over the windows,
  so gluing with sensors in place drips epoxy onto the sensor faces.
  Dabs on the outer wall feet, away from the window rims, so squeeze-out
  creeps outward, not down the bore. Finger-press only, no clamp - the
  walls would crush, and the bond is non-structural anyway (the fence
  moves with the case; its inertial load is millinewtons). The cure
  wait guards geometry, not strength: a part that leans while soft eats
  the 0.4 mm disc clearance. Factory form: two molded pockets in a deck
  rib, zero added parts.
- Brass blackening (bench diagnostic + mild assist, plastic trains
  only): conversion coatings only - swab-on copper-selenide brass black
  at the bench, Ebonol-C-style black CuO dip as the batch process.
  Sub-um thick, no dimensional change; plating is rejected outright,
  2-10 um per flank on 0.31 mm teeth shifts the mesh. Not a lifetime
  fix: mesh wear re-polishes the flank contact band, and the spinning
  pinion sweeps that band past both sensors 10x per rev - a rotating
  mirror facet the coating cannot keep dark. Brass-on-POM wears it
  slowly; a metal gear 1 strips it in hours and turns the glint into a
  drifting error the LUT chased at cal time. The wear-proof black is
  the molded carbon-POM pinion-disc: black in bulk, wear exposes more
  black. The fence stays primary - occlusion has no wear mechanism.

## Flex PCB

2-layer, 0.11 mm polyimide, no stiffener - the motor-deck sandwich is the
stiffener, and every added lamination thickens the clamp stack. Layout
zones, top to bottom:

- Clamp zone (under the motor face): traces and vias only, no bodies.
- Wall-gap run: the first ~6 mm descends the 0.85 mm gap between the motor
  flat and the case wall - bare and conformal. The motor compartment wall
  ends ~6 mm down (half the can); below that is open air.
- Tail-end cluster, past the wall: the passives. Two phototransistor load
  resistors (22-68k, bench-tuned) and two ~1 nF filter caps, 0603. A filter
  cap stiffens its whole net at HF regardless of position, so end-of-tail
  placement is electrically equivalent to sensor-end and mechanically far
  better. LED series resistor lives on the main board.
- NTC wing: a flap wrapping the motor can at mid-height (a static stator
  surface - nothing moves there), 0402/0603 NTC in direct contact with a
  dab of thermal paste. Motor-can temperature feeds stall protection and
  current derating at the actual heat source. The wrap doubles as the
  mid-span anchor against vibration. Divider resistor on the main board.
- Copper: hatched pour on anything that bends, solid allowed under the
  clamped zone and component clusters. The layer-2 ground pour is also a
  light shield - bare polyimide is NIR-translucent, and so is the blue case
  plastic (dyes target visible light, not 850-940 nm).

Tail termination: 5 conductors (LED drive, OUT_A, OUT_B, NTC, GND) at
1.27 mm pitch, lap-soldered to top-face pads on the main board. Board pads
run ~1 mm past the flex tip as the iron's landing zone; a 0.6 mm hole
through each mated pad pair lets solder rivet the joint. Two alignment
holes in the flex match board holes for pin registration. An epoxy bead
behind the pad row takes the bend strain so the fillets never do. Solder
the tail last in the assembly sequence.

## Firmware

- LED strobed, not continuous: sample each channel with the emitter on and
  off, subtract. Cancels the ambient *mean* (sunlight through the
  translucent cap included) and cuts emitter power ~10x (to ~1 mA average
  per servo). What subtraction cannot cancel, the light fence blocks
  physically: ambient shot noise (subtracting adds the two samples' noise
  in quadrature), headroom eaten by the ambient pedestal, and PWM-flickered
  room lighting - which violates the constant-between-samples assumption
  and leaks through as noise.
- Angle: atan2 of the two differential samples via software CORDIC,
  ~3-5 µs. Per-unit gain/offset calibration per channel (sensor spread, LED
  aging, gloss variation) folds into the existing fleet CAL machinery.
- Speed: 30k RPM = 500 rev/s = one sine cycle per 2 ms. Measured Kv
  (~7960 RPM/V) makes that the 4.8-5 V free-run figure; 6 V is ~40k RPM,
  one cycle per 1.5 ms. Loop-rate sampling in the kHz range tracks either
  with a wide unwrap margin.
- Ratio self-cal: with both encoders, the gear ratio is measured, not
  configured - slow sweep, unwrapped motor angle over output angle. A
  one-direction sweep keeps the mesh loaded on one flank, so backlash
  never enters. 180° travel gives ~0.1% (output-encoder INL over the
  baseline) - already sufficient for the wind-up observer. Continuous-
  rotation builds do better: both encoders' dominant errors are periodic
  per rev (eccentricity ripple per output rev, sine harmonics per motor
  rev) and cancel exactly over whole revolutions, so a multi-rev sweep
  reaches ppm - enough for a continued-fraction snap to the exact
  rational (41078/175 needs ~16 ppm). Doubles as a gear-integrity
  self-test: a ratio that stops matching stored CAL means the train
  changed - worn, stripped, or a factory swapped gears mid-production.
- Pin budget: 2 ADC (sine pair) + 1 ADC (NTC) + 1 GPIO (LED strobe) = 4 MCU
  pins. This is the audit item on a TSSOP20 part - freeze the flex tail
  pinout only after the board pinout confirms it.

## Cost (self-assembly, per servo)

| item | est. cost |
|---|---|
| encoder disc (30/$5.30 panel) | $0.18 |
| flex PCB ($2 tier + fee, amortized) | $0.60 |
| ITR8307 x2 | $0.30 |
| NTC 0402 | $0.03 |
| R + C cluster (4x 0603) | $0.04 |
| epoxy, paste, jig filament | ~$0.05 |
| **total per servo** | **~$1.20** |

Order-level: disc panel ~$5.30, flex ~$10-15 with engineering fee, sensors
~$2/10. Ship everything economy; the shipping line is otherwise the largest
item in the entire build.

## Open items

- Bench: ITR8307 signal amplitude and contrast against a greased, running
  pinion - the one physics risk in the design. Run twice, bare vs
  blackened pinion, same disc: the delta in 10th-harmonic ripple prices
  the glint term and decides whether the fence alone suffices.
- Bench: hole-ladder interference selection for the disc press.
- MCU pin audit; then freeze the flex tail pinout.
- Deck top-down photo -> flex outline, window drill-jig and light-fence
  CAD.

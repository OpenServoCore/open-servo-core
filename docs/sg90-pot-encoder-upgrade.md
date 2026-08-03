# SG90 upgrade: pot mod - magnetic output encoder

Status: designed, unbuilt. This is a planned upgrade on top of the pot-only
baseline board, not part of it. It replaces the pot's *electrical* function
with a contactless absolute magnetic encoder while keeping the pot's
*mechanical* functions - which turn out to be the valuable half of the part.

## Concept

A stock SG90 feedback pot does three mechanical jobs besides sensing: its
bushing is the output shaft's top bearing, its shaft top drives the final
gear, and its three legs are a soldered mounting platform. Gut the wiper and
all three survive: the shaft rotates 360° freely, and the pot becomes a
free, pre-aligned mechanical carrier for a magnetic encoder.

- A diametric magnet glues to the exposed shaft rivet on the pot's back face.
- A small coupon PCB hangs on the pot's own legs, which places the sensor on
  the shaft axis by construction - the legs are symmetric about the axis.
- The hall angle sensor sits on the coupon's far side and senses through the
  board.

Why bother: wiper contact noise, deadband, and wear are unreachable by
firmware compensation - they are exactly the class of error that has to be
fixed in hardware. The encoder also unlocks true 360° absolute position,
continuous-rotation builds (trim the final gear's stop tab), and multi-turn
counting in firmware.

## Measured pot geometry

All numbers from calipers on a disassembled donor pot; this part has no
public datasheet, so this table is the datasheet.

| feature | value |
|---|---|
| body | 9.92 mm dia x 4.30 mm |
| shaft | 1.35 mm dia, brass (non-magnetic - magnet will not self-clamp) |
| rivet flare (back face) | ~2.6 mm dia |
| rear bushing stickout (magnet glue surface) | 0.2 mm |
| leg tab holes | 0.5 mm dia, all at r = 3.5 mm from axis |
| outer legs | ±48° from the wiper leg |
| leg length | 4.78 mm |

Stock servo board hole pattern (the factory leg forming, so harvested pots
drop on with zero re-bending): outer holes 6.0 mm c-c, wiper hole 3.33 mm
from each outer - 1.45 mm behind their line - which puts all three holes on
a common r ≈ 4.1 mm circle about the shaft axis, and the axis 2.70 mm from
the outer-hole line. The `shared:Pot_Wire_Pads` footprint encodes this
pattern with its origin at the shaft axis.

## Magnet

D6 x 2.5 mm diametrically magnetized NdFeB, N35. This is the size the
sensor datasheets characterize against; 2.5 mm of thickness is what lets it
sense through a 0.8-1.0 mm board with margin. Field at the die: roughly
80-100 mT at 0.5 mm, ~60-70 mT at 1 mm, ~40 mT at 2 mm - inside the sensor
window across a 0.5-3 mm gap.

Diametric is the one non-negotiable spec: axially magnetized discs (the
common kind) are invisible to hall angle sensors. Incoming check: two
diametric discs snap together edge-to-edge and drag each other into pole
alignment when one is rotated.

Mounting: thin epoxy (CA only for removable bench builds), centered by a
3D-printed cap that registers on the pot body OD with a 6.1 mm bore, plus a
printed follower plug that weights the magnet flat during cure - the brass
shaft gives no magnetic self-clamping. No re-crimp after gutting: the pot
is bearing + mounting platform only. Axial float is bounded by the rivet
flare on one side and the gear stack under the case ceiling on the other,
and the sensor reads field *direction*, not magnitude, so bounded gap
wander is harmless inside the field window. Bench check: the back plate
(the legs' anchor) is what the crimp made rigid - confirm it sits steady
in the case pocket, and the once-per-rev ripple check catches any drift.

Imbalance and adhesion at output-shaft speeds are non-issues; the parked
long-term answer to glue entirely is a custom carrier with a captive magnet
pocket (see memory / future work), justified only at fleet scale.

## Coupon and sensor

- Sensor: MT6701 (~14-bit, QFN3x3). Its ratiometric analog output mode is
  the migration path: it drives the existing pot ADC front end as a drop-in
  fake wiper - zero new MCU pins, zero firmware change for bringup - then
  I2C/SSI later for full resolution. Fallback: AS5600 (12-bit, I2C-only).
- Coupon sits on the pot legs; sensor on the far face, sensing through the
  board. Air gap ≈ 1.8 mm nominal (magnet top to die), mid-window.
- Pads: lay four (5V / GND / OUT-SDA / SCL), use three in analog mode. The
  stock pot harness is also three wires, so the driver board side is
  identical for pot and encoder builds.
- Alignment: magnet-to-shaft eccentricity is fixed by the glue jig;
  sensor-to-axis offset is tuned by bending the legs (bend near the pot
  body to translate without tilt). Live feedback: field-magnitude/AGC
  registers show a once-per-rev ripple when off-center - solder one leg,
  spin, nudge until the ripple flattens, solder the rest. Budget ~0.25 mm
  eccentricity for rated INL.
- Bringup check: read AGC/field-status at the assembled gap and confirm
  mid-window before the epoxy becomes permanent.

## Cost (self-assembly, per servo)

| item | est. cost |
|---|---|
| MT6701 | $0.55 |
| D6x2.5 diametric magnet | $0.54 |
| coupon PCB (panel amortized) | $0.15 |
| decoupling + misc passives | $0.05 |
| epoxy, jig filament | ~$0.05 |
| **total per servo** | **~$1.35** |

Order-level: magnets ~$27/50, coupon panel ~$5-10, sensors ~$5.50/10.
Everything ships economy - nothing here is schedule-critical.

## Open items

- Mock-rig validation: printed fixture, assembled gap, AGC register sweep
  vs distance, ripple-based centering rehearsal.
- Verify the final gear's stop tab trim procedure for continuous-rotation
  builds, and that the magnet's pull on the steel arbor and case screws
  above does not change mesh feel.

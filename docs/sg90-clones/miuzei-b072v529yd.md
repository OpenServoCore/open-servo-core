# Miuzei SG90 (Amazon B072V529YD)

Miuzei 9g micro servo, blue translucent case. Not TowerPro.

## Gear train

All gears white plastic except the brass motor pinion.

| stage | teeth | module | mesh partner |
|---|---|---|---|
| motor pinion | 10 | 0.20 | gear1 big |
| gear1 | 47 / 10 | 0.20 / 0.25 | pinion / gear2 big |
| gear2 | 38 / 8 | 0.25 / 0.30 | gear1 small / gear3 big |
| gear3 | 32 / 7 | 0.30 / 0.40 | gear2 small / gear4 |
| gear4 (output) | 23 | 0.40 | gear3 small |

- Overall ratio: 47/10 x 38/10 x 32/8 x 23/7 = 41078/175 = **234.73:1**.
- Center distances: motor-arbor 5.70, arbor-pot 6.00 (the three pot-arbor
  meshes all land on exactly 6.00 with standard modules - clean design).

### Failure pattern: gear2's 8-tooth pinion strips

3 of 3 failed servos: same gear, same stage, every time caused by stall,
missing 1-3 teeth after. The geometry explains it: a standard root circle
for an 8T module-0.30 pinion (1.65) would sit inside the part's 1.90
bore, so the molded pinion has shallow teeth on a ~0.12 mm wall - the
lowest contact ratio in the train on the thinnest backing, taking stall
torque through the 4:1 final stage. This is the gear to watch (or
protect, or replace in brass) on this clone.

## Motor

FF-M20 family flat can, short variant (the common NFP catalog drawing is
the 15 mm sibling; this one is not it).

- Can: 8.0 across flats x 10.0 dia x 12.3 long, drawn steel, vent holes
  at the brush end.
- Front: 4.0 dia boss, 0.6 tall, no plate stage; shaft 1.0 dia, 2.7
  stickout; 16.6 total rear boss tip to shaft tip.
- Rear: 1.0 boss (0.6 brass bushing + 0.4 plastic collar); shaft
  dead-ends inside the bushing - not accessible from the rear.
- Terminals: 1.5 x 0.3 tabs, 7.3 apart on the round axis, 1.0 long
  (flush with the rear boss).
- Construction: 3-pole wound rotor, 3-segment brass commutator, two
  Z-bent brush springs on a plastic brush card with felt oiler.
- Electrical (bare can, measured on the bench via a locked-rotor step
  and a free-run lap): winding resistance 4.2 ohm; inductance ~0.5 mH
  (electrical tau 102 us); stall current 1.50 A at 7.7 V through the
  4.7 ohm total drive loop; free-run current ~81 mA; Kv ~7960 RPM/V,
  Kt = 1/Kv = 1.20 mN.m/A. The locked-rotor rise converges slightly
  faster than the fitted exponential near stall - iron-core inductance
  dropping as the core saturates.

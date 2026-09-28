"""Measured MG90 facts - the datasheet for parts that have none.

Donor: Vorpal-brand MG90 clone (mg90-a in notebooks/oscnb/servos.py). All-metal
four-stage train, ball-bearing output, plastic D-key coupling to the pot.
Clone families differ (gears, motor, pot, case); a brand switch re-opens every
entry.

Provenance tags: [pho] counted from teardown photos, [der] derived from other
entries. Nothing here is dimensioned yet; only the train is counted.
"""

# --- gear train, tooth counts [pho, counted 2026-09-28] ---
# mesh chain: pinion 9 -> 48 (gear1), 12 -> 48 (gear2), 10 -> 38 (gear3),
# 10 -> 38 (gear4, output)
PINION_T = 9
GEAR1_T = 48
GEAR1_PINION_T = 12
GEAR2_T = 48
GEAR2_PINION_T = 10
GEAR3_T = 38
GEAR3_PINION_T = 10
GEAR4_T = 38               # output gear, drives the horn
SPLINE_T = 20              # [pho] horn spline, not part of the ratio
GEAR_RATIO = (48 * 48 * 38 * 38) / (9 * 12 * 10 * 10)  # 308.0533:1 = 23104/75

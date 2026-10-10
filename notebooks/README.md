# notebooks

This folder is where I look at real data from the servo rig. The firmware
streams raw ADC samples at the full 20 kHz control rate, the captures get
checked in under `telemetry/`, and the notebooks load them, plot them, and
check the control math against what the hardware actually did.

These notebooks are for understanding and for writeups, not for CI. They are
committed with their outputs baked in, so they read as documentation straight
from GitHub.

## Setup

You will need [uv](https://docs.astral.sh/uv/). Then:

```
cd notebooks
uv sync
uv run jupyter lab
```

`uv sync` creates a `.venv` and installs numpy, scipy, pandas, matplotlib, and
jupyterlab. Python is pinned to 3.12 in `.python-version`, and uv will fetch it
for you if you don't have it.

`uv run pytest` runs the tests under `tests/`, which check the parts of
`oscnb` that can be checked offline against data with a known answer.

## Telemetry

The captures live under `telemetry/`, one folder per dataset (one servo or
static load, on one board, on one supply), then one folder per experiment, then
one folder per capture:

```
telemetry/
  sg90-a__dev-v006-D__2s/
    dataset.toml
    grid/
      capture-1/
        meta.json
        sweep.csv.gz
```

Every recording is raw ADC counts straight off the wire, with a per-sample
tick counter, so missing samples are provable from the data itself. The
constants that turn counts into volts and amps live in `oscnb/boards.py` (the
measurement chain) and `oscnb/servos.py` (the thing measured). Each
recording's `meta.json` carries the board's parts in a `sense` block, and the
loader picks the board from it and refuses a capture that does not match.

## The notebooks

Each notebook states what it measures, how, and what is still open at its end.
01 to 08 read an SG90 on board D, the hand-reworked development board before
rev 2A; 09 reads an MG90 on board D; 10 on read rev 2A, with board D alongside
where a comparison needs it.

- `01-bridge-states`: what each ADC sample means in each H-bridge state and
  decay mode, and when it is valid.
- `02-electrical`: winding resistance and brush drop as a two-parameter line,
  the winding inductance, and the burst fit's R-V0 ridge.
- `03-back-emf`: the back-EMF constant by three routes, and ripple speed plus
  coast back-EMF as a speed that needs no pot.
- `04-friction`: Coulomb and viscous friction from coast deceleration and
  steady current, and breakaway.
- `05-inertia`: rotor inertia from the rise curve, cross-checked by coast.
- `06-gear-train`: the ripple tachometer, motor angle against the pot, gear
  play and slip signatures.
- `08-velocity-observer`: rest noise per channel and its structure, then
  back-EMF, pot derivative and ripple as speed estimates, and how the observer
  combines them.
- `09-pot-linearization`: the ripple-referenced position table and degrees
  from counted teeth.
- `10-sense-chain-boards`: capture integrity, and the sense chain graded
  against a meter on a resistor grid, board against board. Then the current
  reading on rev 2A's final amplifier network, on a stalled MG90 and a 15.3 ohm
  resistor, and the settle gain it needs.
- `11-back-emf-on-rev-2a`: the low-duty back-EMF over-read, re-scored on rev 2A.
- `12-pot-table-free-vs-governed`: how far the position table moves between
  sessions, against its own split-half floor.
- `13-output-angle-from-video`: output angle from a camera, proven on rendered
  frames.
- `15-op-amp-over-board-temperature`: current amplifier gain and zero against
  board temperature.
- `16-duty-floor-in-the-tap-sample-timing`: where each tap samples inside a
  short drive window, and the duty floors that follow.
- `17-winding-thermometer-calibrated`: copper's temperature coefficient
  confirmed on the MG90 with a DC meter pair.
- `18-two-node-thermal-model`: the winding and can as two thermal nodes.
- `19-shipped-winding-thermometer`: the NTC-based winding thermometer as
  shipped, graded against a K bead on the can across torque-off gaps, with
  the bridged-segment event, the guards and the boot prior. Dataset
  `mg90-a__dev-v006-2A__dps__thermometer`; loader `oscnb.thermometer`.
- `20-bare-motor` measures the MG90's motor out of the servo, on a plate with
  no gearbox or load: rest resistance across rotor stops, no-load current,
  the back-EMF constant by the coast route, why the duty ramp cannot measure
  the inductance, the resistance in rotation, and breakaway. Dataset
  `mg90-a__dev-v006-2A__dps__bare-motor`; loaders `oscnb.staticsweep` and
  `oscnb.meterlog`.

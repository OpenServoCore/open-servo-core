# mechanical

Parametric CAD for OSC mechanical parts, written in
[build123d](https://github.com/gumyr/build123d) (Python, OCCT kernel).
One folder per project; each project writes its printable outputs
(STEP/STL/SVG/preview PNGs) to its own `build/` (gitignored).

- `sg90/` - SG90 servo modeling: motor, gears, pot, case, light fence.
  Measurements go into `sg90/measurements.py` with provenance; geometry
  is derived from them. One part per module; `build()` returns the
  solid.
- `encbench/` - bench encoder rig for the ITR1204 coupon
  (hardware/boards/encoder-board): base, sled, clamp, discs, shims,
  paper patterns. Parametric over motor dims (imports
  `sg90.measurements`) and quadrature-phase presets.
- `render.py` - shared headless STL preview.

## Setup

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Build

Run from `mechanical/`:

```sh
.venv/bin/python -m sg90.motor    # one part -> sg90/build/
.venv/bin/python -m encbench      # all rig parts -> encbench/build/
.venv/bin/python -m render sg90/build/sg90_motor.stl   # preview PNG
```

Stable STEPs consumed by KiCad get copied to `hardware/shared.3dshapes/`
deliberately, not by the build.

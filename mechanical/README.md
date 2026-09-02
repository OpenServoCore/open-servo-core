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
- `carrier/` - tier-2 encoder carrier: printed pot replacement (stator
  body with coupon shelf + magnet cup rotor) around purchased 1.4mm rod
  and brass tube stock; imports `sg90.measurements`, dims pending the
  donor measurement session.
- `stenciljig/` - solder-paste stencil jig, build123d implementation
  of the mechanism from bobstay's parametric jig
  (printables.com/model/1209372, credited with thanks). Heat-set
  inserts swapped for captive M3 nuts, plate screws top-driven via
  under-base nut bars. One part per file, shared spec in
  `common.py`, mating cavities offset from the owning part's
  profile; every part exports pre-oriented and support-free (all
  internal ceilings are 45-degree tents, hoppers or cones). Two
  stacking parts: the jig stands on screw-on feet next to an open
  tote (`box.py`) standing on four of the same feet, holding the
  thickness adapters, low-profile standing-wedge squeegees, stencil
  sheets and spares; for transport the jig stacks onto the box as
  its lid - the rear feet's hook noses latch the box wall and two
  printed quarter-turn cam knobs seat under the box's descending
  ceiling tents to draw it down tight, no hardware in the joint.
  TPU desk pads snap into the feet's through bores (one pad and one
  foot model everywhere).
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

"""Export every part's STL: python -m stenciljig [name...] (from
mechanical/; names filter by substring). Each part is rotated to its
module's PRINT pose and dropped onto the bed, so every STL slices
as-is with no flipping in the slicer."""
import pathlib
import sys

from build123d import Pos, export_stl

from stenciljig import (adapter, base, box, clamp_plate, foot, knob,
                        nutbar, pad, side_plate, squeegee, tongue,
                        tray)
from stenciljig.common import AD_PAIRS

BUILDERS = {
    "base": (base.build_base, base.PRINT),
    "side": (side_plate.build_side_plate, side_plate.PRINT),
    "clamp": (clamp_plate.build_clamp_plate, clamp_plate.PRINT),
    "tongue": (tongue.build_tongue, tongue.PRINT),
    "nutbar": (nutbar.build_nutbar, nutbar.PRINT),
    "foot": (foot.build_foot, foot.PRINT),
    "hookfoot": (lambda: foot.build_foot(True), foot.PRINT),
    "pad": (pad.build_pad, pad.PRINT),
    "box": (box.build_box, box.PRINT),
    "tray": (tray.build_tray, tray.PRINT),
    "knobrod": (knob.build_rod, knob.PRINT),
    "knobhandle": (knob.build_handle, knob.PRINT),
}
for ta, tb in AD_PAIRS:
    BUILDERS[f"adapter{round(ta * 10):02d}_{round(tb * 10):02d}"] = \
        (lambda ta=ta, tb=tb: adapter.build_adapter(ta, tb),
         adapter.PRINT)
for w, k in squeegee.SQ_SIZES:
    BUILDERS[f"squeegee{round(w):02d}"] = \
        (lambda w=w, k=k: squeegee.build_squeegee(w, k), squeegee.PRINT)


def print_pose(part, rot):
    q = rot * part if rot else part
    bb = q.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2,
               -bb.min.Z) * q


if __name__ == "__main__":
    picks = sys.argv[1:]
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    for name, (build, rot) in BUILDERS.items():
        if picks and not any(a in name for a in picks):
            continue
        part = build()
        posed = print_pose(part, rot)
        export_stl(posed, str(out / f"{part.label}.stl"), tolerance=0.01)
        print(f"{part.label}: volume {part.volume / 1000:.1f} cm^3 -> {out}")

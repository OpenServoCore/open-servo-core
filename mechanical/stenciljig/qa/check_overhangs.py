"""Print-pose overhang audit: every part must be support-free in its
export pose - no down-face steeper than 45 deg unless it sits on the
bed or spans <=3.7 as a bridge. Narrow-span = min(bbox edges,
2*area/perimeter) so annular rings measure their true width, not
their diameter. Curved faces grounded on the bed (lying-rod belly
bands) are exempt - the bed squash supports them.
Run from mechanical/: PYTHONPATH=. .venv/bin/python stenciljig/qa/check_overhangs.py"""
import math

from build123d import GeomType

from stenciljig.__main__ import BUILDERS, print_pose

DOWN = math.cos(math.radians(45)) + 0.01   # normal.Z < -DOWN = steeper
BRIDGE = 3.7
BED = 0.15

fails = 0
for name, (build, rot) in BUILDERS.items():
    part = build()
    posed = print_pose(part, rot)
    bad = []
    for f in posed.faces():
        uv = ((0.5, 0.5),) if f.geom_type == GeomType.PLANE else \
            tuple((u / 2, v / 2) for u in range(3) for v in range(3))
        down = False
        for u, v in uv:
            try:
                if f.normal_at(f.position_at(u, v)).Z < -DOWN:
                    down = True
                    break
            except Exception:
                continue
        if not down:
            continue
        bb = f.bounding_box()
        if bb.max.Z < BED:
            continue                       # sits on the bed
        if f.geom_type != GeomType.PLANE and bb.min.Z < BED:
            continue                       # lying-rod belly band
        per = sum(e.length for e in f.edges())
        span = min(bb.size.X, bb.size.Y, bb.size.Z,
                   2 * f.area / per if per > 0 else 1e9)
        if span > BRIDGE:
            bad.append((round(span, 2), round(bb.min.Z, 2)))
    fails += bool(bad)
    print(f"{part.label:28s} {'ok' if not bad else f'OVERHANGS {bad}'}")
print("ALL PRINTABLE" if not fails else f"{fails} PARTS FAIL")
raise SystemExit(1 if fails else 0)

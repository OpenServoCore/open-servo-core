"""SG90 gear train, all five parts. Axis = Z, bottom face at z=0.

True involute flanks; tips truncated to the measured ODs (measured reads
a few % under nominal m*(N+2), see measurements.py). Compound gears: big
wheel uses the upstream mesh module, small pinion the downstream one.
"""
import math

from build123d import *

from . import measurements as m

PRESSURE_ANGLE_EST = 20.0    # UNVERIFIED - standard assumption
PINION_BORE_EST = 0.98       # press-fit on 1.00 motor shaft
ARBOR_BORE_EST = 1.25        # gear1/gear3 slip on 1.20 arbor
GEAR2_BORE_EST = 1.90        # slips on 1.85 pot shaft main section
GEAR4_BORE_D_EST = 1.40      # slips on 1.35 pot shaft tip
GEAR4_BORE_FLAT_EST = 1.03   # across the D flat, shaft flat 1.00
# std dedendum root (dia 1.65) falls inside the 1.90 bore; gear3 tips
# sweep r=1.10 from this axis, so clamp between bore and mate sweep
GEAR2_PINION_ROOT_R_EST = 1.07
# 180-deg stop tab on gear4 top face at the rim - dims unmeasured
STOP_TAB_W_EST = 1.5         # tangential
STOP_TAB_DEPTH_EST = 1.0     # radial, inward from rim
STOP_TAB_H_EST = 1.2         # above big-wheel top face
# horn spline: 21 serrations (SPLINE_T) on the boss tip, smooth bearing
# section below rides the case boss bore
SPLINE_L_EST = 3.0           # UNVERIFIED - splined length from boss tip
SPLINE_ROOT_D_EST = 4.4      # UNVERIFIED - serration root dia

BRASS = (0.72, 0.58, 0.28)
POM = (0.92, 0.92, 0.88)


def _gear_face(n, mod, tip_d, root_r=None, flank_pts=8, arc_pts=3):
    pa = math.radians(PRESSURE_ANGLE_EST)
    rp = n * mod / 2
    rb = rp * math.cos(pa)
    rt = tip_d / 2
    rr = root_r if root_r is not None else rp - 1.25 * mod
    inv_a = math.tan(pa) - pa

    def half_t(r):  # angular half tooth thickness on the involute
        phi = math.acos(min(1.0, rb / r))
        return math.pi / (2 * n) + inv_a - (math.tan(phi) - phi)

    def lin(a, b, k):
        return [a + (b - a) * i / (k - 1) for i in range(k)]

    r0 = max(rr, rb)  # below rb the flank drops radially to the root
    tooth = [(rr, a) for a in lin(-math.pi / n, -half_t(r0), arc_pts)]
    tooth += [(r, -half_t(r)) for r in lin(r0, rt, flank_pts + 1)]
    tooth += [(rt, a) for a in lin(-half_t(rt), half_t(rt), arc_pts)]
    tooth += [(r, half_t(r)) for r in lin(rt, r0, flank_pts + 1)]
    tooth += [(rr, a) for a in lin(half_t(r0), math.pi / n, arc_pts)]

    pts = []
    for i in range(n):
        base = 2 * math.pi * i / n
        for r, a in tooth:
            p = (r * math.cos(a + base), r * math.sin(a + base))
            if not pts or math.dist(p, pts[-1]) > 1e-6:
                pts.append(p)
    if math.dist(pts[0], pts[-1]) < 1e-6:
        pts.pop()
    return Polygon(*pts, align=None)


def _spur(n, mod, tip_d, h, root_r=None) -> Part:
    return extrude(_gear_face(n, mod, tip_d, root_r), h)


def _finish(p, label, color) -> Part:
    p.label = label
    p.color = Color(*color)
    return p


def build_pinion() -> Part:
    p = _spur(m.PINION_T, m.MESH_MODULES[0], m.PINION_D, m.PINION_H)
    p -= Cylinder(PINION_BORE_EST / 2, 3 * m.PINION_H)
    return _finish(p, "pinion", BRASS)


def build_gear1() -> Part:
    p = _spur(m.GEAR1_T, m.MESH_MODULES[0], m.GEAR1_D, m.GEAR1_H)
    p += Pos(0, 0, m.GEAR1_H) * _spur(
        m.GEAR1_PINION_T, m.MESH_MODULES[1], m.GEAR1_PINION_D,
        m.GEAR1_TOTAL_H - m.GEAR1_H)
    p -= Cylinder(ARBOR_BORE_EST / 2, 3 * m.GEAR1_TOTAL_H)
    return _finish(p, "gear1", POM)


def build_gear2() -> Part:
    p = _spur(m.GEAR2_T, m.MESH_MODULES[1], m.GEAR2_D, m.GEAR2_H)
    p += Pos(0, 0, m.GEAR2_H) * _spur(
        m.GEAR2_PINION_T, m.MESH_MODULES[2], m.GEAR2_PINION_D,
        m.GEAR2_TOTAL_H - m.GEAR2_H, root_r=GEAR2_PINION_ROOT_R_EST)
    p -= Cylinder(GEAR2_BORE_EST / 2, 3 * m.GEAR2_TOTAL_H)
    return _finish(p, "gear2", POM)


def build_gear3() -> Part:
    p = _spur(m.GEAR3_T, m.MESH_MODULES[2], m.GEAR3_D, m.GEAR3_H)
    p += Pos(0, 0, m.GEAR3_H) * _spur(
        m.GEAR3_PINION_T, m.MESH_MODULES[3], m.GEAR3_PINION_D,
        m.GEAR3_TOTAL_H - m.GEAR3_H)
    p -= Cylinder(ARBOR_BORE_EST / 2, 3 * m.GEAR3_TOTAL_H)
    return _finish(p, "gear3", POM)


def build_gear4() -> Part:
    p = _spur(m.GEAR4_T, m.MESH_MODULES[3], m.GEAR4_D, m.GEAR4_H)
    boss_h = m.GEAR4_TOTAL_H - m.GEAR4_H
    p += Pos(0, 0, m.GEAR4_H) * Cylinder(
        m.GEAR4_BOSS_D / 2, boss_h - SPLINE_L_EST,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p += Pos(0, 0, m.GEAR4_TOTAL_H - SPLINE_L_EST) * _spur(
        m.SPLINE_T, m.GEAR4_BOSS_D / (m.SPLINE_T + 2), m.GEAR4_BOSS_D,
        SPLINE_L_EST, root_r=SPLINE_ROOT_D_EST / 2)
    p += Pos(0, m.GEAR4_D / 2 - STOP_TAB_DEPTH_EST, m.GEAR4_H) * Box(
        STOP_TAB_W_EST, STOP_TAB_DEPTH_EST, STOP_TAB_H_EST,
        align=(Align.CENTER, Align.MIN, Align.MIN))
    r = GEAR4_BORE_D_EST / 2
    d_face = Circle(r) - Pos(0, GEAR4_BORE_FLAT_EST - r) * Rectangle(
        GEAR4_BORE_D_EST, r, align=(Align.CENTER, Align.MIN))
    p -= Pos(0, 0, -1) * extrude(d_face, m.GEAR4_TOTAL_H + 2)
    return _finish(p, "gear4", POM)


def build() -> Compound:
    # preview lineup only; the assembly imports the individual builders
    parts = [build_pinion(), build_gear1(), build_gear2(),
             build_gear3(), build_gear4()]
    dias = [m.PINION_D, m.GEAR1_D, m.GEAR2_D, m.GEAR3_D, m.GEAR4_D]
    placed, x = [], 0.0
    for i, (p, d) in enumerate(zip(parts, dias)):
        if i:
            x += dias[i - 1] / 2 + 2.0 + d / 2
        q = Pos(x, 0, 0) * p
        q.label, q.color = p.label, p.color
        placed.append(q)
    return Compound(label="sg90_gears", children=placed)


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "sg90_gears.step"))
    export_stl(part, str(out / "sg90_gears.stl"), tolerance=0.01)
    from blueprint import blueprint
    blueprint(part, str(out / "sg90_gears_bp.svg"), name="sg90_gears")
    print(f"sg90_gears: volume {part.volume:.0f} mm^3 -> {out}")

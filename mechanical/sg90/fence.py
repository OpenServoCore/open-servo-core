"""SG90 light fence - snoot boxes for the ITR8307 pair, glued to the
deck top (docs/sg90-motor-encoder-upgrade.md, tooth-glint section).

Frame: motor axis = Z at origin, deck top = z0, mesh axis (toward the
gear1 arbor) = +X. Print flat, 0.2 nozzle, carbon-black filament.
All dims *_EST until the deck top-down photo pins the window geometry.
"""
import math

from build123d import *

WALL_T_EST = 0.5           # double perimeter on a 0.2 nozzle
FENCE_H_EST = 1.2          # disc bottom at 1.67 above deck -> 0.47 clear
SENSE_R_EST = 2.9          # window center radius [doc]
WINDOW_ANGLES_EST = (130.0, 220.0)  # from mesh axis [doc]
SENSOR_L_EST = 3.2         # ITR8307 body, tangential (radial would
                           # overhang the PHI4 deck hole to r~1.3)
SENSOR_W_EST = 1.7         # ITR8307 body, radial
CAVITY_CLEAR_EST = 0.25    # per side around the body
LINTEL_Z_EST = 0.4         # inboard wall bottom: the keyhole slot opens
                           # into the PHI4 deck hole, so the inboard wall
                           # has no deck to stand on and spans as a
                           # lintel above the sensor face (0.25 proud)
BRIDGE_R_IN_EST = 2.65     # bridge stands on deck between the windows
BRIDGE_R_OUT_EST = 3.15
BRIDGE_ARC_EST = (150.0, 200.0)

BLACK = (0.15, 0.15, 0.15)


def _at(angle, solid):
    return Rot(0, 0, angle) * Pos(SENSE_R_EST, 0, 0) * solid


def _sector(r_in, r_out, a0, a1, h):
    f = Circle(r_out) - Circle(r_in)
    pts = [(0.0, 0.0)]
    steps = 8
    for i in range(steps + 1):
        a = math.radians(a0 + (a1 - a0) * i / steps)
        pts.append((10 * math.cos(a), 10 * math.sin(a)))
    return extrude(f & Polygon(*pts, align=None), h)


def build() -> Part:
    cav_x = SENSOR_W_EST + 2 * CAVITY_CLEAR_EST   # radial
    cav_y = SENSOR_L_EST + 2 * CAVITY_CLEAR_EST   # tangential
    shell = Box(cav_x + 2 * WALL_T_EST, cav_y + 2 * WALL_T_EST,
                FENCE_H_EST, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = _at(WINDOW_ANGLES_EST[0], shell) + _at(WINDOW_ANGLES_EST[1], shell)
    p += _sector(BRIDGE_R_IN_EST, BRIDGE_R_OUT_EST,
                 BRIDGE_ARC_EST[0], BRIDGE_ARC_EST[1], FENCE_H_EST)
    for a in WINDOW_ANGLES_EST:
        # cavities cut after the union so no wall invades a neighbor
        p -= _at(a, Box(cav_x, cav_y, 4 * FENCE_H_EST))
        p -= _at(a, Pos(-(cav_x + WALL_T_EST) / 2, 0, 0) * Box(
            WALL_T_EST + 0.02, cav_y + 2 * WALL_T_EST + 0.02,
            2 * LINTEL_Z_EST))
    p.label = "sg90_fence"
    p.color = Color(*BLACK)
    return p


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "sg90_fence.step"))
    export_stl(part, str(out / "sg90_fence.stl"), tolerance=0.01)
    from blueprint import blueprint
    blueprint(part, str(out / "sg90_fence_bp.svg"), name="sg90_fence")
    print(f"sg90_fence: volume {part.volume:.0f} mm^3 -> {out}")

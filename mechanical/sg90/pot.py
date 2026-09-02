"""SG90 feedback pot. Axis = Z, back face at z=0, shaft up, wiper tab at -Y."""
import math

from build123d import *

from . import measurements as m


def _polar(r, deg):
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def _terminal() -> Part:
    # built pointing -Y (wiper position), caller rotates
    tab = extrude(Pos(0, -m.POT_LEG_R - 0.1, 0.05) * RectangleRounded(
        m.POT_TAB_W, m.POT_TAB_L_EST, 0.5), m.POT_TAB_T)
    tab -= Pos(0, -m.POT_LEG_R, 0) * Cylinder(m.POT_TAB_HOLE / 2, 2)
    leg = Pos(0, -m.POT_FORMED_LEG_R, m.POT_TAB_T + 0.05) * Box(
        m.POT_LEG_W_EST, m.POT_TAB_T, m.POT_LEG_L,
        align=(Align.CENTER, Align.CENTER, Align.MAX))
    return tab + leg


def _wave_washer() -> Part:
    rm = (m.POT_WASHER_OD_EST + m.POT_WASHER_ID_EST) / 4
    amp = m.POT_WASHER_WAVE_EST
    pts = [Vector(rm * math.cos(a), rm * math.sin(a), amp * math.sin(3 * a))
           for a in (i * 2 * math.pi / 36 for i in range(36))]
    path = Spline(*pts, periodic=True)
    sec = (path ^ 0) * Rectangle(
        (m.POT_WASHER_OD_EST - m.POT_WASHER_ID_EST) / 2, m.POT_WASHER_T_EST)
    return sweep(sec, path=path)


def build() -> Compound:
    floor_z = m.POT_FLOOR_T_EST
    front_z = m.POT_BODY_L
    tip_z = front_z + m.POT_FRONT_SHAFT_L_EST
    bush_top = floor_z + m.POT_BUSH_H_EST

    # --- body: cup open to the front ---
    body = Cylinder(m.POT_BODY_D / 2, m.POT_BODY_L,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    body -= Pos(0, 0, floor_z) * Cylinder(
        m.POT_BODY_D / 2 - m.POT_WALL_EST, m.POT_BODY_L,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    body -= Cylinder(m.POT_RECESS_D_EST / 2, 2 * m.POT_RECESS_DEPTH_EST)
    body -= Cylinder(m.POT_SHAFT_D / 2 + 0.05, 2 * m.POT_BODY_L)
    for s in (1, -1):  # mold holes mirror the outer legs about +Y
        x, y = _polar(m.POT_LEG_R, 90 + s * m.POT_LEG_ANGLE)
        body -= Pos(x, y, 0) * Cylinder(
            m.POT_MOLD_HOLE_D_EST / 2, 2 * m.POT_MOLD_HOLE_DEPTH_EST)
    for a in (0, m.POT_LEG_ANGLE, -m.POT_LEG_ANGLE):  # tab exit channels
        body -= Rot(0, 0, a) * Pos(0, -3.8, 0) * Box(
            m.POT_TAB_W + 0.4, 2.9, 2 * (m.POT_TAB_T + 0.15))

    # --- shaft: rivet flare -> through body -> front stub; brass bushing ---
    # flare top flush with the recess floor, face slightly sub-flush
    shaft = Pos(0, 0, 0.05) * Cylinder(
        m.POT_SHAFT_D / 2, tip_z - 0.05,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    shaft += Pos(0, 0, 0.05) * Cylinder(
        m.POT_RIVET_D / 2, m.POT_RECESS_DEPTH_EST - 0.05,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    shaft += Pos(0, 0, floor_z) * Cylinder(
        m.POT_BUSH_D_EST / 2, m.POT_BUSH_H_EST,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    shaft += Pos(0, 0, floor_z) * Cylinder(
        m.POT_BUSH_FLANGE_D_EST / 2, 0.15,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    shaft -= Pos(0, 0, tip_z) * Box(
        m.POT_SHAFT_SLOT_W_EST, m.POT_SHAFT_D + 1,
        2 * m.POT_SHAFT_SLOT_DEPTH_EST)

    # --- track: horseshoe on the cup floor, gap centered at -Y ---
    # ends land on the outer-terminal rivets, so gap = 2 * leg angle
    ann = Circle(m.POT_TRACK_OD_EST / 2) - Circle(m.POT_TRACK_ID_EST / 2)
    gap = Polygon((0, 0), _polar(10, -90 - m.POT_LEG_ANGLE), (0, -10),
                  _polar(10, -90 + m.POT_LEG_ANGLE), align=None)
    track = extrude(Pos(0, 0, floor_z) * (ann - gap), m.POT_TRACK_T_EST)

    # --- rotor: rounded-square disc + front hub, counterbored over bushing ---
    disc_h = m.POT_ROTOR_H_EST - 1.0
    rotor = extrude(Pos(0, 0, 1.45) * RectangleRounded(
        m.POT_ROTOR_D_EST, m.POT_ROTOR_D_EST, 2.0), disc_h)
    rotor += Pos(0, 0, 1.45 + disc_h) * Cylinder(
        m.POT_ROTOR_HUB_D_EST / 2, 1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    rotor -= Pos(0, 0, 1.45) * Cylinder(
        m.POT_BUSH_D_EST / 2 + 0.15, bush_top - 1.45 + 0.05,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    rotor -= Cylinder(m.POT_SHAFT_D / 2 + 0.05, 20)

    # --- wiper: base plate under the rotor, 3 fingers on the track,
    #     2 on the bushing flange; rides at +Y, opposite the track gap ---
    track_mid = (m.POT_TRACK_OD_EST + m.POT_TRACK_ID_EST) / 4
    wiper = Pos(0, 2.1, 1.375) * Box(2.4, 1.4, 0.15)
    outer = Pos(0, track_mid + 0.15, 1.25) * Rot(-11.5, 0, 0) * Box(
        0.28, 1.5, 0.09)
    for a in (0, 13, -13):
        wiper += Rot(0, 0, a) * outer
    for sx in (1, -1):
        wiper += Pos(sx * 0.35, 1.9, 1.24) * Rot(13, 0, 0) * Box(
            0.28, 0.7, 0.09)

    # --- terminals: wiper at -Y, outers at +/-48 deg ---
    terminals = Part()
    for a in (0, m.POT_LEG_ANGLE, -m.POT_LEG_ANGLE):
        terminals += Rot(0, 0, a) * _terminal()

    washer = _wave_washer()
    washer_a = Pos(0, 0, front_z + 0.45) * washer
    washer_b = Pos(0, 0, front_z + 1.05) * washer

    for p, label, color in (
            (body, "body", (0.10, 0.10, 0.10)),
            (shaft, "shaft", (0.72, 0.58, 0.28)),
            (rotor, "rotor", (0.92, 0.92, 0.88)),
            (wiper, "wiper", (0.78, 0.78, 0.80)),
            (track, "track", (0.25, 0.24, 0.23)),
            (terminals, "terminals", (0.85, 0.70, 0.30)),
            (washer_a, "washer_wave", (0.35, 0.40, 0.55)),
            (washer_b, "washer_wave", (0.75, 0.75, 0.78))):
        p.label = label
        p.color = Color(*color)
    return Compound(label="sg90_pot", children=[
        body, shaft, rotor, wiper, track, terminals, washer_a, washer_b])


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "sg90_pot.step"))
    export_stl(part, str(out / "sg90_pot.stl"), tolerance=0.01)
    from blueprint import blueprint
    blueprint(part, str(out / "sg90_pot_bp.svg"), name="sg90_pot")
    print(f"sg90_pot: volume {part.volume:.0f} mm^3 -> {out}")

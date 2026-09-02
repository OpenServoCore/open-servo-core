"""Carrier stator: pot-envelope body, bearing bore, arms, coupon shelf."""
import math

from build123d import *

from . import params as p


def _sector(r_in, r_out, half_deg):
    # annular sector opening toward +Y
    a = math.radians(half_deg)
    span = 2 * r_out
    wedge = Polygon((0, 0), (span * math.sin(a), span * math.cos(a)),
                    (-span * math.sin(a), span * math.cos(a)), align=None)
    return (Circle(r_out) - Circle(r_in)) & wedge


def _under_structure() -> Part:
    """Arc shells + ledge + bayonet features below the body.

    Everything except the snap hooks stays inside OPENING_R so the part
    inserts through the shell floor opening; the hooks ride on slotted
    fingers (the arcs themselves are hoop-stiff and cannot flex).
    """
    shell = Pos(0, 0, p.ARM_BOT_Z) * extrude(
        Circle(p.ARM_R_OUT) - Circle(p.ARM_R_IN), -p.ARM_BOT_Z)
    # board seat: underside at COUPON_TOP_Z sets the sensor gap
    shell += Pos(0, 0, p.COUPON_TOP_Z) * extrude(
        Circle(p.ARM_R_IN + 0.05) - Circle(p.LEDGE_R_IN), p.LEDGE_H)
    # tab gaps at +/-X, full height
    for a in (-90, 90):
        shell -= Rot(0, 0, a) * Pos(0, 0, p.ARM_BOT_Z - 1) * extrude(
            _sector(p.LEDGE_R_IN - 0.3, p.ARM_R_OUT + 1, p.GAP_HALF_DEG),
            -p.ARM_BOT_Z + 2)
    # bayonet bands: tabs twist from the gaps toward the end stops
    for a in (-90, 90):
        shell -= Rot(0, 0, a) * Pos(0, 0, p.BAY_BOT_Z) * extrude(
            _sector(p.ARM_R_IN - 0.1, p.ARM_R_OUT + 1, p.BAY_HALF_DEG),
            p.BAY_TOP_Z - p.BAY_BOT_Z)
    # free a snap finger out of each arc at +/-Y
    for s in (1, -1):
        for sx in (1, -1):
            shell -= Pos(sx * (p.FINGER_W / 2 + p.FINGER_SLOT / 2),
                         s * 3.85, p.FINGER_SLOT_BOT_Z) * Box(
                p.FINGER_SLOT, 1.4, -p.FINGER_SLOT_BOT_Z + 1,
                align=(Align.CENTER, Align.CENTER, Align.MIN))
        shell -= Pos(0, s * 3.85, -0.40) * Box(
            p.FINGER_W + 2 * p.FINGER_SLOT, 1.4, 0.38,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        shell += Pos(0, s * (p.ARM_R_OUT + p.HOOK_T / 2),
                     p.HOOK_TOP_Z - p.HOOK_H / 2) * Box(
            2.0, p.HOOK_T, p.HOOK_H)
    # detents behind the seated tab's trailing edge (tab spans 20-50 deg
    # at the end stop); crush-height, the click is a light yield
    for a in (18, 198):
        shell += Rot(0, 0, a) * Pos(3.85, 0, p.BAY_BOT_Z) * Box(
            0.4, 0.5, 0.05, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return shell


def build() -> Part:
    body = Cylinder(p.BODY_D / 2, p.BODY_H,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    # printed gear2 thrust seat; z-exact, so the tube can press in flush
    body += Pos(0, 0, p.BODY_H) * Cylinder(
        p.SEAT_BOSS_D / 2, p.SEAT_BOSS_H_EST,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    body -= Cylinder((p.BUSH_OD - p.BUSH_PRESS) / 2, 3 * p.BODY_H)
    # ear insets: open to the rim so they drop over the shell ears
    for s in (1, -1):
        body -= Pos(s * (p.BODY_D / 2 - 1.9 + p.EAR_INSET_W / 2), 0, 0) * Box(
            p.EAR_INSET_W + 1.0, p.EAR_INSET_L, 2 * p.EAR_INSET_DEPTH)
    body += _under_structure()
    return body


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "carrier_body.step"))
    export_stl(part, str(out / "carrier_body.stl"), tolerance=0.01)
    print(f"carrier_body: volume {part.volume:.0f} mm^3 -> {out}")

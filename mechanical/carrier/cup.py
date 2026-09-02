"""Magnet cup rotor: crush-rib grip on the pin, snap-captive magnet.

Pocket opens DOWN: magnet clicks past three nubs and sits captive
between the pocket ceiling and the nubs; bare magnet face toward the
sensor, no printed floor in the magnetic gap.
"""
from build123d import *

from . import params as p


def build() -> Part:
    cup = Pos(0, 0, p.CUP_RIM_Z) * Cylinder(
        p.CUP_OD / 2, p.CUP_TOP_Z - p.CUP_RIM_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    cup += Pos(0, 0, p.CUP_TOP_Z) * Cylinder(
        p.CUP_HUB_D / 2, p.CUP_HUB_TOP_Z - p.CUP_TOP_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    web_bot = p.MAG_TOP_Z + p.CUP_BUMP_H
    # pin bore, ribs added back after the cut
    cup -= Pos(0, 0, web_bot) * Cylinder(
        (p.PIN_D + 0.1) / 2, p.CUP_HUB_TOP_Z - web_bot + 1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    # magnet pocket, up to the web underside
    cup -= Pos(0, 0, web_bot) * Cylinder(
        (p.MAG_D + 0.1) / 2, web_bot - p.CUP_RIM_Z + 1,
        align=(Align.CENTER, Align.CENTER, Align.MAX))
    # three crush ribs in the pin bore
    rib_h = p.CUP_HUB_TOP_Z - web_bot
    for a in (0, 120, 240):
        rib_mid = (p.PIN_D - p.CUP_RIB_GRIP + p.PIN_D + 0.2) / 4
        cup += Rot(0, 0, a) * Pos(0, rib_mid, web_bot) * Box(
            0.45, (p.PIN_D + 0.2) / 2 - (p.PIN_D - p.CUP_RIB_GRIP) / 2 + 0.1,
            rib_h, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # ceiling crush bumps: yield under the anvil press, clamp the magnet
    for a in (0, 120, 240):
        cup += Rot(0, 0, a) * Pos(0, 0.9, web_bot) * Box(
            0.5, 0.5, p.CUP_BUMP_H,
            align=(Align.CENTER, Align.CENTER, Align.MAX))
    # petal-spring radial ribs on the pocket wall, one per petal
    for a in (60, 180, 300):
        rib_mid = (p.MAG_D - p.CUP_POCKET_RIB + p.MAG_D + 0.1) / 4
        cup += Rot(0, 0, a) * Pos(0, rib_mid, web_bot) * Box(
            0.5, (p.MAG_D + 0.1) / 2 - (p.MAG_D - p.CUP_POCKET_RIB) / 2 + 0.1,
            web_bot - (p.CUP_RIM_Z + 0.3),
            align=(Align.CENTER, Align.CENTER, Align.MAX))
    # three snap nubs at the pocket mouth
    nub_top = p.CUP_RIM_Z + 0.05 + p.CUP_NUB_H
    for a in (60, 180, 300):
        nub_mid = (p.CUP_NUB_D + p.MAG_D + 0.1) / 4
        cup += Rot(0, 0, a) * Pos(0, nub_mid, nub_top) * Box(
            0.9, (p.MAG_D + 0.1) / 2 - p.CUP_NUB_D / 2 + 0.1, p.CUP_NUB_H,
            align=(Align.CENTER, Align.CENTER, Align.MAX))
    # petal slots between the nubs: rim -> just under the pocket ceiling
    for a in (0, 120, 240):
        cup -= Rot(0, 0, a) * Pos(0, (p.MAG_D + p.CUP_OD) / 4,
                                  p.CUP_RIM_Z - 0.5) * Box(
            p.CUP_SLOT_W, p.CUP_OD / 2, p.MAG_TOP_Z - 0.1 - p.CUP_RIM_Z + 0.5,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    return cup


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "carrier_cup.step"))
    export_stl(part, str(out / "carrier_cup.stl"), tolerance=0.01)
    print(f"carrier_cup: volume {part.volume:.1f} mm^3 -> {out}")

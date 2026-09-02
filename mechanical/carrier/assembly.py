"""Full tier-2 stack: carrier + rotor + purchased parts + gear references."""
from build123d import *

from sg90 import measurements as m
from . import params as p
from . import body as body_mod
from . import cup as cup_mod


def _cyl(r, z0, z1):
    return Pos(0, 0, z0) * Cylinder(
        r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def build() -> Compound:
    body = body_mod.build()
    cup = cup_mod.build()
    pin = _cyl(p.PIN_D / 2, p.PIN_BOT_Z, p.PIN_TOP_Z)
    # jig-filed D-flat on the gear4 zone (form closure, protocol above)
    pin -= Pos(p.PIN_FLAT_ACROSS - p.PIN_D / 2, 0,
               p.PIN_TOP_Z - p.PIN_FLAT_L) * Box(
        p.PIN_D, p.PIN_D + 1, p.PIN_FLAT_L + 1,
        align=(Align.MIN, Align.CENTER, Align.MIN))
    bush = _cyl(p.BUSH_OD / 2, p.BUSH_Z, p.BUSH_Z + p.BUSH_L) \
        - _cyl(p.BUSH_ID / 2, p.BUSH_Z - 1, p.BUSH_Z + p.BUSH_L + 1)
    magnet = _cyl(p.MAG_D / 2, p.MAG_TOP_Z - p.MAG_H, p.MAG_TOP_Z)

    # round board shown in the installed position: tabs twisted 35 deg
    coupon = _cyl(p.COUPON_D / 2, p.COUPON_TOP_Z - p.COUPON_T,
                  p.COUPON_TOP_Z)
    for a in (35, 215):
        coupon += Rot(0, 0, a) * Pos(
            (p.COUPON_D / 2 + p.TAB_R_OUT) / 2 - 0.05, 0,
            p.COUPON_TOP_Z - p.COUPON_T) * Box(
            p.TAB_R_OUT - p.COUPON_D / 2 + 0.3, p.TAB_W, p.COUPON_T,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    sensor = Pos(0, 0, p.COUPON_TOP_Z) * Box(
        p.SENSOR_SZ, p.SENSOR_SZ, p.SENSOR_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN))

    # gear references: plain cylinders, positions approximate
    g2_z = p.GEAR2_SEAT_Z
    g4_z = g2_z + m.GEAR2_TOTAL_H + 0.1
    sleeve = _cyl(p.G2_SLEEVE_OD / 2, g2_z, g4_z) \
        - _cyl(p.G2_SLEEVE_ID / 2, g2_z - 1, g4_z + 1)
    gear2 = _cyl(m.GEAR2_D / 2, g2_z, g2_z + m.GEAR2_H)
    gear2 += _cyl(m.GEAR2_PINION_D / 2, g2_z + m.GEAR2_H,
                  g2_z + m.GEAR2_TOTAL_H)
    gear2 -= _cyl(p.G2_SLEEVE_OD / 2 + 0.05, g2_z - 1,
                  g2_z + m.GEAR2_TOTAL_H + 1)
    gear4 = _cyl(m.GEAR4_D / 2, g4_z, g4_z + m.GEAR4_H)
    gear4 += _cyl(m.GEAR4_BOSS_D / 2, g4_z + m.GEAR4_H,
                  g4_z + m.GEAR4_TOTAL_H)
    gear4 -= _cyl(p.PIN_D / 2, g4_z - 1, g4_z + p.GEAR4_BORE_DEPTH_EST)

    for part, label, color in (
            (body, "body", (0.20, 0.25, 0.35)),
            (cup, "cup", (0.85, 0.45, 0.10)),
            (pin, "pin", (0.75, 0.75, 0.78)),
            (sleeve, "g2_sleeve", (0.80, 0.66, 0.35)),
            (bush, "bushing", (0.72, 0.58, 0.28)),
            (magnet, "magnet", (0.35, 0.35, 0.38)),
            (coupon, "coupon", (0.15, 0.45, 0.25)),
            (sensor, "sensor", (0.10, 0.10, 0.10)),
            (gear2, "gear2_ref", (0.88, 0.88, 0.84)),
            (gear4, "gear4_ref", (0.88, 0.88, 0.84))):
        part.label = label
        part.color = Color(*color)
    return Compound(label="tier2_carrier", children=[
        body, cup, pin, sleeve, bush, magnet, coupon, sensor, gear2, gear4])


def build_section() -> Compound:
    """+X half removed, for reviewing the internal stack."""
    half = Pos(15, 0, 0) * Box(30, 30, 60)
    kids = []
    for child in build().children:
        cut = child - half
        cut.label, cut.color = child.label, child.color
        kids.append(cut)
    return Compound(label="tier2_carrier_section", children=kids)


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "carrier_assembly.step"))
    export_stl(part, str(out / "carrier_assembly.stl"), tolerance=0.01)
    from blueprint import blueprint
    blueprint(part, str(out / "carrier_assembly_bp.svg"),
              name="carrier_assembly")
    sec = build_section()
    export_step(sec, str(out / "carrier_section.step"))
    export_stl(sec, str(out / "carrier_section.stl"), tolerance=0.01)
    blueprint(sec, str(out / "carrier_section_bp.svg"),
              name="carrier_section")
    print(f"carrier_assembly + carrier_section -> {out}")

"""Fastener clamp-stack audit for stenciljig: between screw head and
nut (or formed threads), EVERY part the joint joins must sit inside
the clamp stack, and the nut must react against a part OTHER than
the one the head bears on - a nut that reacts against the head's own
part clamps nothing else. Two ~20h bases were printed with exactly
that bug (clamp nuts roofed by the clamp plate; foot nuts roofed by
the foot itself), so this runs alongside check_all before any host
reprint.

Per joint:
1. completeness: a head-diameter column between the bearing planes
   intersects every joined part
2. reaction: the nut displaced 0.4 along the screw-tension direction
   bears the intended reaction part and NOT the head-bearing part;
   tap joints instead require thread-envelope bite in the reaction
   part and zero bite in the head part
Run from mechanical/: PYTHONPATH=. .venv/bin/python check_joints.py"""
from build123d import *

from stenciljig.adapter import build_adapter, AD_CBORE
from stenciljig.base import build_base
from stenciljig.box import build_box, BOX_FOOT_POSE, BOX_H
from stenciljig.clamp_plate import build_clamp_plate
from stenciljig.common import (_hex, foot_pt, AD_SCREW_Y, BASE_T,
                               BTN_CBORE, CLAMP_HOLES, CLAMP_T,
                               FOOT_POSE, FOOT_SCREW_OFF, LEDGE,
                               M3_NUT_T, NUT_AF, PCB_T, PCB_TOP,
                               PCB_W, SIDE_W, SLOT_X0, SLOT_Y,
                               TONGUE_CLEAR)
from stenciljig.foot import build_foot
from stenciljig.knob import (build_handle, build_rod, SEAT_Z,
                             TAP_ENGAGE, TIP_Z)
from stenciljig.nutbar import build_nutbar
from stenciljig.side_plate import build_side_plate, EAR_T

base = build_base()
plate = build_side_plate()
ad = build_adapter(1.6, 1.0)
box = build_box()
clamp = build_clamp_plate()
bar = build_nutbar()
foot, hookfoot = build_foot(), build_foot(True)
rod, handle = build_rod(), build_handle()

HEAD_R = 2.85              # button head bearing radius
THREAD_R = 1.6             # M3 thread envelope


def vol(a):
    try:
        return a.volume if a.solids() else 0.0
    except Exception:
        return 0.0


def column(x, y, z0, z1):
    return Pos(x, y, min(z0, z1)) * Cylinder(
        HEAD_R, abs(z1 - z0),
        align=(Align.CENTER, Align.CENTER, Align.MIN))


def check_stack(name, col, stack):
    for label, part in stack:
        v = vol(col & part)
        assert v > 0.01, (name, label, "NOT in the clamp stack")


def nut_joint(name, x, y, seat, head_z, stack, head, reaction):
    """seat = nut top face z; head_z = head bearing plane z."""
    check_stack(name, column(x, y, seat, head_z), stack)
    nut = Pos(x, y, seat - M3_NUT_T) * extrude(_hex(NUT_AF), M3_NUT_T)
    d = 0.4 if head_z > seat else -0.4     # tension pulls nut headward
    shifted = Pos(0, 0, d) * nut
    vr, vh = vol(shifted & reaction[1]), vol(shifted & head[1])
    assert vr > 0.1, (name, "nut does not bear on", reaction[0])
    assert vh == 0.0, (name, "nut reacts against the head's own part",
                       head[0])
    print(f"{name:24s} stack {'+'.join(l for l, _ in stack)}, "
          f"nut reacts on {reaction[0]}")


def tap_joint(name, x, y, z0, z1, head_z, stack, head, reaction):
    """z0..z1 = thread engagement band in the reaction part."""
    check_stack(name, column(x, y, head_z, z1), stack)
    bite_cyl = Pos(x, y, z0) * Cylinder(
        THREAD_R, z1 - z0,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    vr, vh = vol(bite_cyl & reaction[1]), vol(bite_cyl & head[1])
    assert vr > 4.0, (name, "threads do not bite", reaction[0], vr)
    assert vh == 0.0, (name, "threads bite the head's own part",
                       head[0])
    print(f"{name:24s} stack {'+'.join(l for l, _ in stack)}, "
          f"taps {reaction[0]} ({vr:.1f} mm^3)")


# side-plate ear screws -> nut bars (bar nut pressed up under its web)
for y in SLOT_Y:
    nut_joint(f"ear y={y:g}", SLOT_X0, y, -2.0, BASE_T + EAR_T,
              (("plate", plate), ("base", base), ("bar", bar)),
              ("plate", plate), ("bar", bar))

# plank screws -> captive plate nuts
xc = PCB_W / 2 - LEDGE + SIDE_W / 2
for y in AD_SCREW_Y:
    nut_joint(f"plank y={y:g}", xc, y, 14.3, PCB_TOP - AD_CBORE,
              (("plank", ad), ("plate", plate)),
              ("plank", ad), ("plate", plate))

# clamp screws -> bottom-entry base nuts (centre hangs from the
# tongue groove floor)
gfloor = PCB_TOP - PCB_T - TONGUE_CLEAR
for x, y in CLAMP_HOLES:
    top = gfloor if x == 0.0 else PCB_TOP
    nut_joint(f"clamp ({x:g},{y:g})", x, y, top - 2.6,
              PCB_TOP + CLAMP_T - BTN_CBORE,
              (("clamp", clamp), ("base", base)),
              ("clamp", clamp), ("base", base))

# foot screws thread-form the host pilots, jig then box
for x, y, a, hook in FOOT_POSE:
    f = Pos(x, y, 0) * Rot(0, 0, a) * (hookfoot if hook else foot)
    sx, sy = foot_pt(x, y, a, FOOT_SCREW_OFF)
    tap_joint(f"foot ({x:g},{y:g})", sx, sy, 0.4, 5.3, -4.7,
              (("foot", f), ("base", base)),
              ("foot", f), ("base", base))
for x, y, a in BOX_FOOT_POSE:
    f = Pos(x, y, -BOX_H) * Rot(0, 0, a) * foot
    sx, sy = foot_pt(x, y, a, FOOT_SCREW_OFF)
    tap_joint(f"boxfoot ({x:g},{y:g})", sx, sy, -BOX_H + 0.4,
              -BOX_H + 5.3, -BOX_H - 4.7,
              (("foot", f), ("box", box)),
              ("foot", f), ("box", box))

# knob spine self-taps the rod's wing root
tap_joint("knob spine", 0, 0, TIP_Z, TIP_Z + TAP_ENGAGE, SEAT_Z,
          (("handle", handle), ("rod", rod)),
          ("handle", handle), ("rod", rod))

print("ALL JOINTS SOUND")

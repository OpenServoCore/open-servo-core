"""Full boolean check suite for the refactored stenciljig package:
jig assembly (plate/adapter/board both edges), plank screw + captive
nut path, center nut vs tongue, the two-part stack (feet, hooks,
cam), box tenants. Everything asserts; run from mechanical/ with
PYTHONPATH=. .venv/bin/python <this>."""
from build123d import *

from stenciljig.adapter import build_adapter, AD_CBORE
from stenciljig.box import (build_box, AD_D, AD_XS, AD_SLOT_W,
                            AD_SLOTS, AD_Z, BAY_D, BAY_Y1, BOX_H,
                            EXT, FLOOR_Z, HW_BIN, PKT, SHEET,
                            SHEET_Z, SQ_XY0)
from stenciljig.base import build_base
from stenciljig.clamp_plate import build_clamp_plate
from stenciljig.common import (_hex, foot_pt, AD_SCREW_Y, BASE_D,
                               BASE_T, BORDER, BTN_H, CAM_XY, CAP_T,
                               COL_X, COL_Y, FOOT_H, FOOT_INSET,
                               FOOT_POSE, FOOT_SCREW_OFF, LEDGE,
                               LEDGE_PROUD, M3_NUT_T, MID_Y, NUT_AF,
                               PCB_D, PCB_T, PCB_TOP, PCB_W, PLATE_H,
                               PLATE_YC, RAIL_DROP, SIDE_W,
                               SLOT_LEN, TONGUE_CLEAR)
from stenciljig.foot import build_foot
from stenciljig.knob import (build_handle, build_knob, build_rod,
                             SEAT_Z, TAP_ENGAGE, TIP_Z)
from stenciljig.nutbar import build_nutbar
from stenciljig.pad import build_pad, PAD_PROUD
from stenciljig.side_plate import EAR_L, EAR_T
from stenciljig.squeegee import build_squeegee, SQ_SIZES
from stenciljig.tongue import build_tongue
from stenciljig.tray import build_tray, TRAY_FLOOR

from stenciljig.side_plate import build_side_plate

base = build_base()
plate = build_side_plate()
ad = build_adapter(1.6, 1.0)
box = build_box()
knob = build_knob()
rod, handle = build_rod(), build_handle()
foot, hookfoot = build_foot(), build_foot(True)
bar = build_nutbar()
pad = build_pad()
tongue = build_tongue()
clamp = build_clamp_plate()
tray = build_tray()


def vol(a):
    try:
        return a.volume if a.solids() else 0.0
    except Exception:
        return 0.0


for part in (base, plate, ad, box, knob, rod, handle, foot, hookfoot,
             bar, pad, tongue, clamp, tray):
    n = len(part.solids())
    assert n == 1, (part.label, n)
print("connectivity:           every part is 1 solid")

xc = (PCB_W / 2 - LEDGE) + SIDE_W / 2
face_x = PCB_W / 2 - LEDGE + SIDE_W

# --- jig assembly (old check_jig) ---
# self-rotation lands the ears inboard and the nut-channel mouths
# outboard, so only the BODY is self-symmetric; the working
# both-hands proof is the left-station pose further down
q = Pos(2 * xc, 2 * PLATE_YC, 0) * Rot(0, 0, 180) * plate
d = vol(plate - q) + vol(q - plate)
assert d < 3200.0, d
adflip = Pos(2 * xc, 2 * PLATE_YC, 0) * Rot(0, 0, 180) * ad
for name, a, b in (("plate^base", plate, base),
                   ("adapter^plate", ad, plate),
                   ("adapter^base", ad, base),
                   ("adflip^plate", adflip, plate),
                   ("plate^base mid", Pos(-25, 0, 0) * plate, base),
                   ("plate^base closed", Pos(-SLOT_LEN, 0, 0) * plate,
                    base),
                   ("plate^base out", Pos(30, 0, 0) * plate, base)):
    v = vol(a & b)
    assert v == 0.0, (name, v)
v = vol(Pos(0, 0, 1) * plate & base)
assert v > 0.0, v
lift = Pos(0, 0, RAIL_DROP + 0.2) * plate
assert vol(lift & base) > 0.0
left = Pos(0, PLATE_YC, 0) * Rot(0, 0, 180) * Pos(0, -PLATE_YC, 0) * plate
assert vol(left & base) == 0.0
assert vol(Pos(0, 0, RAIL_DROP + 0.2) * left & base) > 0.0
probe = Pos(face_x + EAR_L / 2, PLATE_YC, BASE_T + 1.0) * Box(
    4, 4, EAR_T - 1.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
assert vol(probe & plate) == 0.0
# fully closed: both plates ride to the centerline, faces kiss
rc = Pos(-SLOT_LEN, 0, 0) * plate
lc = Pos(SLOT_LEN, 0, 0) * Pos(0, 2 * PLATE_YC, 0) * Rot(0, 0, 180) \
    * plate
v = vol(rc & lc)
assert v < 0.05, v
for t, meas, a in ((1.6, 1.579, ad), (1.0, 0.98, adflip)):
    board = Pos(0, BORDER + PCB_D / 2, PCB_TOP - (t - LEDGE_PROUD)) * Box(
        100, 100, meas, align=(Align.CENTER, Align.CENTER, Align.MIN))
    assert vol(board & a) == 0.0 and vol(board & plate) == 0.0
    assert vol(Pos(0, -30, 0) * board & a) == 0.0
print("jig assembly:           flip/slide/lift-block/board both edges ok")

# --- plank screws: shank + head path, nut seats press, hopper holds ---
floor_z = 14.3 - M3_NUT_T          # nut seated under the hopper
for y in AD_SCREW_Y:
    shank = Pos(xc, y, PCB_TOP - AD_CBORE - 10.0) * Cylinder(
        1.5, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    head = Pos(xc, y, PCB_TOP - AD_CBORE) * Cylinder(
        2.85, BTN_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    nut = Pos(xc, y, floor_z) * extrude(_hex(NUT_AF), M3_NUT_T)
    v = vol(shank & ad) + vol(shank & plate) + vol(head & ad) \
        + vol(nut & ad)
    assert v == 0.0, (y, v)
    bite = vol(nut & plate)            # press band: exact at nominal
    assert bite < 0.05, (y, bite)
    drop = vol(Pos(0, 0, -4.0) * nut & plate)  # loose shaft passes it
    assert drop < 0.05, (y, drop)
    roof = vol(Pos(0, 0, 1.2) * nut & plate)   # hopper blocks pull-up
    assert roof > 2.0, (y, roof)
    tap = vol(shank & nut)
    assert tap > 15.0, (y, tap)
print("plank screws:           M3x10 path clear, nut seats press, "
      "hopper retains")

# --- clamp nuts: bottom-entry seats, hoppers hold, screws + tongue ---
gfloor = PCB_TOP - PCB_T - TONGUE_CLEAR
for (x, y), top in (((0.0, MID_Y), gfloor),
                    ((COL_X, COL_Y[0]), PCB_TOP),
                    ((-COL_X, COL_Y[1]), PCB_TOP)):
    seat = top - 2.6
    nut = Pos(x, y, seat - M3_NUT_T) * extrude(_hex(NUT_AF), M3_NUT_T)
    v = vol(nut & base)
    assert v < 0.05, (x, y, v)         # press band: exact at nominal
    drop = vol(Pos(0, 0, -6.0) * nut & base)   # loose shaft passes it
    assert drop < 0.05, (x, y, drop)
    roof = vol(Pos(0, 0, 1.2) * nut & base)    # hopper blocks pull-up
    assert roof > 2.0, (x, y, roof)
    # M3x16 from the clamp seat: shank clear down to 0.5 past the nut
    shank = Pos(x, y, seat - M3_NUT_T - 0.5) * Cylinder(
        1.5, PCB_TOP + 6.0 - (seat - M3_NUT_T - 0.5),
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    vs = vol(shank & base) + vol(shank & clamp)
    assert vs == 0.0, (x, y, vs)
assert vol(tongue & base) == 0.0 and vol(clamp & base) == 0.0
cnut = Pos(0, MID_Y, gfloor - 2.6 - M3_NUT_T) * extrude(_hex(NUT_AF),
                                                        M3_NUT_T)
assert vol(tongue & cnut) == 0.0       # centre nut under groove floor
print("clamp nuts + tongue:    seat press, hoppers hold, screws "
      "clear, tongue passes over")

# --- feet mounted: pin seats, screw core clear, thread zone bites ---
worst = 0.0
posed_feet = []
for x, y, a, hook in FOOT_POSE:
    f = Pos(x, y, 0) * Rot(0, 0, a) * (hookfoot if hook else foot)
    posed_feet.append(f)
    worst = max(worst, vol(f & base))
    sx, sy = foot_pt(x, y, a, FOOT_SCREW_OFF)
    # M3x10 from below: head seat -4.7, tip 5.3 - core (minor dia)
    # passes free, thread envelope bites the host pilot
    shank = Pos(sx, sy, -4.6) * Cylinder(
        1.2, 9.9, align=(Align.CENTER, Align.CENTER, Align.MIN))
    head = Pos(sx, sy, -4.7 - BTN_H) * Cylinder(
        2.85, BTN_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    vp = vol(shank & base) + vol(shank & f) + vol(head & f)
    assert vp == 0.0, (x, y, vp)
    bite = vol(Pos(sx, sy, 0.4) * Cylinder(
        1.6, 4.9, align=(Align.CENTER, Align.CENTER, Align.MIN)) & base)
    assert bite > 4.0, (x, y, bite)
assert worst < 0.05, worst
print(f"4 feet mounted:         kiss max {worst:.4f}, taps bite, "
      f"M3x10 path clear")

# --- the stack: rig seated / tilted / hooked on the box ---
rig = base + posed_feet
v = vol(rig & box)
assert v == 0.0, v
print(f"rig seated on box:      {v:.4f} (want 0)")
tilt = Pos(0, 169.5, 0) * Rot(-1.5, 0, 0) * Pos(0, -169.5, 0) * rig
v = vol(tilt & box)
assert v == 0.0, v
print(f"rig tilted 1.5 deg:     {v:.4f} (want 0)")
grab = vol(Pos(0, 0, 1.0) * rig & box)
assert grab > 0.0, grab
print(f"hooks catch lift +1:    overlap {grab:.2f} > 0")

# --- cam knobs: entry drops free, locked seats under the tents ---
for s in (1, -1):
    at = Pos(s * CAM_XY[0], CAM_XY[1], BASE_T)
    ke, kl = at * knob, at * Rot(0, 0, -90) * knob
    khalf = at * Rot(0, 0, -45) * knob
    ve = vol(ke & base) + vol(ke & box)
    assert ve == 0.0, (s, ve)
    vl_ba, vl_bo = vol(kl & base), vol(kl & box)
    assert vl_ba == 0.0, (s, vl_ba)
    assert 0.02 < vl_bo < 5.0, (s, vl_bo)
    print(f"knob {s:+d}: entry free, mid {vol(khalf & box):.3f}, "
          f"locked bite {vl_bo:.3f} (want 0.02..5)")
at = Pos(CAM_XY[0], CAM_XY[1], BASE_T)
for k in (at * knob, at * Rot(0, 0, -90) * knob):
    v = vol(k & plate)
    assert v == 0.0, v
print("knob^plate max opening: 0 both poses")

# --- knob spine: boss fits, pilot clear, tap zone bites, head sinks ---
v = vol(rod & handle)
assert v == 0.0, v
pilot = Pos(0, 0, SEAT_Z) * Cylinder(
    1.4, SEAT_Z - TIP_Z + 0.1, align=(Align.CENTER, Align.CENTER, Align.MAX))
vp = vol(pilot & handle) + vol(pilot & rod)
assert vp == 0.0, vp
bite = Pos(0, 0, TIP_Z + TAP_ENGAGE) * Cylinder(
    1.6, TAP_ENGAGE, align=(Align.CENTER, Align.CENTER, Align.MAX))
vb = vol(bite & rod)
assert vb > 5.0, vb
headp = Pos(0, 0, SEAT_Z) * Cylinder(
    2.9, BTN_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
vh = vol(headp & handle)
assert vh == 0.0, vh
print(f"knob spine:             boss free, pilot clear, "
      f"tap bite {vb:.1f} mm^3, head sinks")

# --- nut-bar sweep over the stacked box ---
for s in (1, -1):
    for dx in (0.0, -SLOT_LEN):
        b = Pos(dx, 0, 0) * bar
        if s < 0:
            b = mirror(b, Plane.YZ)
        v = vol(b & box)
        assert v == 0.0, (s, dx, v)
print("nut bars ^box:          0 at both extremes, both sides")

# --- box tenants + stacked pads ---
pads = [Pos(x, y, -FOOT_H - PAD_PROUD) * pad for x, y, a, h in FOOT_POSE]
tenants = []
adl = Rot(0, 90, 0) * ad
bb = adl.bounding_box()
worst = 0.0
for x0 in AD_XS:
    q = Pos(x0 + PKT - bb.min.X,
            BAY_Y1 - AD_D + PKT - bb.min.Y, AD_Z - bb.min.Z) * adl
    worst = max(worst, vol(q & box))
    ptop = q.bounding_box().max.Z
    assert abs(ptop - (AD_Z + SIDE_W)) < 0.01, ptop
    tenants.append(q)
assert worst == 0.0, worst
print(f"{AD_SLOTS} planks in slots:      ^box 0, "
      f"tops at {AD_Z + SIDE_W:.1f}")
# tray on the bay's side ledges, flush with the rim
tp = Pos(0, BAY_Y1 - BAY_D / 2, FLOOR_Z) * tray
v = vol(tp & box)
assert v == 0.0, v
assert abs(tp.bounding_box().max.Z - (-13.0)) < 0.01
tenants.append(tp)
# a stack of 50x50 boards in the tray - tops must clear the
# stacked jig's base underside (z=0) with margin
stack = Pos(-25.0, BAY_Y1 - BAY_D / 2 - 25.0, FLOOR_Z + TRAY_FLOOR) * Box(
    50, 50, 16.0, align=(Align.MIN, Align.MIN, Align.MIN))
v = vol(stack & box) + vol(stack & tp)
assert v == 0.0, v
assert stack.bounding_box().max.Z <= -1.0
tenants.append(stack)
print("tray on ledges + 50x50 board stack inside: ^box 0, "
      "under the stacked base")
x0, y0 = SQ_XY0
for w, k in SQ_SIZES:
    sq = build_squeegee(w, k)
    bb = sq.bounding_box()
    q = Pos(x0 + PKT - bb.min.X, y0 + PKT - bb.min.Y,
            FLOOR_Z - bb.min.Z) * sq
    v = vol(q & box)
    assert v == 0.0, (w, v)
    tenants.append(q)
    x0 += (w - 1 + 2 * 2.5 + 2 * PKT) + 2.4
sheet = Pos(-BAY_D / 2 + PKT, BAY_Y1 - SHEET - PKT, SHEET_Z) * Box(
    SHEET, SHEET, 0.12, align=(Align.MIN, Align.MIN, Align.MIN))
assert vol(sheet & box) == 0.0 and vol(sheet & tp) == 0.0
tenants.append(sheet)
worst = 0.0
for q in pads:
    worst = max(worst, vol(q & box), *(vol(q & t) for t in tenants))
assert worst == 0.0, worst
print("squeegees+sheet stored; stacked pads clear box + tenants")

# --- box feet: same mount interface, mounted at the bottom face ---
from stenciljig.box import BOX_FOOT_POSE
worst = 0.0
for x, y, a in BOX_FOOT_POSE:
    f = Pos(x, y, -BOX_H) * Rot(0, 0, a) * foot
    worst = max(worst, vol(f & box))
    sx, sy = foot_pt(x, y, a, FOOT_SCREW_OFF)
    shank = Pos(sx, sy, -BOX_H - 4.6) * Cylinder(
        1.2, 9.9, align=(Align.CENTER, Align.CENTER, Align.MIN))
    vp = vol(shank & box) + vol(shank & f)
    assert vp == 0.0, (x, y, vp)
    bite = vol(Pos(sx, sy, -BOX_H + 0.4) * Cylinder(
        1.6, 4.9, align=(Align.CENTER, Align.CENTER, Align.MIN)) & box)
    assert bite > 4.0, (x, y, bite)
assert worst < 0.05, worst
print(f"4 box feet mounted:     kiss max {worst:.4f}, taps bite")

# --- pads seated in the feet, near-zero kiss ---
for f in (foot, hookfoot):
    q = Pos(0, 0, -FOOT_H - PAD_PROUD) * pad
    v = vol(q & f)
    assert v < 0.05, (f.label, v)
print("pads seated in feet:    kiss < 0.05 (snap fit)")

print("ALL GREEN")

"""Open tool tote the jig stacks onto - two parts, side by side on
the bench, locked together for storage and transport:
- BOX: an open tub sharing the base's plan (base.plan() offset by
  the wall = the interior, minus the gusset footprints = the cavity,
  so nothing can poke past the outer rounds). The pocketed deck
  inside holds every loose tool in reach while the jig works next to
  it. The jig's base seats on the wall tops as the lid, full
  perimeter plus the gusset tops; its feet drop just inside the rim
  and hover over the tools, and its nut bars sweep -6..0 clear above
  the deck
- LOCK: gabled slots in the rear wall catch the hook noses on the
  jig's rear feet (set down rear-first at a shallow tilt, lower the
  front); a corner gusset behind the front wall hides a swing cavity
  each for a knob's T-bar. The cavity ceiling is a fan of 45 deg
  sector tents DESCENDING CAM_RISE over the quarter turn - the
  ceiling itself is the cam. Drop the knob through base keyhole and
  entry slot, quarter turn clockwise to the -90 stop, and its wing
  roof seats face-on-face under ever lower tents, drawing the base
  onto the walls; the last RISE - CLEAR is PETG-spring preload and
  the shallow step-to-step descent cannot shake open
- layout: adapter slots split 3 per side wall (the six benched
  planks on edge, records in a crate, proud of the rim for the
  grab); the whole middle rear is the bay, split by height:
  stencils to 125 square lie flat in a sheet well at the deep
  floor, and a lift-out tray (tray.py) rides the well's side
  ledges at the tool floor, hauling boards + bench sundries flush
  with the rim. The bay grows a centered finger alcove toward the
  small squeegee pocket (which keeps its own wall), floor flush
  with the well's - pinch the tray's front wall there to lift it
  out, fish sheets down the same hole once it's gone. Squeegees
  front-left + center, loose hardware
  front-right (the knobs live there between stackings). The tongue
  is not stored: it lives in its groove

Walls are 6 perimeters of a 0.6 nozzle; 0% infill slices the deck as
skins over air - any infill welcome for stiffness. Prints as
modeled: every ceiling is a 45 deg tent, gable, or cone, the widest
flat down-face is the ~3.6 strip atop a sector tent."""
from build123d import *

from stenciljig.base import plan
from stenciljig.common import (_box, BASE_D, BASE_T, BASE_W,
                               CAM_ENTRY_FIT, CAM_LID, CAM_RISE,
                               CAM_TENT, CAM_XY, CAP_T, FIT_LOOSE,
                               FOOT_H, ORANGE, FOOT_INSET, FOOT_POSE,
                               PCB_D, SIDE_W)
from stenciljig.foot import mount_cutter, HOOK_H, HOOK_W
from stenciljig.knob import t_slot
from stenciljig.squeegee import (SQ_BASE, SQ_CHEEK_REACH, SQ_CHEEK_T,
                                 SQ_SIZES)

WALL = 3.6                 # 6 perimeters of a 0.6 nozzle
DIV = 2.4                  # pocket dividing walls
PKT = 0.8                  # pocket clearance per side, drop-in

EXT = BASE_W / 2           # shares the base plan, 103
BOX_H = 38.0
IX = EXT - WALL            # interior half-width
IY = (WALL, BASE_D - WALL) # interior front / rear faces
RIM = -13.0                # deck top: a stored plank tops out at
                           # -9.2, 3.8 proud for the grab
FLOOR_Z = -25.4            # common tool floor + the bay's tray ledges
SHEET_Z = -29.6            # sheet well's deep floor, 2.4 over the slab

AD_SLOTS = 6               # the six benched planks; the mounted pair
                           # rides the jig
AD_SLOT_W = CAP_T + 2 * PKT
AD_DIV = 1.8               # slot fins, 3 perimeters
AD_D = PCB_D + 2 * PKT     # slot length, plank on edge
AD_Z = RIM - SIDE_W + 3.8  # slot floor: a plank stands SIDE_W tall,
                           # 3.8 proud of the rim for the grab; the
                           # slab below keeps the 8.4 the stack needs
# slots split 3 per side wall so the whole middle is the bay
_AD_GRP = 3 * AD_SLOT_W + 2 * AD_DIV
AD_XS = tuple(-IX + 2.0 + i * (AD_SLOT_W + AD_DIV) for i in range(3)) \
    + tuple(IX - 2.0 - _AD_GRP + i * (AD_SLOT_W + AD_DIV)
            for i in range(3))
SHEET = 125.0              # max square sheet stored - the bay front
                           # is pinned by the sq60 pocket + divider,
                           # not by width (151.2 between slot groups)

# pocket anchors: wall-adjacent pockets inset 2 so their rounded
# corners stay clear of the cavity's corner rounds
BAY_Y1 = IY[1] - 2.0       # rear edge of bay + adapter slots
BAY_X0 = -IX + 2.0 + _AD_GRP + DIV
BAY_W = 2 * -BAY_X0        # 151.2, symmetric
BAY_D = SHEET + 2 * PKT
SQ_XY0 = (-75.6, WALL + 2.0)            # squeegee row start
HW_BIN = (20.4, WALL + 2.0, 75.6, 35.2) # loose hardware, x0 y0 x1 y1
NOTCH = (-7.6, 18.0)       # finger alcove, centered: the bay grows
                           # a front stem over the small squeegee
                           # pocket's span - the pocket keeps its
                           # own rear wall, the floor is well-deep

# cam gussets (front corners, tops seat the base)
GUS = (78.0, 29.6)         # gusset inner corner |x|, y
CAV_Z0 = -11.0             # swing cavity band floor
CAM_STEPS = range(0, -91, -6)           # sector fan, locked at -90

# rear-foot hook slots (rear wall)
TSLOT_D = 2.6              # into the wall, 1.0 of exterior skin kept
TSLOT_TOP = -FOOT_H + HOOK_H + FIT_LOOSE  # mouth edge over the nose

# the box stands on the SAME screw-on feet as the jig (4 plain), one
# per corner, long axis along the side walls so the r10 pad ends
# nest under the r10 corner rounds; the 8.4 stance leaves finger
# room to lift the box from below
BOX_FOOT_POSE = tuple(
    (s * (EXT - FOOT_INSET), y, -s * 90.0)
    for s in (1, -1) for y in (FOOT_INSET, BASE_D - FOOT_INSET))

PRINT = None               # open top up, as modeled


def _pocket(x0, y0, w, d, floor):
    # corner r must intrude less than the PKT clearance: square-
    # cornered tenants (stencil sheets, squeegee cheeks) sit flush
    return Pos(x0 + w / 2, y0 + d / 2, floor) * extrude(
        RectangleRounded(w, d, 2.0), RIM - floor + 1)


def build_box() -> Part:
    p = Pos(0, 0, -BOX_H) * extrude(plan(), BOX_H)

    # cavity = interior (plan offset by the wall) minus the gusset
    # footprints: walls, corner rounds and gussets all fall out of
    # one sketch boolean, clipped to the outline by construction
    cav = offset(plan(), -WALL)
    for s in (1, -1):
        cav -= _gusset_rect(s)
    p -= Pos(0, 0, RIM) * extrude(cav, -RIM + 0.1)
    e = p.edges().group_by(Axis.Z)
    p = chamfer(e[0] + e[-1], 0.5)          # seat lead-in + bed edge,
                                            # before cuts break them up

    # cam swing cavities: entry slot through the ceiling band, then a
    # fan of sector cuts whose 45 deg tents step down CAM_RISE toward
    # the -90 stop - the descending tents are the cam surface. Both
    # sides cut at the same absolute angles so ONE knob model locks
    # clockwise on either side (mirroring would flip its handedness)
    eslot = offset(t_slot(), CAM_ENTRY_FIT)
    desc = CAM_RISE / (len(CAM_STEPS) - 1)
    for s in (1, -1):
        at = Pos(s * CAM_XY[0], CAM_XY[1])
        p -= at * Pos(0, 0, -CAM_LID - 0.1) * extrude(eslot, CAM_LID + 0.2)
        for k, a in enumerate(CAM_STEPS):
            sk = Rot(0, 0, a) * eslot
            zb = -CAM_LID - desc * k
            p -= at * Pos(0, 0, CAV_Z0) * extrude(sk, zb - CAV_Z0)
            p -= at * Pos(0, 0, zb) * extrude(sk, CAM_TENT, taper=45)

    # rear-foot hook slots: mouth in the rear wall's inner face; the
    # roof gables up 45 deg from the mouth, so the catch is the
    # mouth's crisp printed edge, never a drooping bridge
    for x, y, a, hook in FOOT_POSE:
        if not hook:
            continue
        # cutter runs 1.6 proud of the wall face: the interior corner
        # round bulges past the face there and would clip the nose
        p -= _box(x - HOOK_W / 2 - 0.4, x + HOOK_W / 2 + 0.4,
                  IY[1] - 1.6, IY[1] + TSLOT_D,
                  -FOOT_H - 0.5, TSLOT_TOP)
        roof = Polygon((IY[1] - 0.1, TSLOT_TOP),
                       (IY[1] + TSLOT_D, TSLOT_TOP + TSLOT_D + 0.1),
                       (IY[1] + TSLOT_D, TSLOT_TOP), align=None)
        p -= Pos(x, 0, 0) * extrude(Plane.YZ * roof,
                                    HOOK_W / 2 + 0.4, both=True)

    # bay, split by height: a full-width shelf at the tool floor
    # over a BAY_D-square sheet well - the side steps are the
    # ledges the tray rides on
    p -= _pocket(BAY_X0, BAY_Y1 - BAY_D, BAY_W, BAY_D, FLOOR_Z)
    p -= _pocket(-BAY_D / 2, BAY_Y1 - BAY_D, BAY_D, BAY_D, SHEET_Z)
    # access: centered finger alcove between the small squeegee
    # pocket and the bay, floor flush with the sheet well (a DIV
    # wall keeps the pocket enclosed): pinch the tray's front wall
    # to lift it, reach sheets to the bottom of the stack
    p -= _box(NOTCH[0], NOTCH[1], 30.4,
              BAY_Y1 - BAY_D + 0.1, SHEET_Z, RIM + 1)

    # squeegee pockets, front left, clear of the gusset
    x0, y0 = SQ_XY0
    for w, k in SQ_SIZES:
        sx = w - 1 + 2 * SQ_CHEEK_T + 2 * PKT
        sy = SQ_BASE * k + 2 * SQ_CHEEK_REACH + 2 * PKT
        p -= _pocket(x0, y0, sx, sy, FLOOR_Z)
        x0 += sx + DIV

    # adapter slots, planks on edge, 3 against each side wall
    for x0 in AD_XS:
        p -= _pocket(x0, BAY_Y1 - AD_D, AD_SLOT_W, AD_D, AD_Z)
    # loose hardware between squeegees and the right gusset
    p -= _pocket(HW_BIN[0], HW_BIN[1], HW_BIN[2] - HW_BIN[0],
                 HW_BIN[3] - HW_BIN[1], FLOOR_Z)

    # foot mounts in the bottom face, same shared cutter as the base
    # (the slab is >=8.4 everywhere here, the stack needs 6)
    cut = mount_cutter()
    for x, y, a in BOX_FOOT_POSE:
        p -= Pos(x, y, -BOX_H) * Rot(0, 0, a) * cut

    p.label = "stenciljig_box"
    p.color = ORANGE
    return p


def _gusset_rect(s):
    """Gusset footprint, oversize outboard and forward - the cavity
    boolean clips it to the walls, so no corner can poke past the
    outer rounds (square blocks unioned on did exactly that)."""
    return Pos(s * (GUS[0] + EXT + 2) / 2, (GUS[1] - 1) / 2) * \
        Rectangle(EXT + 2 - GUS[0], GUS[1] + 1)

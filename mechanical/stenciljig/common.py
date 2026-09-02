"""Shared spec for the stencil jig set (design story in __init__.py):
printed fits, M3 hardware, the jig frame, and every cross-part
interface. A dimension lives here only when two parts must agree on
it; everything else stays in its part's file, and a mating cavity is
built by offset()-ing the owning part's profile by a fit from here
rather than re-drawing it.

Frame: x = 0 at the jig centerline, y = 0 at the front edge, z = 0 at
the base underside. Board slides in from the front (-y).

Printability doctrine (every part exports pre-oriented for slicing,
see __main__.py): no down-face steeper than 45 deg off the bed, no
bridge over ~3.6 wide; internal ceilings close as 45 deg cones/tents;
catch and bearing surfaces are printed edges or 45 deg faces, never
flat bridges. check_overhangs.py in the session scratchpad audits
every part's print pose against this.
"""
import math

from build123d import *

# --- printed fits (clearance per side, mating part also printed) ---
FIT_LOOSE = 0.15           # free slide
FIT_STD = 0.10             # drop-in
FIT_TIGHT = 0.05           # push-in, holds inverted
FIT_PRESS = 0.0            # interference from print shrinkage alone

# --- M3 hardware: button heads (ISO 7380) + plain nuts in printed
# --- pockets (encbench benched: 5.7 drop-out, 5.4 broach) ---
M3_HOLE = 3.4              # vertical clearance, prints ~3.25
M3_FREE = 3.6              # pass-throughs, drop-through loose
M3_NUT_T = 2.4
NUT_AF = 5.5               # across flats, prints ~true
NUT_PRESS_AF = NUT_AF + 2 * FIT_PRESS
                           # every nut SEAT: interference comes from
                           # print shrinkage, the far-side screw's
                           # preload drags the nut home
NUT_SLIDE_AF = NUT_AF + 2 * FIT_LOOSE
                           # deep-pocket travel shaft: the nut rides
                           # up free, then hits the press band
M3_TAP = 2.9               # self-tap pilot: M3 machine screw forms
                           # its own threads in PETG (knob spine +
                           # foot mounts)
BTN_H = 1.65               # ISO 7380 button head, 5.7 across
BTN_CBORE = BTN_H + FIT_TIGHT      # head pockets: dome sits flush
BTN_CBORE_D = 6.5          # drop-in room around the 5.7 head

# --- frame (bobstay parameter names kept recognizable) ---
BORDER = 5.0
SIDE_W = 20.0              # side plate width
LEDGE = 1.0                # board rests on this, each side
PCB_W = 120.0              # max board the plates open to
PCB_D = 120.0
PCB_T = 2.0                # thickest supported board (JLC catalog
                           # top); the base's rear notch is cut for
                           # it, thinner boards clear
AD_PAIRS = ((1.6, 1.0), (1.2, 0.8), (0.6, 0.4), (2.0, 1.6))
                           # adapter planks, one thickness per long
                           # edge (flip 180 to swap) - the full JLC
                           # rigid ladder, workhorse 1.6 spared twice
LEDGE_PROUD = 0.1          # ledge drop = pcb_t - this: the board rides
                           # proud so the stencil seats on the board,
                           # not the plates (m007 measures 1.579 ->
                           # 0.08 proud); slice at 0.10 layers so
                           # every drop lands on a boundary
STENCIL_W = 168.0          # max stencil width, drives BASE_W - sized
                           # so the box below closes its layout
UNDER = 15.0               # clear space under the board
BASE_T = 6.0
REAR_D = 40.0              # rear plate depth
CLAMP_D = 34.0             # clamp plate depth
CLAMP_T = 6.0
SCREW_SP = 4.0             # bobstay's insertHoleDia, kept as spacing unit
NOTCH_EXTRA = 0.02         # rear ledge cut slightly deep - board never catches
OUTLINE_R = 10.0           # base / box shared corner round

BASE_W = 2 * BORDER + SIDE_W + 2 * SCREW_SP + STENCIL_W  # 206
BASE_D = 2 * BORDER + PCB_D + REAR_D                     # 170
PLATE_H = UNDER + PCB_T                                  # 16.6
PCB_TOP = BASE_T + PLATE_H                               # 22.6 stencil plane
REAR_Y = (BORDER + PCB_D, BORDER + PCB_D + REAR_D)       # 125 .. 165
REAR_HW = BASE_W / 2 - BORDER                            # 98
PLATE_YC = BORDER + PCB_D / 2                            # 65 plate y center
CLAMP_Y0 = REAR_Y[1] - CLAMP_D

# --- ejection tongue channel (base <-> tongue) ---
TONGUE_W = 20.0
TONGUE_CLEAR = FIT_LOOSE   # groove growth each side
TONGUE_DROP = 0.1          # tongue 0.1 under board thickness - slides free
TONGUE_LEN = PCB_D * 0.8

# --- rail slots (base <-> side plate travel; track the ear holes) ---
# rows centered on the plate depth (bobstay used board thirds) so a
# plate flipped 180 deg lands on the same slots
SLOT_Y = (PLATE_YC - PCB_D / 6, PLATE_YC + PCB_D / 6)
SLOT_W = 4.0               # loose on the M3 shank
EAR_HOLE = 4.0             # ear hole center, out from the plate face
SLOT_X0 = (PCB_W / 2 - LEDGE) + SIDE_W + EAR_HOLE  # screw x at max opening
SLOT_LEN = PCB_W / 2 - LEDGE  # travel band: plate inner faces touch
                              # at the centerline when fully closed

# --- keel rail (base <-> side plate; profile shared with the adapter
# --- deck's dovetail) ---
RAIL_Y = (SLOT_Y[0] + SLOT_Y[1]) / 2
RAIL_W = 8.0               # dovetail top (wide) width
RAIL_ROOT = 5.0            # root width: 1.5 overhang each side keeps
                           # the mating part from lifting off
RAIL_H = 3.0               # under-board clearance drops 15 -> 12 here
RAIL_X = PCB_W / 2 - LEDGE + SIDE_W     # half-length, plate outer face
RAIL_SLIDE = 2 * FIT_LOOSE # groove width over rail - tight slide
RAIL_DROP = 0.3            # groove deeper than rail: mating part seats
                           # on the face, the rail only guides
GROOVE_ROOT = RAIL_ROOT + RAIL_SLIDE
GROOVE_TOP = RAIL_W + RAIL_SLIDE + RAIL_DROP * (RAIL_W - RAIL_ROOT) / RAIL_H
GROOVE_H = RAIL_H + RAIL_DROP  # groove keeps the rail's flank slope

# --- adapter interface (side plate <-> adapter) ---
CAP_T = 3.0                # adapter plank thickness over the plate
                           # body; fully backed by the plate top, so
                           # only the head cbore web and the 1.9 max
                           # ledge drop size it
AD_SCREW_Y = (PLATE_YC - PCB_D / 3, PLATE_YC + PCB_D / 3)
                           # plank screw rows, symmetric about the
                           # plate center so a 180 flip lands on the
                           # same holes; clear of the keel gable attic
                           # at RAIL_Y (M3x10 reach audit in adapter.py)

# --- clamp screw grid (base <-> clamp plate; bobstay's 9 positions
# --- less the rear corner pair - two is enough, 2+ used) ---
COL_X = STENCIL_W / 2 + SCREW_SP
COL_Y = (CLAMP_Y0 + CLAMP_D / 4, CLAMP_Y0 + CLAMP_D / 2)
MID_X = 2 * REAR_HW / 6
MID_Y = CLAMP_Y0 + CLAMP_D - 1.5 * SCREW_SP
CLAMP_HOLES = tuple((s * COL_X, y) for s in (1, -1) for y in COL_Y) \
    + tuple((x, MID_Y) for x in (-MID_X, 0.0, MID_X))

# --- stacking interface (base <-> foot <-> box <-> knob: the jig
# --- stands beside the open box on screw-on feet and stacks onto it
# --- as the lid; rear-foot hooks + front cam knobs lock the seam) ---
FOOT_INSET = 10.0          # box floor pad centers, in from each corner
FOOT_H = 6.4               # puck height, stance 8.4 with the pad:
                           # clears the nut bars (6) on the bench;
                           # taller would put the stacked pads into
                           # the boxed squeegee's crown
FOOT_SCREW_OFF = 12.0      # screw station, inboard of the pad center
FOOT_PIN_OFF = 18.5        # anti-rotation pin, further inboard
FOOT_PIN_D = 4.0
FOOT_PIN_H = 2.2
# pad-center anchor, long-axis angle (screw side = local -y), hook.
# Rear feet point their noses at the box wall, anchored inboard so
# their screw + pin cuts clear the clamp pockets' full-depth broach
# shafts at (+/-COL_X, COL_Y); front feet turn sideways so screw +
# pin stay clear of the rail slot rows
FOOT_POSE = ((60.0, 14.0, -90.0, False), (-60.0, 14.0, 90.0, False),
             (78.0, 156.0, 0.0, True), (-78.0, 156.0, 0.0, True))


def foot_pt(x, y, a, d):
    """Point d along a foot's screw side (local -y) in jig frame."""
    r = math.radians(a)
    return x + d * math.sin(r), y - d * math.cos(r)


CAM_XY = (88.5, 16.6)      # cam axes (+/-x): head circle 0.5 clear of
                           # a side plate at max opening, ears 1.4
                           # away, T swing inside the box wall
CAM_LID = 3.0              # box gusset ceiling band the wings ride under
CAM_CLEAR = 0.65           # roof-edge drop below that ceiling at entry
CAM_RISE = 0.8             # total descent of the box's sector tents
                           # over the quarter turn; bites RISE - CLEAR
CAM_TENT = 2.2             # shared 45 deg tent height: the knob's wing
                           # roof and the box's ceiling tents use the
                           # same taper so they seat face on face
CAM_KEY_FIT = 2 * FIT_LOOSE        # base keyhole over the T profile
CAM_ENTRY_FIT = 0.5        # box entry slot over the T profile

# print palette (ELEGOO PETG): BLACK = the whole jig (base, plates,
# clamp, tongue, bars), ORANGE = box, GREY = feet + pads, GREEN =
# hand tools (knob, squeegees) - the touch points. Planks are silk
# PLA, one color per design (adapter.AD_COLORS)
GREEN = Color(0.10, 0.48, 0.15)
BLACK = Color(0.04, 0.04, 0.045)
ORANGE = Color(0.85, 0.33, 0.05)
WHITE = Color(0.88, 0.88, 0.86)
GREY = Color(0.42, 0.43, 0.45)


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def _hex(af):
    return RegularPolygon(af / math.sqrt(3), 6)


def _nut_broach(seat):
    """Bottom-entry captive-nut cutter, mouth at z=0 opening down, nut
    top face at z=seat: loose hex shaft rides the nut up, a 45 lead
    into the press band, and the far-side screw's preload drags it the
    last 0.5 onto the 45 tapered-hex hopper that carries the tension.
    Caller adds the screw hole above the seat."""
    band0 = seat - M3_NUT_T - 0.5
    c = Pos(0, 0, -0.1) * extrude(_hex(NUT_SLIDE_AF), band0 + 0.1)
    c += Pos(0, 0, band0) * extrude(_hex(NUT_SLIDE_AF),
                                    FIT_LOOSE - FIT_PRESS, taper=45)
    c += Pos(0, 0, band0) * extrude(_hex(NUT_PRESS_AF), seat - band0)
    c += Pos(0, 0, seat - 0.01) * extrude(_hex(NUT_PRESS_AF), 1.0,
                                          taper=45)
    return c


def _plate(w, d, h, r, bevel_):
    """Rounded-corner plate with top/bottom edge bevels, centered in
    xy, z 0..h (bobstay's RoundedBevelCube)."""
    p = extrude(RectangleRounded(w, d, r), h)
    edges = p.edges().group_by(Axis.Z)
    return chamfer(edges[0] + edges[-1], bevel_)


def _dovetail_rail(root, top, h, half_len, lead=0.0, gable=False):
    """Dovetail bar along x, root (narrow) plane at z=0 flaring to top
    width at z=h - the overhang blocks lift-off, so mating parts only
    slide in from the ends. lead chamfers the end faces' upper edges
    for entry. gable replaces the flat top with a 45 deg roof ridge:
    use it for GROOVE cuts whose ceiling would otherwise print as a
    bridge and droop into the rail path (a printed keel groove jammed
    on the base rail exactly that way); the mating rail never reaches
    the ceiling, so the attic costs nothing."""
    pts = [(-root / 2, 0), (root / 2, 0), (top / 2, h)]
    if gable:
        pts.append((0, h + top / 2))
    pts.append((-top / 2, h))
    prof = Polygon(*pts, align=None)
    bar = extrude(Plane.YZ * prof, half_len, both=True)
    if lead:
        ends = [e for i in (0, -1) for e in bar.edges().group_by(Axis.X)[i]
                if e.center().Z > 0.01]
        bar = chamfer(ends, lead)
    return bar

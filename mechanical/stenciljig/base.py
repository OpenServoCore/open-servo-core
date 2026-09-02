"""Jig base: rail bed, rear stencil deck, keel rail, and the sockets
for every bolt-on - foot nuts pocket into the underside, cam
keyholes pass the knob's T, nut bars slide underneath. The top face
keeps only its original through-cuts; every underside pocket closes
as a 45 deg hopper or cone, so the base prints as modeled with zero
bridges.

Screws through this part: side plates clamp with M3x16 through the
ear tabs into nut bars (reach audit: 4 ear + 6 base + 2 bar web +
2.4 nut = 14.4, tip lands flush with the bar underside); clamp plate
M3x16 into bottom-entry deck pockets (nuts broach up from the
underside, tips ride into the loose shafts); feet take M3x10 from
below (foot.py) thread-forming the underside pilots. Everything else
runs from above with one allen key."""
from build123d import *

from stenciljig.common import (_box, _nut_broach, _plate,
                               _dovetail_rail, BASE_D, BASE_T, BASE_W,
                               BORDER, CAM_KEY_FIT, CAM_XY,
                               CLAMP_HOLES, FOOT_POSE, LEDGE,
                               LEDGE_PROUD, M3_HOLE, NOTCH_EXTRA,
                               OUTLINE_R, PCB_T, PCB_TOP, RAIL_H,
                               RAIL_ROOT, RAIL_W, RAIL_X, RAIL_Y,
                               REAR_HW, REAR_Y, SLOT_LEN, SLOT_W,
                               SLOT_X0, SLOT_Y, TONGUE_CLEAR,
                               TONGUE_W, BLACK)
from stenciljig.foot import mount_cutter
from stenciljig.knob import t_slot

PRINT = None               # underside down, as modeled


def plan():
    """Base outline; the box shares it as its own plan."""
    return Pos(0, BASE_D / 2) * RectangleRounded(BASE_W, BASE_D,
                                                 OUTLINE_R)


def build_base() -> Part:
    p = extrude(plan(), BASE_T)
    e = p.edges().group_by(Axis.Z)
    p = chamfer(e[0] + e[-1], 1.0)

    # rear plate: stencil clamping deck, top = the stencil plane
    p += _box(-REAR_HW, REAR_HW, REAR_Y[0], REAR_Y[1], BASE_T, PCB_TOP)

    # keel rail: dovetail riding the groove under each side plate - a
    # loose plate stays square AND held down; plates slide on from the
    # rail ends (root sunk 0.1 for the boolean)
    p += Pos(0, RAIL_Y, BASE_T - 0.1) * _dovetail_rail(
        RAIL_ROOT, RAIL_W, RAIL_H + 0.1, RAIL_X, lead=0.8)

    # board rear ledge: 1 mm notch the board backs into - the y datum.
    # Floor is cut for the thickest board and supports its rear edge;
    # thinner boards float above it, corners on the side ledges, floor
    # as a backstop if the squeegee flexes the rear edge down
    p -= _box(-REAR_HW - 0.1, REAR_HW + 0.1, REAR_Y[0] - LEDGE,
              REAR_Y[0] + LEDGE, PCB_TOP - (PCB_T - LEDGE_PROUD) - NOTCH_EXTRA,
              PCB_TOP + 0.1)

    # ejection tongue groove through the rear plate deck
    gw = TONGUE_W / 2 + TONGUE_CLEAR
    p -= _box(-gw, gw, REAR_Y[0] - 0.1, REAR_Y[1] + 0.1,
              PCB_TOP - PCB_T - TONGUE_CLEAR, PCB_TOP + 0.1)

    # rail slots, one continuous slot per row per side
    for s in (1, -1):
        for y in SLOT_Y:
            p -= Pos(s * (SLOT_X0 - SLOT_LEN / 2), y, -0.1) * extrude(
                SlotOverall(SLOT_LEN + SLOT_W, SLOT_W), BASE_T + 0.2)

    # clamp nut pockets, bottom-entry: loose hex shaft up from the
    # underside, press band + 45 hopper carrying the clamp tension,
    # M3_HOLE web up to the deck; the screw from the top drags each
    # nut to seat, x16 tips ride into the loose shaft. The centre
    # pocket hangs from the tongue groove floor instead, so its nut
    # sits fully recessed and the tongue slides over a bare M3 hole
    for x, y in CLAMP_HOLES:
        top = PCB_TOP - PCB_T - TONGUE_CLEAR if x == 0.0 else PCB_TOP
        seat = top - 2.6           # 1.0 hopper + 1.6 web over the nut
        p -= Pos(x, y, 0) * _nut_broach(seat)
        p -= Pos(x, y, seat) * Cylinder(
            M3_HOLE / 2, top - seat + 0.1,
            align=(Align.CENTER, Align.CENTER, Align.MIN))

    # foot mounts, all in the underside (the top face gains nothing;
    # the rear pair sits under the deck where top access would be
    # impossible anyway): the shared cutter's tap pilot + pin socket
    # fit the bare 6 slab
    cut = mount_cutter()
    for x, y, a, hook in FOOT_POSE:
        p -= Pos(x, y, 0) * Rot(0, 0, a) * cut

    # cam keyholes: the knob's T drops through into the box's gusset
    # cavity; a quarter turn seats its wing roof under the box's
    # descending ceiling tents and draws the seam tight (knob.py,
    # box.py). Unstacked, the knobs live in the box's hardware bin
    for s in (1, -1):
        p -= Pos(s * CAM_XY[0], CAM_XY[1], -0.1) * extrude(
            offset(t_slot(), CAM_KEY_FIT), BASE_T + 0.2)

    p.label = "stenciljig_base"
    p.color = BLACK
    return p

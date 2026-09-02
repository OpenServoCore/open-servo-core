"""Thickness-agnostic side plate, one model for both hands: flipped
180 deg about vertical the ear lands outboard and the holes land on
the recentered slot rows (front<->rear swap is the whole trick -
print two). The board ledge lives on the adapter plank (adapter.py)
screwed flat onto the top: two M3x10 drop through the plank into
nuts ridden up bottom-entry hex pockets, and the screw preload
drags each nut into its press band and tapered-hex hopper.
The plank roofs the two top holes; the base roofs the pocket
mouths. Modeled in the right-hand station at max opening.

Prints upright as modeled: the keel groove's gable, the nut
hoppers' 45 tapers and the horizontal M3 holes all carry their own
angles."""
from build123d import *

from stenciljig.common import (_box, _nut_broach, _plate,
                               _dovetail_rail, AD_SCREW_Y, BASE_T,
                               BORDER, CAP_T, EAR_HOLE, BLACK,
                               GROOVE_H, GROOVE_ROOT, GROOVE_TOP,
                               LEDGE, M3_FREE, PCB_D, PCB_W, PLATE_H,
                               RAIL_Y, SIDE_W, SLOT_Y)

EAR_L = 9.0                # sticks out from the plate's outer face;
                           # 3.2 of rim outboard of the hole = 5
                           # perimeters of a 0.6 nozzle
EAR_T = 4.0                # head bears straight down onto the base
EAR_W = 16.0               # per-row tab span, hole centered

PRINT = None               # upright, as modeled


def build_side_plate() -> Part:
    x0 = PCB_W / 2 - LEDGE                  # inner face at max opening
    xc = x0 + SIDE_W / 2
    # body corners rounded to the plank's radius so the stack shares
    # one outline
    p = _box(x0, x0 + SIDE_W, BORDER, BORDER + PCB_D,
             BASE_T, BASE_T + PLATE_H - CAP_T)
    p = fillet(p.edges().filter_by(Axis.Z), 3.0)
    e = p.edges().group_by(Axis.Z)
    p = chamfer(e[0] + e[-1], 0.5)

    face_x = x0 + SIDE_W
    # ears: one rounded low tab per rail row, screw hole centered;
    # the heads bear straight down through them onto the base, ~10
    # below the stencil plane in open air. Buried 2 into the plate
    # face so the rounds only show outboard
    for y in SLOT_Y:
        p += Pos(face_x + (EAR_L - 2) / 2, y, BASE_T) * _plate(
            EAR_L + 2, EAR_W, EAR_T, 3.0, 0.5)
        p -= Pos(face_x + EAR_HOLE, y, BASE_T - 0.1) * Cylinder(
            M3_FREE / 2, EAR_T + 0.2,
            align=(Align.CENTER, Align.CENTER, Align.MIN))

    # keel groove across the plate body underside (the ear tabs sit
    # clear of the rail band); dovetail over the base rail, deeper by
    # RAIL_DROP so the plate seats on the base face. Gabled roof:
    # printed upright the channel needs no bridge
    p -= Pos((x0 + face_x) / 2, RAIL_Y, BASE_T - 0.1) * \
        _dovetail_rail(GROOVE_ROOT, GROOVE_TOP, GROOVE_H + 0.1,
                       SIDE_W / 2 + 1, gable=True)

    # plank screw rows: bottom-entry captive nuts (common._nut_broach:
    # loose ride up, press band, 45 hopper - the screw's preload from
    # above seats the nut); the M3x10 tip rides 0.85 proud into the
    # loose shaft, and the base roofs the mouth once mounted
    top = BASE_T + PLATE_H - CAP_T
    seat = 14.3                # nut top face; M3x10 tip lands at 11.05
    for y in AD_SCREW_Y:
        p -= Pos(xc, y, BASE_T) * _nut_broach(seat - BASE_T)
        p -= Pos(xc, y, seat) * Cylinder(
            M3_FREE / 2, top - seat + 0.1,
            align=(Align.CENTER, Align.CENTER, Align.MIN))

    p.label = "stenciljig_side"
    p.color = BLACK
    return p

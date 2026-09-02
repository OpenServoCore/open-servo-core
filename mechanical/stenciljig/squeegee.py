"""Standing-wedge paste squeegee: both faces at SQ_ANGLE, so it sits
flat on the stencil - attack angle from geometry, not the hand - and
fore-aft symmetric, either face leads. Only a RAIL strip behind each
edge touches (base hollowed between). Cheek dams at the ends ride
the sheet, pen the bead in, and seal the hollow. Low blade profile:
wedge, short fin, T-grip bar - the whole tool is SQ_H tall so it
lies in the box like an adapter; fingers hook the grip flanks for
push or pull. w is the edge width (board + margin), k scales the
profile for small stencils, one size per stencil via SQ_SIZES.

Prints as modeled, base down: the working edges are bed corners
(zero elephant-foot compensation); the hollow is a single 45 deg
gable, rail to ridge, so no layer bridges it; the grip flare is
27 deg."""
import math

from build123d import *

from stenciljig.common import GREEN, PCB_TOP, PLATE_YC

SQ_ANGLE = 60.0            # face-to-stencil attack, both faces
SQ_BASE = 18.0             # stance depth - short enough that the
                           # wedge closes below the grip bar
SQ_H = 16.0                # overall height = an adapter lying flat
SQ_RAIL = 1.5              # contact strip behind each working edge
SQ_RELIEF = 0.8            # base hollowed between the strips
SQ_FIN_T = 6.0             # fin between wedge and grip
SQ_GRIP_H = 4.0            # T-grip bar atop the fin
SQ_GRIP_W = 10.0           # flares 2 per side over the 4 rise
SQ_CHEEK_T = 2.5           # end dams: bead stays between them
SQ_CHEEK_REACH = 5.0       # dam ahead of each edge; a max-depth board
                           # sweep still clears the clamp plate by 1
SQ_CHEEK_H = 12.0
SQ_SIZES = ((60.0, 1.0), (20.0, 0.6))  # (edge width, profile scale)

PRINT = None


def build_squeegee(w=60.0, k=1.0) -> Part:
    base, ft = SQ_BASE * k, SQ_FIN_T * k
    h1 = (base - ft) / 2 * math.tan(math.radians(SQ_ANGLE))
    ht, gh, gw = SQ_H * k, SQ_GRIP_H * k, SQ_GRIP_W * k
    prof = Polygon((-base / 2, 0), (base / 2, 0), (ft / 2, h1),
                   (ft / 2, ht - gh), (gw / 2, ht), (-gw / 2, ht),
                   (-ft / 2, ht - gh), (-ft / 2, h1), align=None)
    p = extrude(Plane.YZ * prof, w / 2, both=True)
    p = chamfer(p.faces().sort_by(Axis.Z)[-1].edges(), ft / 4)
    ch = SQ_CHEEK_H * k
    prof = Pos(0, ch / 2) * RectangleRounded(
        base + 2 * SQ_CHEEK_REACH, ch, ch / 4)
    for s in (1, -1):
        p += Pos(s * (w / 2 - 0.5 + SQ_CHEEK_T / 2), 0, 0) * extrude(
            Plane.YZ * prof, SQ_CHEEK_T / 2, both=True)
    # hollow, cut 0.1 into the cheeks so they close its ends; gabled
    # roof, no flat span (ridge clears the 60 deg faces by >= 1.8)
    hw = base / 2 - SQ_RAIL
    p -= extrude(Plane.YZ * Polygon(
        (-hw, -0.1), (hw, -0.1), (hw, SQ_RELIEF),
        (0, SQ_RELIEF + hw), (-hw, SQ_RELIEF), align=None),
        w / 2 - 0.4, both=True)
    p = Pos(0, PLATE_YC, PCB_TOP) * p
    p.label = f"stenciljig_squeegee{round(w):02d}"
    p.color = GREEN
    return p

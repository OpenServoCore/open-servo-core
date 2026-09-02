"""T-nut bar: slides flat against the base underside carrying the
two side-plate nuts, one per rail row. Hex pockets open DOWN in the
working pose - press the nuts up in from below, the screws retain
them after that. Clamp tension pulls each nut up into the 2 mm web,
the web presses the base, the plate presses down from above: base
sandwiched, nothing to spin or rip out.

Exports flipped: printed web-down the pockets open upward, so the
part has no down-faces at all. Print 2, same part both sides;
modeled under the right plate at max opening."""
from build123d import *

from stenciljig.common import (_hex, _plate, M3_FREE, NUT_PRESS_AF,
                               SLOT_X0, SLOT_Y, BLACK)

BAR_W = 12.0               # cross-section under the base
BAR_L = 60.0               # spans both rail rows (nuts at SLOT_Y)
                           # with a solid block past each pocket -
                           # 48 left 2 perimeters at the ends
BAR_T = 6.0                # 2 web + 2.4 nut + tip room; sweeps 7
                           # over the stacked box's deck
BAR_WEB = 2.0              # roof between nut and base underside

PRINT = Rotation(180, 0, 0)  # web down, pockets opening up


def build_nutbar() -> Part:
    xc = SLOT_X0                            # under the ear holes at
    yc = (SLOT_Y[0] + SLOT_Y[1]) / 2        # max opening
    p = Pos(xc, yc, -BAR_T) * _plate(BAR_W, BAR_L, BAR_T, 3.0, 0.5)
    for y in SLOT_Y:
        p -= Pos(xc, y, -BAR_T - 0.1) * extrude(
            _hex(NUT_PRESS_AF), BAR_T - BAR_WEB + 0.1)
        p -= Pos(xc, y, -BAR_T - 0.1) * Cylinder(
            M3_FREE / 2, BAR_T + 0.2,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = "stenciljig_nutbar"
    p.color = BLACK
    return p

"""Stencil clamp plate over the rear deck: M3x16 from above into the
nuts captive in the deck's bottom-entry pockets, heads flush in 1.7
pockets. Two screws clamp fine; the rest of the grid is bobstay's
"overkill clamping". Prints as modeled."""
from build123d import *

from stenciljig.common import (_plate, BTN_CBORE, BTN_CBORE_D, CLAMP_D,
                               CLAMP_HOLES, CLAMP_T, CLAMP_Y0, M3_FREE,
                               BLACK, PCB_TOP, REAR_HW)

PRINT = None


def build_clamp_plate() -> Part:
    p = Pos(0, CLAMP_Y0 + CLAMP_D / 2, PCB_TOP) * _plate(
        2 * REAR_HW, CLAMP_D, CLAMP_T, CLAMP_D / 8, 0.5)
    for x, y in CLAMP_HOLES:
        p -= Pos(x, y, PCB_TOP - 0.1) * Cylinder(
            M3_FREE / 2, CLAMP_T + 0.2,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= Pos(x, y, PCB_TOP + CLAMP_T - BTN_CBORE) * Cylinder(
            BTN_CBORE_D / 2, BTN_CBORE + 0.1,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = "stenciljig_clamp"
    p.color = BLACK
    return p

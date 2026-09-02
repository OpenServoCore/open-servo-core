"""Ejection tongue: slides in the rear-plate groove, under the
stencil, flush-ish with the board top; the handle hangs off the
back. Push to walk the pasted board forward instead of fingering it
out (don't smear). Works for all board thicknesses: the front face
spans the groove floor up to 0.3 under the stencil plane, so it
still catches a 0.6 board's edge. Lives in its groove, not the box.
Prints as modeled."""
from build123d import *

from stenciljig.common import (_plate, BLACK, PCB_T, PCB_TOP, REAR_Y,
                               SLOT_W, TONGUE_DROP, TONGUE_LEN,
                               TONGUE_W)

PRINT = None


def build_tongue() -> Part:
    t = PCB_T - TONGUE_DROP
    p = Pos(0, REAR_Y[0] + TONGUE_LEN / 2, PCB_TOP - t) * _plate(
        TONGUE_W, TONGUE_LEN, t, TONGUE_W / 4, 0.2)
    p -= Pos(0, REAR_Y[0] + TONGUE_LEN / 2, PCB_TOP - t - 0.1) * extrude(
        Rot(0, 0, 90) * SlotOverall(TONGUE_LEN - TONGUE_W + SLOT_W, SLOT_W),
        t + 0.2)
    p.label = "stenciljig_tongue"
    p.color = BLACK
    return p

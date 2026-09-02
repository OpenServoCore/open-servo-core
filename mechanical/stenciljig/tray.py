"""Lift-out tray splitting the box bay by height: stencil sheets
lie flat in the well below (a dozen barely fill it), the tray rides
the bay's side ledges at the tool floor and carries boards + bench
sundries (paste syringe, wipes) up at hand height, flush with the
deck rim so the stacked jig sweeps clear. Lift it by pinching the
front wall through the bay's finger alcove.

Prints as modeled, floor on the bed."""
from build123d import *

from stenciljig.box import BAY_D, BAY_W, FLOOR_Z, PKT, RIM
from stenciljig.common import ORANGE

TRAY_WALL = 2.4            # 4 perimeters of a 0.6 nozzle
TRAY_FLOOR = 2.4
TRAY_H = RIM - FLOOR_Z     # flush with the deck rim
TRAY_W = BAY_W - 2 * PKT
TRAY_D = BAY_D - 2 * PKT

PRINT = None               # floor on the bed, as modeled


def build_tray() -> Part:
    p = extrude(RectangleRounded(TRAY_W, TRAY_D, 2.0), TRAY_H)
    p -= Pos(0, 0, TRAY_FLOOR) * extrude(
        RectangleRounded(TRAY_W - 2 * TRAY_WALL,
                         TRAY_D - 2 * TRAY_WALL, 2.0), TRAY_H)
    e = p.edges().group_by(Axis.Z)
    p = chamfer(e[0] + e[-1], 0.5)
    p.label = "stenciljig_tray"
    p.color = ORANGE
    return p

"""Board-thickness adapter: a flat plank screwed onto the plate top,
one thickness per long edge - flip it 180 to swap edges, so four
planks cover the full JLC rigid ladder. The board rides LEDGE_PROUD
high on the inboard edge's ledge (bearing on the strip, the step face
takes the clamp); the outboard edge's ledge is the other thickness,
parked. Two M3x10 on the centerline drop into nuts captive in the
side plate; the screw rows are symmetric about the plate center so
either orientation lands on the same holes, and the snug shank holes
locate the plank. Each thickness label runs along its own edge, digit
tops toward that edge, and repeats at both ends (4 per plank), so
whichever edge is inboard - and whichever end faces the viewer - its
label reads.

Reach audit: deck top 23.0, head seat 1.95 down, M3x10 tip at 11.05 -
through the plate-top wall and the whole nut (14.2..16.6) with 0.65
of relief floor to spare (side_plate.py owns the channel).

Prints flat, ledges up (the old standing tower sheared its layers -
tall and skinny wobbles): the deck top and ledge strips are top skin,
so the drop between them is an exact layer count - slice at 0.10 so
every drop lands on a boundary. 0.25 nozzle for the label engraving.
Print 2 per pair."""
from build123d import *

from stenciljig.common import (_box, AD_SCREW_Y, BORDER, BTN_CBORE,
                               BTN_CBORE_D, CAP_T, LEDGE, LEDGE_PROUD,
                               M3_HOLE, PCB_D, PCB_TOP, PCB_W,
                               PLATE_YC, SIDE_W, WHITE)

AD_CBORE = BTN_CBORE + 0.25  # head 0.25 sub-flush: only the 0.1-proud
                             # board edge meets the stencil
ENGRAVE = 0.4              # thickness label depth
ENGRAVE_SIZE = 6.0

# silk PLA per design (MIKA3D bundle), color weight tracks thickness;
# both copies of a pair print the same color
AD_COLORS = {
    (0.6, 0.4): WHITE,
    (1.2, 0.8): Color(0.85, 0.72, 0.10),   # yellow
    (1.6, 1.0): Color(0.08, 0.52, 0.62),   # peacock blue
    (2.0, 1.6): Color(0.72, 0.08, 0.08),   # red
}

PRINT = None               # flat as modeled, ledges up


def build_adapter(ta=1.6, tb=1.0) -> Part:
    """ta ledges the inboard (modeled) edge, tb the outboard."""
    x0 = PCB_W / 2 - LEDGE
    xc = x0 + SIDE_W / 2
    p = Pos(xc, PLATE_YC, PCB_TOP - CAP_T) * extrude(
        RectangleRounded(SIDE_W, PCB_D, 3.0), CAP_T)
    p = chamfer(p.faces().sort_by(Axis.Z)[-1].edges(), 0.5)
    p = chamfer(p.faces().sort_by(Axis.Z)[0].edges(), 0.3)

    # ledge: drop LEDGE mm of the top along each edge so that edge's
    # board rides LEDGE_PROUD high
    for xa, xb, t in ((x0 - 0.1, x0 + LEDGE, ta),
                      (x0 + SIDE_W - LEDGE, x0 + SIDE_W + 0.1, tb)):
        p -= _box(xa, xb, BORDER - 0.1, BORDER + PCB_D + 0.1,
                  PCB_TOP - (t - LEDGE_PROUD), PCB_TOP + 0.1)

    for y in AD_SCREW_Y:
        p -= Pos(xc, y, PCB_TOP - CAP_T - 0.1) * Cylinder(
            M3_HOLE / 2, CAP_T + 0.2,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= Pos(xc, y, PCB_TOP - AD_CBORE) * Cylinder(
            BTN_CBORE_D / 2, AD_CBORE + 0.1,
            align=(Align.CENTER, Align.CENTER, Align.MIN))

    # labels run along their edges, digit tops toward the edge, at
    # both ends: the mounted pair is mirrored, so single-ended labels
    # leave one hand blank-first; edge-aligned text is the only layout
    # where two labels fit the 18-wide deck side by side
    for t, s in ((ta, 1), (tb, -1)):
        for e in (1, -1):
            plane = Plane((xc - s * 4.5,
                           PLATE_YC - e * (PCB_D / 2 - 9.0), PCB_TOP),
                          x_dir=(0, s, 0), z_dir=(0, 0, 1))
            p -= extrude(plane * Text(f"{t:.1f}", ENGRAVE_SIZE),
                         -ENGRAVE)

    p.label = f"stenciljig_adapter{round(ta * 10):02d}_{round(tb * 10):02d}"
    p.color = AD_COLORS.get((ta, tb), WHITE)
    return p

"""TPU desk pad + the recess profiles every host part reuses. The pad
snaps stem-first through an hourglass throat - captive by geometry,
peels out with a fingernail. One model serves the jig's feet and the
box floor: print 8 in TPU, exported FLIPPED - the retention flare's
full 14 disc lands on the bed (best TPU adhesion, and its 45 cone
turns into a shrinking dome instead of an overhang); the desk face
prints as the top skin."""
from build123d import *

from stenciljig.common import GREY

PAD_D = 14.0               # pad rim / recess interior
PAD_LIP = 0.6              # throat squeeze per side; also the 45 deg
                           # flare heights either side of the throat
PAD_H = 4.0
PAD_PROUD = 2.0            # rubber, not PETG, meets the desk
PAD_RECESS = PAD_H - PAD_PROUD
PAD_STOP = 1.0             # 45 deg shoulder ring above the recess: in
                           # the foot the pad bore runs through (no
                           # ceiling to print), the ring is what the
                           # pad's flare seats against
PAD_BORE_R = PAD_D / 2 - PAD_STOP

PRINT = Rotation(180, 0, 0)  # flare down, desk face up


def recess_cutter():
    """Hourglass recess: entry funnel, PAD_LIP throat, retention
    flare, all 45 deg. z=0 at the opening face, material above. The
    ceiling at PAD_RECESS is OPEN - close it with stop_ring() (foot:
    through-bore above) or stop_cone() (box: blind, cones shut)."""
    r = Pos(0, 0, -0.1) * Cone(
        PAD_D / 2 + 0.1, PAD_D / 2 - PAD_LIP, PAD_LIP + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    r += Pos(0, 0, PAD_LIP) * Cone(
        PAD_D / 2 - PAD_LIP, PAD_D / 2, PAD_LIP,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    r += Pos(0, 0, 2 * PAD_LIP - 0.05) * Cylinder(
        PAD_D / 2, PAD_RECESS - 2 * PAD_LIP + 0.05,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    return r


def stop_ring():
    """45 deg ring closing the recess down to the through-bore (base
    radius 0.05 proud so no flat sliver survives the union)."""
    return Pos(0, 0, PAD_RECESS - 0.05) * Cone(
        PAD_D / 2 + 0.05, PAD_BORE_R, PAD_STOP + 0.05,
        align=(Align.CENTER, Align.CENTER, Align.MIN))


def stop_cone():
    """45 deg cone closing a blind recess to a point (box floor)."""
    return Pos(0, 0, PAD_RECESS - 0.05) * Cone(
        PAD_D / 2 + 0.05, 0, PAD_D / 2 + 0.05,
        align=(Align.CENTER, Align.CENTER, Align.MIN))


def build_pad() -> Part:
    p = Cylinder(PAD_D / 2 - PAD_LIP, PAD_H - PAD_LIP,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    p += Pos(0, 0, PAD_H - PAD_LIP) * Cone(
        PAD_D / 2 - PAD_LIP, PAD_D / 2, PAD_LIP,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = chamfer(p.edges().group_by(Axis.Z)[0], 0.3)
    p.label = "stenciljig_pad"
    p.color = GREY
    return p

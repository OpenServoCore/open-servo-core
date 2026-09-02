"""Screw-on bench foot, ONE design for jig and box: an oblong puck
with two stations. The pad station carries the TPU pad in a THROUGH
bore (hourglass throat at the bottom, 45 deg stop ring, open to the
mounting face - no ceiling to print); the screw station takes an
M3x10 driven from BELOW - head recessed in the foot's underside, a
45 deg cone ring closing up to the free hole - thread-forming into
the host's M3_TAP pilot (mount_cutter below; seat the foot, drive -
no nut anywhere). An anti-rotation pin beyond the screw keys
the long axis. hook=True adds the rear nose that latches into the
box's rear-wall slot - the jig's rear feet are bench stance,
stacking registration AND the rear lock in one part.

Screw audit: head seat at -4.7, M3x10 tip at +5.3 into the host's
pilot ending 5.6 - 5.3 of formed thread (knob-spine precedent).

Modeled z=0 at the base underside, pad center at the origin, screw
side local -y (common.FOOT_POSE turns each foot on the jig,
box.BOX_FOOT_POSE on the box). Prints as modeled, bottom down: the
only internal down-faces are the 45 deg cones and the hourglass
flare. Print 6 plain + 2 hooked (4 plain go under the box)."""
from build123d import *

from stenciljig.common import (_box, BTN_CBORE, BTN_CBORE_D,
                               FIT_STD, FOOT_H, FOOT_PIN_D, FOOT_PIN_H,
                               FOOT_PIN_OFF, FOOT_SCREW_OFF, M3_FREE,
                               M3_TAP, GREY)
from stenciljig.pad import (recess_cutter, stop_ring, PAD_BORE_R,
                            PAD_RECESS, PAD_STOP)

FOOT_W = 20.0              # pad recess + 3 of rim
FOOT_L = FOOT_SCREW_OFF + FOOT_W        # oblong plan, end circles at
                                        # the pad and screw stations
HOOK_W = 12.0              # rear nose: width, reach past the pad-end
HOOK_REACH = 2.2           # rim, band height - locks into the box's
HOOK_H = 3.0               # rear-wall slots

PRINT = None               # bottom down, as modeled


def mount_cutter():
    """Underside cuts a host takes to mount a foot, in the foot's
    frame (z=0 mounting face, pad center at origin, screw side local
    -y): a blind M3_TAP pilot the screw thread-forms (clamp stack =
    head -> foot -> host threads, so BOTH parts are clamped; a nut
    pocketed here reacted against the foot's own roof and left the
    host floating 0.2 - caught on the printed base), 45 entry cone,
    end 5.6 up (M3x10 tip 5.3), and a coned pin socket. Both the jig
    base and the box consume this."""
    c = Pos(0, -FOOT_SCREW_OFF, -0.1) * Cylinder(
        M3_TAP / 2, 5.7,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    c += Pos(0, -FOOT_SCREW_OFF, -0.1) * Cone(
        M3_FREE / 2, M3_TAP / 2, (M3_FREE - M3_TAP) / 2 + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    r = FOOT_PIN_D / 2 + FIT_STD
    c += Pos(0, -FOOT_PIN_OFF, -0.1) * Cylinder(
        r, FOOT_PIN_H + 0.3,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    c += Pos(0, -FOOT_PIN_OFF, FOOT_PIN_H + 0.2) * Cone(
        r, 0, r, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return c


def build_foot(hook=False) -> Part:
    p = Pos(0, -FOOT_SCREW_OFF / 2, -FOOT_H) * extrude(
        SlotOverall(FOOT_L, FOOT_W, rotation=90), FOOT_H)
    p = chamfer(p.edges().group_by(Axis.Z)[0], 1.0)
    if hook:
        n = _box(-HOOK_W / 2, HOOK_W / 2, FOOT_W / 2 - 2.5,
                 FOOT_W / 2 + HOOK_REACH, -FOOT_H, -FOOT_H + HOOK_H)
        top_rear = n.edges().group_by(Axis.Y)[-1].group_by(Axis.Z)[-1]
        p += chamfer(top_rear, 1.2)
    pin = Pos(0, -FOOT_PIN_OFF) * Cylinder(
        FOOT_PIN_D / 2, FOOT_PIN_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p += chamfer(pin.edges().group_by(Axis.Z)[-1], 0.5)

    # pad station: through bore, the stop ring seats the pad's flare
    p -= Pos(0, 0, -FOOT_H) * (recess_cutter() + stop_ring())
    p -= Pos(0, 0, -FOOT_H + PAD_RECESS + PAD_STOP - 0.05) * Cylinder(
        PAD_BORE_R, FOOT_H + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    # screw station: head well from below, cone ring up to the free
    # hole - every internal face 45 deg or vertical
    ring = (BTN_CBORE_D - M3_FREE) / 2
    p -= Pos(0, -FOOT_SCREW_OFF, -FOOT_H - 0.1) * Cylinder(
        BTN_CBORE_D / 2, BTN_CBORE + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(0, -FOOT_SCREW_OFF, -FOOT_H + BTN_CBORE - 0.05) * Cone(
        BTN_CBORE_D / 2 + 0.05, M3_FREE / 2, ring + 0.05,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(0, -FOOT_SCREW_OFF, -FOOT_H + BTN_CBORE + ring - 0.1) * \
        Cylinder(M3_FREE / 2, FOOT_H + 0.2,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))

    p.label = "stenciljig_hookfoot" if hook else "stenciljig_foot"
    p.color = GREY
    return p

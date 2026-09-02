"""Cam bolt for the stack lock, two printed parts on an M3x16 spine:
drop the T through the base keyhole and the box's entry slot, quarter
turn clockwise to the stop. The cam ramp lives in the BOX (descending
ceiling tents, box.py); the rod carries the follower: a 45 deg
tapered roof on the wing whose faces seat flat on those tents,
drawing the handle down onto the keyhole's side strips.

Split so each part prints in its strong pose (a one-piece knob either
lies down and loses the full-circle head, or stands head-down with
every weld across the tension path):
- rod prints standing as modeled: wing root bending and the cam
  faces stay in-plane, the roof prints as a clean 45 tent
- handle prints flat: a full scallop circle bearing on the keyhole
  strips across its layers, top unbroken but for the steel head and
  the T groove
- the spine self-taps TAP_ENGAGE into the wing root (M3_TAP pilot)
  and carries all cam tension in steel; plastic only sees
  compression, bearing and torque. The two-flat boss keys the
  torque, and being 2-fold like the T, the top groove always marks
  the T direction. No nut anywhere in the knob.

Modeled installed, z=0 at the handle underside = base top. One knob
serves both sides - the box cuts both cavities at the same absolute
angles. Between stackings it lives in the hardware bin."""
import math

from build123d import *

from stenciljig.common import (_box, BASE_T, BTN_CBORE, BTN_CBORE_D,
                               CAM_CLEAR, CAM_LID, CAM_TENT, FIT_LOOSE,
                               M3_FREE, M3_HOLE, M3_TAP, GREEN)

CAM_T = (16.0, 7.0)        # T-bar plan slot
CAM_T_ROOT = 6.0           # wing slab under the roof
CAM_SHAFT = 7.0
CAM_LEAD = -(BASE_T + CAM_LID + CAM_CLEAR)
                           # wing roof-edge plane when the handle
                           # sits on the base top

HANDLE_D = 18.0            # bears on the keyhole's side strips
HANDLE_T = 4.5
BOSS_W = 5.0               # torque key flats; the boss is the shaft
                           # circle with two flats so its arcs continue
                           # the shaft surface (a slot merely tangent
                           # at +/-x welds a degenerate seam - the
                           # mesher drops faces, open neck)
BOSS_H = 2.1
SOCKET_FIT = FIT_LOOSE     # socket offset; the socket is also 0.1
                           # deeper so the spine clamps handle to
                           # shaft shoulder, never boss top to web
SPINE_L = 16.0             # the house M3x16
SEAT_Z = HANDLE_T - BTN_CBORE      # head seat in the handle
TIP_Z = SEAT_Z - SPINE_L           # 1.95 shy of the wing underside
TAP_ENGAGE = 6.0           # threads formed across shaft + wing root

PRINT = None               # both parts model in their print pose


def t_slot():
    """T-bar plan profile: base keyhole and box entry slot are offsets
    of this."""
    return SlotOverall(*CAM_T)


def boss_profile():
    """Double-D torque key, 2-fold like the T; the handle socket is an
    offset of this."""
    return Circle(CAM_SHAFT / 2) & Rectangle(CAM_SHAFT + 1, BOSS_W)


def build_rod() -> Part:
    p = Cylinder(CAM_SHAFT / 2, -CAM_LEAD,
                 align=(Align.CENTER, Align.CENTER, Align.MAX))
    p += Pos(0, 0, CAM_LEAD - CAM_T_ROOT) * extrude(t_slot(), CAM_T_ROOT)
    # follower roof: same tent as the box ceiling, faces seat flat
    p += Pos(0, 0, CAM_LEAD) * extrude(t_slot(), CAM_TENT, taper=45)
    # keyhole entry lead on the wing's leading face
    p = chamfer(p.edges().group_by(Axis.Z)[0], 0.8)
    p += extrude(boss_profile(), BOSS_H)
    p = chamfer(p.edges().group_by(Axis.Z)[-1], 0.5)
    p -= Pos(0, 0, BOSS_H) * Cylinder(
        M3_HOLE / 2, BOSS_H - (TIP_Z + TAP_ENGAGE),
        align=(Align.CENTER, Align.CENTER, Align.MAX))
    p -= Pos(0, 0, TIP_Z + TAP_ENGAGE) * Cylinder(
        M3_TAP / 2, TAP_ENGAGE + 0.5,
        align=(Align.CENTER, Align.CENTER, Align.MAX))
    p.label = "stenciljig_knob_rod"
    p.color = GREEN
    return p


def build_handle() -> Part:
    p = Cylinder(HANDLE_D / 2, HANDLE_T,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    for i in range(8):
        a = math.radians(i * 45 + 22.5)
        p -= Pos(9.0 * math.cos(a), 9.0 * math.sin(a), -0.1) * \
            Cylinder(1.3, HANDLE_T + 0.2,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = chamfer(p.edges().group_by(Axis.Z)[-1], 0.5)
    p = chamfer(p.edges().group_by(Axis.Z)[0], 0.3)
    # T-direction groove, interrupted by the spine head
    p -= _box(-7.5, 7.5, -0.6, 0.6, HANDLE_T - 0.6, HANDLE_T + 0.1)
    p -= extrude(offset(boss_profile(), SOCKET_FIT), BOSS_H + 0.1)
    p -= Cylinder(M3_FREE / 2, 2 * HANDLE_T)
    p -= Pos(0, 0, SEAT_Z) * Cylinder(
        BTN_CBORE_D / 2, BTN_CBORE + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = "stenciljig_knob_handle"
    p.color = GREEN
    return p


def build_knob() -> Part:
    """Assembled rod + handle (they fuse at the z=0 shoulder) for
    scenes and interference checks - not a print."""
    p = build_rod() + build_handle()
    p.label = "stenciljig_knob"
    p.color = GREEN
    return p

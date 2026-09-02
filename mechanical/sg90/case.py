"""SG90 case: bottom cap, middle body, top cap with output boss.

Global frame: Z up, z=0 at the case bottom exterior, length along X,
pot/output axis toward -X, motor toward +X. build() returns the three
sections already in assembled position.
"""
from build123d import *

from . import measurements as m

# axis chain anchored with the central arbor at x=0; the chain runs
# 0.66 longer than the clear interior, so the motor can apex nests 0.35
# and the pot bore 0.46 into the 1.0 end walls (donor-style thinning)
ARBOR_AXIS_X = 0.0
MOTOR_AXIS_X = ARBOR_AXIS_X + m.AXIS_ARBOR_MOTOR_DESIGN  # +5.70
POT_AXIS_X = ARBOR_AXIS_X - m.AXIS_POT_ARBOR_DESIGN      # -6.00
OUTPUT_AXIS_X = POT_AXIS_X

# measured section heights are the exterior shells; the assembled total
# runs 0.187 over their sum, so the lips hold a 0.0935 seam gap at each
# joint (sections cannot stack taller than their sum any other way)
JOINT_GAP = (m.CASE_TOTAL_H - m.CASE_BOT_H - m.CASE_MID_H - m.CASE_TOP_H) / 2
MID_Z0 = m.CASE_BOT_H + JOINT_GAP
MID_Z1 = MID_Z0 + m.CASE_MID_H
TOP_Z0 = MID_Z1 + JOINT_GAP
TOP_Z1 = TOP_Z0 + m.CASE_TOP_H  # == m.CASE_TOTAL_H
# the motor stack fixes the deck: rear boss rests on the bottom floor,
# then the can; the 0.6 front boss seats inside the 1.0-thick deck hole
DECK_Z0 = m.CASE_WALL + m.MOTOR_REAR_BOSS_L + m.MOTOR_BODY_L  # underside
DECK_Z1 = DECK_Z0 + m.CASE_DECK_T         # gear-chamber floor
CEIL_Z = TOP_Z1 - m.CASE_WALL             # flat gear-chamber ceiling
POT_FACE_Z = DECK_Z1                      # pot front face flush with deck
POT_BACK_Z = POT_FACE_Z - m.POT_BODY_L
BOSS_TOP_Z = TOP_Z1 + m.CASE_BOSS_H       # == m.CASE_TOTAL_H_BOSS
BOSS_CAV_Z1 = BOSS_TOP_Z - m.CASE_BOSS_CAP_T_EST  # boss cavity ceiling
SHAFT_TIP_Z = DECK_Z0 + m.MOTOR_FRONT_BOSS_L + m.MOTOR_SHAFT_L
SCREW_TIP_Z = m.CASE_SCREW_CBORE_DEPTH_EST + m.SCREW_L  # ~= CEIL_Z + 0.07

# donor cross-checks: shaft tip meets the pinion span; big wheels stay
# under the flat ceiling; the gear3 stack tip clears the lobe arbor boss
assert abs(SHAFT_TIP_Z - (DECK_Z1 + m.GEAR_FLOOR_CLEAR + m.PINION_H)) < 0.05
assert DECK_Z1 + m.GEAR_FLOOR_CLEAR + m.GEAR1_TOTAL_H + m.GEAR3_H < CEIL_Z
assert DECK_Z1 + m.GEAR_FLOOR_CLEAR + m.GEAR1_TOTAL_H + m.GEAR3_TOTAL_H \
    < BOSS_CAV_Z1 - m.CASE_TOP_ARBOR_BOSS_H_EST

_IN_L = m.CASE_OUT_L - 2 * m.CASE_WALL
_IN_W = m.CASE_OUT_W - 2 * m.CASE_WALL
# caliper diag 3.5 = corner tip to far column face -> column r ~1.2
SCREW_COL_R = m.CASE_SCREW_DIAG - (
    Vector(m.CASE_OUT_L, m.CASE_OUT_W) - Vector(m.SCREW_CC_L, m.SCREW_CC_W)
).length / 2


def _outer_plan():
    return RectangleRounded(m.CASE_OUT_L, m.CASE_OUT_W, m.CASE_CORNER_R_EST)


def _inner_plan(grow=0.0):
    return RectangleRounded(_IN_L + 2 * grow, _IN_W + 2 * grow, 0.3)


def _at_cols(shape):
    p = Part()
    for sx in (1, -1):
        for sy in (1, -1):
            p += Pos(sx * m.SCREW_CC_L / 2, sy * m.SCREW_CC_W / 2, 0) * shape
    return p


def _columns(z0, z1):
    return _at_cols(Pos(0, 0, z0) * Cylinder(
        SCREW_COL_R, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)))


def _pilots(z0, z1):
    return _at_cols(Pos(0, 0, z0) * Cylinder(
        m.CASE_SCREW_PILOT_D_EST / 2, z1 - z0,
        align=(Align.CENTER, Align.CENTER, Align.MIN)))


def _lip(z0, z1, anchor_up, ends_only=False):
    """Joint lip: ring protruding over z0..z1 into a cap (0.05 fit), plus
    an anchor band lapping 0.1 into the shell walls so it fuses."""
    inset = m.CASE_LIP_CLEAR_EST
    inner = _inner_plan(-inset - m.CASE_LIP_T_EST)
    ring = extrude(Pos(0, 0, z0) * (_inner_plan(-inset) - inner), z1 - z0)
    az = z1 if anchor_up else z0 - 1.0
    ring += extrude(Pos(0, 0, az) * (_inner_plan(0.1) - inner), 1.0)
    if ends_only:
        # gear1/3 (r 4.9) sweep past the lip line along the sides - only
        # the end segments survive at the top joint
        ring -= Pos(0, 0, (z0 + z1) / 2) * Box(17.0, m.CASE_OUT_W + 2, 20)
    # interrupted at the corners so the screw columns pass
    ring -= _at_cols(Pos(0, 0, (z0 + z1) / 2) * Cylinder(
        SCREW_COL_R + 0.5, 2 * (z1 - z0) + 3))
    return ring


def _motor_pocket_plan():
    r = m.MOTOR_BODY_D / 2 + m.CASE_MOTOR_CLEAR_EST
    return Pos(MOTOR_AXIS_X, 0) * Circle(r) & Pos(MOTOR_AXIS_X, 0) * \
        Rectangle(2 * r + 0.2, m.MOTOR_BODY_FLAT + 2 * m.CASE_MOTOR_CLEAR_EST)


def build_bottom() -> Part:
    part = extrude(_outer_plan(), m.CASE_BOT_H)
    part -= extrude(Pos(0, 0, m.CASE_WALL) * _inner_plan(), m.CASE_BOT_H)
    part += _columns(m.CASE_WALL, m.CASE_BOT_H)
    part -= _pilots(-0.1, m.CASE_BOT_H + 0.1)
    part -= _at_cols(Pos(0, 0, -0.1) * Cylinder(
        m.CASE_SCREW_CBORE_D_EST / 2, m.CASE_SCREW_CBORE_DEPTH_EST + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN)))
    # motor can spans this cap (boss on the floor); its apex nests 0.35
    # into the +X wall - relieve above the floor only
    part -= extrude(Pos(0, 0, m.CASE_WALL) * _motor_pocket_plan(),
                    m.CASE_BOT_H - m.CASE_WALL + 0.1)
    return part


def build_middle() -> Part:
    part = extrude(Pos(0, 0, MID_Z0) * _outer_plan(), m.CASE_MID_H)
    part -= extrude(Pos(0, 0, MID_Z0 - 0.1) * _inner_plan(),
                    m.CASE_MID_H + 0.2)

    # deck (gear-chamber floor) + arbor lower boss
    part += extrude(Pos(0, 0, DECK_Z0) * _inner_plan(0.1), m.CASE_DECK_T)
    part += Pos(ARBOR_AXIS_X, 0, DECK_Z1) * Cylinder(
        m.CASE_ARBOR_BOSS_D_EST / 2, m.CASE_ARBOR_BOSS_H_EST,
        align=(Align.CENTER, Align.CENTER, Align.MIN))

    # motor compartment: solid block below the deck (pocket cut later)
    r_can = m.MOTOR_BODY_D / 2 + m.CASE_MOTOR_CLEAR_EST
    x0 = MOTOR_AXIS_X - r_can - 1.0  # 1.0 cross-wall at the -X apex
    x1 = _IN_L / 2 + 0.5
    block = Pos((x0 + x1) / 2, 0, DECK_Z0 - m.CASE_MOTOR_POCKET_H) * Box(
        x1 - x0, _IN_W, m.CASE_MOTOR_POCKET_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    part += block & extrude(
        Pos(0, 0, DECK_Z0 - m.CASE_MOTOR_POCKET_H) * _inner_plan(0.1),
        m.CASE_MOTOR_POCKET_H)

    # pot pocket: bore wall below the deck + seat ring, clipped to shell
    wall = Pos(POT_AXIS_X, 0, POT_BACK_Z) * Cylinder(
        6.0, DECK_Z0 - POT_BACK_Z + 0.2,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    seat = Pos(POT_AXIS_X, 0, POT_BACK_Z - m.CASE_POT_SEAT_T_EST) * Cylinder(
        6.0, m.CASE_POT_SEAT_T_EST,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    part += (wall + seat) & extrude(_outer_plan(), m.CASE_MID_H + 20)

    # mounting ears on the top edge
    ear_l = m.EAR_SPAN / 2 - _IN_L / 2  # overlaps the wall for a clean fuse
    for sx in (1, -1):
        ear = extrude(
            Pos(sx * (m.EAR_SPAN / 2 - ear_l / 2), 0, MID_Z1 - m.EAR_T) *
            RectangleRounded(ear_l, m.EAR_W_EST, 1.0), m.EAR_T)
        ear -= Pos(sx * m.EAR_HOLE_CC / 2, 0, MID_Z1 - m.EAR_T - 0.1) * \
            Cylinder(m.EAR_HOLE_D / 2, m.EAR_T + 0.2,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))
        part += ear

    # screw columns + joint lips into both caps
    lip_lo = MID_Z0 - JOINT_GAP - m.CASE_LIP_H_EST
    lip_hi = MID_Z1 + JOINT_GAP + m.CASE_LIP_H_EST
    part += _columns(MID_Z0, MID_Z1)
    part += _lip(lip_lo, MID_Z0, anchor_up=True)
    part += _lip(MID_Z1, lip_hi, anchor_up=False, ends_only=True)

    # motor pocket: can outline cut down through the bottom lip
    part -= extrude(Pos(0, 0, lip_lo - 0.1) * _motor_pocket_plan(),
                    DECK_Z0 - lip_lo + 0.1)

    # pot/gear bore: pot fit below the deck, continues up as clearance
    # for gear2/gear4 (r 4.89 at the pot axis outruns the 1.0 end wall)
    part -= Pos(POT_AXIS_X, 0, POT_BACK_Z) * Cylinder(
        m.CASE_POT_POCKET_D_EST / 2, lip_hi - POT_BACK_Z + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= Pos(POT_AXIS_X, 0, POT_BACK_Z - m.CASE_POT_SEAT_T_EST - 0.1) * \
        Cylinder(m.CASE_POT_SEAT_ID_EST / 2, m.CASE_POT_SEAT_T_EST + 0.2,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= Pos(MOTOR_AXIS_X, 0, DECK_Z0 - 0.5) * Cylinder(
        m.CASE_DECK_HOLE_D / 2, m.CASE_DECK_T + 1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= Pos(ARBOR_AXIS_X, 0, DECK_Z1 + m.CASE_ARBOR_BOSS_H_EST - 1.4) * \
        Cylinder(m.CASE_ARBOR_HOLE_D_EST / 2, 1.5,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))

    # wire exit: notch through the pot-end wall and lip, open to the
    # bottom edge
    part -= Pos(-m.CASE_OUT_L / 2, 0, lip_lo - 0.1) * Box(
        2 * (m.CASE_WALL + m.CASE_LIP_T_EST + m.CASE_LIP_CLEAR_EST) + 0.4,
        m.CASE_WIRE_SLOT_W_EST,
        MID_Z0 + m.CASE_WIRE_SLOT_H_EST - lip_lo + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))

    part -= _pilots(lip_lo - 0.1, lip_hi + 0.1)
    return part


def build_top() -> Part:
    part = extrude(Pos(0, 0, TOP_Z0) * _outer_plan(), m.CASE_TOP_H)
    part -= extrude(Pos(0, 0, TOP_Z0 - 0.1) * _inner_plan(),
                    CEIL_Z - TOP_Z0 + 0.1)

    # twin boss like the reference: round output boss truncated flush at
    # the -X case end + smaller lobe over the arbor, both full height
    boss = Pos(POT_AXIS_X, 0, TOP_Z1) * Cylinder(
        m.CASE_BOSS_D_EST / 2, m.CASE_BOSS_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    boss += Pos(ARBOR_AXIS_X, 0, TOP_Z1) * Cylinder(
        m.CASE_ARBOR_LOBE_D_EST / 2, m.CASE_BOSS_H,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    part += boss & extrude(Pos(0, 0, TOP_Z1 - 0.1) * _outer_plan(),
                           m.CASE_BOSS_H + 0.2)
    part += _columns(TOP_Z0, CEIL_Z + 0.1)

    # boss cavity: gear4's wheel and the gear3/4 stack tips rise past the
    # flat ceiling, so both boss interiors open down to the ceiling plane
    cav = Pos(POT_AXIS_X, 0, CEIL_Z) * Cylinder(
        m.CASE_BOSS_CAVITY_D_EST / 2, BOSS_CAV_Z1 - CEIL_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    cav += Pos(ARBOR_AXIS_X, 0, CEIL_Z) * Cylinder(
        m.CASE_ARBOR_LOBE_HOLE_D_EST / 2, BOSS_CAV_Z1 - CEIL_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= cav

    # arbor tip boss hangs from the lobe cap; blind bore, 0.5 left above
    part += Pos(ARBOR_AXIS_X, 0, BOSS_CAV_Z1 - m.CASE_TOP_ARBOR_BOSS_H_EST) \
        * Cylinder(m.CASE_TOP_ARBOR_BOSS_D_EST / 2,
                   m.CASE_TOP_ARBOR_BOSS_H_EST + 0.1,
                   align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= Pos(ARBOR_AXIS_X, 0,
                BOSS_CAV_Z1 - m.CASE_TOP_ARBOR_BOSS_H_EST - 0.1) * \
        Cylinder(m.CASE_TOP_ARBOR_HOLE_D_EST / 2,
                 m.CASE_TOP_ARBOR_BOSS_H_EST + m.CASE_BOSS_CAP_T_EST - 0.4,
                 align=(Align.CENTER, Align.CENTER, Align.MIN))

    # gear2/gear4 clearance: same bore as the middle, thins the -X wall
    part -= Pos(POT_AXIS_X, 0, TOP_Z0 - 0.1) * Cylinder(
        m.CASE_POT_POCKET_D_EST / 2, CEIL_Z - TOP_Z0 + 0.2,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    # output opening through the boss cap; gear4 boss/spline pass here
    part -= Pos(POT_AXIS_X, 0, BOSS_CAV_Z1 - 0.1) * Cylinder(
        m.CASE_BOSS_HOLE_D_EST / 2, m.CASE_BOSS_CAP_T_EST + 0.2,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    # screw tips land at the ceiling plane (SCREW_TIP_Z); pilots follow
    part -= _pilots(TOP_Z0 - 0.1, SCREW_TIP_Z + 0.2)
    return part


def build() -> Compound:
    parts = []
    for builder, label in ((build_bottom, "case_bottom"),
                           (build_middle, "case_middle"),
                           (build_top, "case_top")):
        p = builder()
        p.label = label
        p.color = Color(0.15, 0.25, 0.75, 0.6)
        parts.append(p)
    return Compound(label="sg90_case", children=parts)


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "sg90_case.step"))
    export_stl(part, str(out / "sg90_case.stl"), tolerance=0.01)
    from blueprint import blueprint
    blueprint(part, str(out / "sg90_case_bp.svg"), name="sg90_case")
    print(f"sg90_case: volume {part.volume:.0f} mm^3 -> {out}")

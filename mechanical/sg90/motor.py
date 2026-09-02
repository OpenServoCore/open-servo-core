"""M10 flat-can motor (8x10x12.3 donor; FF-M20SA dwg is the 15mm
sibling). Axis = Z, rear face at z=0, shaft up."""
import math

from build123d import *

from . import measurements as m

# Internals are proportion-estimates: donor teardown photos plus the wound
# rotor in reference/servo.STEP (rotor OD 5.75, commutator 2.43 in a
# 6.83-flat can, scaled to this 8.0-flat can). Exterior stays MOTOR_*.
CAN_WALL_EST = 0.3         # drawn-steel can wall
VENT_D_EST = 1.0           # vent holes on the flats, brush region
VENT_X_EST = 2.0           # hole offset +/- along the round axis
VENT_Z_EST = 2.8
CARD_T_EST = 1.2           # brush card disc thickness
BUSH_SLEEVE_D_EST = 2.0    # brass bushing sleeve through the card
BUSH_TOP_EST = 2.1         # sleeve top above rear face
OILER_OD_EST = 3.4         # felt oiler ring on the card
OILER_T_EST = 0.8
COMM_D_EST = 2.8
COMM_Z_EST = 2.7
COMM_L_EST = 2.2
COMM_GAP_EST = 0.12        # slot between the 3 segments
ROTOR_OD_EST = 6.6
ROTOR_HUB_D_EST = 2.2
WEB_W_EST = 1.3            # T-pole web width
SHOE_T_EST = 0.55          # pole shoe radial thickness
SHOE_ARC_EST = 100.0       # pole shoe arc, deg
ROTOR_Z_EST = 6.0
STACK_EST = 4.6            # lamination stack height
COIL_T_EST = 0.85          # winding build per side of the web
COIL_OVER_EST = 1.0        # end-turn overhang beyond the stack
MAG_T_EST = 1.2            # ferrite arc radial thickness
MAG_ARC_EST = 110.0
MAG_Z_EST = 4.4
MAG_L_EST = 7.6
WASHER_OD_EST = 2.4
WASHER_T_EST = 0.15
BRUSH_W_EST = 0.6          # brush spring strip width
BRUSH_T_EST = 0.12


def _sector(r_out, r_in, arc_deg):
    # annulus segment centered on +X
    half = arc_deg / 2
    pts = [(0.0, 0.0)] + [
        (2 * r_out * math.cos(math.radians(-half + i * arc_deg / 12)),
         2 * r_out * math.sin(math.radians(-half + i * arc_deg / 12)))
        for i in range(13)]
    return (Circle(r_out) - Circle(r_in)) & Polygon(*pts, align=None)


def _flat_profile(r, flat):
    return Circle(r) & Rectangle(4 * r, flat)


def build() -> Compound:
    bore_r = m.MOTOR_BODY_D / 2 - CAN_WALL_EST
    bore_flat = m.MOTOR_BODY_FLAT - 2 * CAN_WALL_EST
    bore_top = m.MOTOR_BODY_L - CAN_WALL_EST
    shaft_r = m.MOTOR_SHAFT_D / 2

    # --- can: shell open to the rear, plus front plate and boss ---
    can = extrude(_flat_profile(m.MOTOR_BODY_D / 2, m.MOTOR_BODY_FLAT),
                  m.MOTOR_BODY_L)
    can -= extrude(_flat_profile(bore_r, bore_flat), bore_top)
    z = m.MOTOR_BODY_L
    if m.MOTOR_FRONT_PLATE_L > 0:
        can += Pos(0, 0, z) * Cylinder(
            m.MOTOR_FRONT_PLATE_D / 2, m.MOTOR_FRONT_PLATE_L,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        z += m.MOTOR_FRONT_PLATE_L
    can += Pos(0, 0, z) * Cylinder(
        m.MOTOR_FRONT_BOSS_D / 2, m.MOTOR_FRONT_BOSS_L,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    boss_top = z + m.MOTOR_FRONT_BOSS_L
    can -= Pos(0, 0, bore_top - 0.1) * Cylinder(
        shaft_r + 0.05, boss_top - bore_top + 0.2,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    for sx in (-1, 1):  # through both flats
        can -= Pos(sx * VENT_X_EST, 0, VENT_Z_EST) * Rot(90, 0, 0) * Cylinder(
            VENT_D_EST / 2, m.MOTOR_BODY_FLAT + 2)

    # --- shaft: bushing to tip, exterior stickout unchanged ---
    shaft = Pos(0, 0, 0.3) * Cylinder(
        shaft_r, boss_top + m.MOTOR_SHAFT_L - 0.3,
        align=(Align.CENTER, Align.CENTER, Align.MIN))

    # --- magnets: two ferrite arcs on the round sides ---
    mag2d = _sector(bore_r, bore_r - MAG_T_EST, MAG_ARC_EST)
    mag2d &= _flat_profile(bore_r, bore_flat)
    magnets = extrude(Pos(0, 0, MAG_Z_EST) * (mag2d + Rot(0, 0, 180) * mag2d),
                      MAG_L_EST)

    # --- rotor: 3-pole T-core, one solid stack ---
    shoe_in = ROTOR_OD_EST / 2 - SHOE_T_EST
    arm = Pos(shoe_in / 2, 0) * Rectangle(shoe_in, WEB_W_EST)
    arm += _sector(ROTOR_OD_EST / 2, shoe_in, SHOE_ARC_EST)
    core2d = Circle(ROTOR_HUB_D_EST / 2)
    for a in (90, 210, 330):
        core2d += Rot(0, 0, a) * arm
    rotor = extrude(Pos(0, 0, ROTOR_Z_EST) * core2d, STACK_EST)
    rotor -= Cylinder(shaft_r + 0.02, 4 * m.MOTOR_BODY_L)

    # --- coils: smooth rounded blocks around the webs ---
    coil_w = WEB_W_EST + 2 * COIL_T_EST
    coil_in = ROTOR_HUB_D_EST / 2 + 0.05
    coil2d = Pos((coil_in + shoe_in - 0.05) / 2, 0) * RectangleRounded(
        shoe_in - 0.05 - coil_in, coil_w, 0.5)
    coils = Part()
    for a in (90, 210, 330):
        c = extrude(Pos(0, 0, ROTOR_Z_EST - COIL_OVER_EST) * Rot(0, 0, a)
                    * coil2d, STACK_EST + 2 * COIL_OVER_EST)
        coils += fillet(c.edges(), 0.3)
    coils -= rotor

    # --- commutator: 3 brass segments on the shaft ---
    comm = Pos(0, 0, COMM_Z_EST) * Cylinder(
        COMM_D_EST / 2, COMM_L_EST,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    comm -= Cylinder(shaft_r + 0.02, 4 * m.MOTOR_BODY_L)
    for a in (30, 150, 270):
        comm -= Rot(0, 0, a) * Pos(
            (COMM_D_EST + 0.4) / 4 + 0.2, 0,
            COMM_Z_EST + COMM_L_EST / 2) * Box(
                COMM_D_EST / 2 + 0.4, COMM_GAP_EST, COMM_L_EST + 0.2)

    # --- brush card: black disc closing the rear, spring posts ---
    card = extrude(_flat_profile(bore_r - 0.05, bore_flat - 0.1), CARD_T_EST)
    card -= Cylinder(BUSH_SLEEVE_D_EST / 2 + 0.05, 4 * CARD_T_EST)
    for s in (1, -1):
        card += Pos(s * 2.7, -s * 2.35, CARD_T_EST) * Cylinder(
            0.5, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))

    # --- bushing: rear boss is the brass bearing, sleeve through the card ---
    bushing = Pos(0, 0, -m.MOTOR_REAR_BOSS_L) * Cylinder(
        m.MOTOR_REAR_BOSS_D / 2, m.MOTOR_REAR_BOSS_L,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    bushing += Cylinder(BUSH_SLEEVE_D_EST / 2, BUSH_TOP_EST,
                        align=(Align.CENTER, Align.CENTER, Align.MIN))
    bushing -= Pos(0, 0, 0.25) * Cylinder(
        shaft_r + 0.02, BUSH_TOP_EST,
        align=(Align.CENTER, Align.CENTER, Align.MIN))

    # --- felt oiler around the sleeve ---
    oiler = extrude(Pos(0, 0, CARD_T_EST) * (
        Circle(OILER_OD_EST / 2) - Circle(BUSH_SLEEVE_D_EST / 2 + 0.02)),
        OILER_T_EST)

    # --- brushes: two bent strips, card to commutator, 180-deg symmetric ---
    contact_r = COMM_D_EST / 2 + 0.05
    contact_z = COMM_Z_EST + COMM_L_EST / 2
    ca = math.radians(15)
    s_pt = Vector(3.1, -2.2, CARD_T_EST + 0.15)
    e_pt = Vector((contact_r + 0.4) * math.cos(ca),
                  (contact_r + 0.4) * math.sin(ca), contact_z)
    path = Polyline(s_pt, e_pt)
    brush = sweep((path ^ 0) * Rectangle(BRUSH_W_EST, BRUSH_T_EST), path=path)
    brush += Pos(s_pt.X, s_pt.Y, CARD_T_EST) * Box(
        0.6, 0.6, 0.15, align=(Align.CENTER, Align.CENTER, Align.MIN))
    brush += Rot(0, 0, 15) * Pos(contact_r + 0.2, 0, contact_z) * Box(
        0.45, BRUSH_W_EST, 0.8)
    brushes = brush + Rot(0, 0, 180) * brush

    # --- terminals ---
    terms = Part()
    for sx in (-1, 1):
        tab = Pos(sx * m.MOTOR_TERM_SPAN / 2, 0, -m.MOTOR_TERM_L) * Box(
            m.MOTOR_TERM_T, m.MOTOR_TERM_W, m.MOTOR_TERM_L,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        tab -= Pos(sx * m.MOTOR_TERM_SPAN / 2, 0,
                   -m.MOTOR_TERM_L + 0.5) * Rot(0, 90, 0) * Cylinder(
                       m.MOTOR_TERM_HOLE / 2, 2)
        terms += tab

    # --- thrust washers: rear above the oiler, front under the can face ---
    ring = Circle(WASHER_OD_EST / 2) - Circle(shaft_r + 0.05)
    washers = extrude(Pos(0, 0, COMM_Z_EST - 0.25) * ring, WASHER_T_EST)
    washers += extrude(
        Pos(0, 0, bore_top - WASHER_T_EST) * ring, WASHER_T_EST)

    for p, label, color in (
            (can, "can", (0.42, 0.43, 0.45)),
            (magnets, "magnets", (0.22, 0.22, 0.24)),
            (rotor, "rotor", (0.32, 0.33, 0.36)),
            (coils, "coils", (0.72, 0.45, 0.20)),
            (comm, "commutator", (0.72, 0.58, 0.28)),
            (card, "brush_card", (0.10, 0.10, 0.10)),
            (bushing, "bushing", (0.72, 0.58, 0.28)),
            (oiler, "oiler", (0.72, 0.62, 0.42)),
            (brushes, "brushes", (0.78, 0.78, 0.80)),
            (terms, "terminals", (0.80, 0.80, 0.82)),
            (washers, "washers", (0.70, 0.70, 0.72)),
            (shaft, "shaft", (0.75, 0.76, 0.78))):
        p.label = label
        p.color = Color(*color)
    return Compound(label="sg90_motor", children=[
        can, magnets, rotor, coils, comm, card, bushing, oiler, brushes,
        terms, washers, shaft])


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    part = build()
    export_step(part, str(out / "sg90_motor.step"))
    export_stl(part, str(out / "sg90_motor.stl"), tolerance=0.01)
    from blueprint import blueprint
    blueprint(part, str(out / "sg90_motor_bp.svg"), name="sg90_motor")
    print(f"sg90_motor: volume {part.volume:.0f} mm^3 -> {out}")

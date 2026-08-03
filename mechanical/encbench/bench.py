"""Bench encoder rig, modular: FF-M20 clamped to the base, ITR1204
coupon in a height-adjustable wall sled, printed discs on the pinion.

Frame: motor axis = X (+X toward the coupon), Z up, base top = z0,
nominal motor rear face = x0, axis at z = AXIS_Z. Sled geometry is
modeled at the NOMINAL (topmost, 90 deg) position.

Parts (one print each, color per part):
- base: desk-clamp plate (no holes) + motor saddle + M2 clamp ears +
  rear fin with two M3 screw holes; M3 hex nuts drop into pockets in
  the fin's back face, retained by the screws.
- sled (BLACK PETG - the wall is the optical shroud): rectangular wall
  with the coupon pocket stamped from the real PCB outline (projected
  from encbench/build/encoder-board.step, regenerate with `kicad-cli
  pcb export step --subst-models -o mechanical/encbench/build/encoder-board.step
  hardware/boards/encoder-board/encoder-board.kicad_pcb`), sensor
  windows + J1 tail relief in the 0.4 front wall, and two wings whose
  open-top slots hang on the fin screws. Slots are cut snug to the M3
  shank (3.3 drawn) - the screws themselves register the sled laterally.
  Topmost position (screws at the slot bottoms) = 90 deg quadrature.
  Press the board in with the sled OFF the stand.
- clamp: bridges the motor top flat, 0.1 squeeze, two M2 screws into
  nuts inserted from BELOW the ears. Print upside down.
- discs: Ø10 (presets 90/62) and Ø20 (preset 31); press on the pinion
  against a flat table until the shaft tip touches - flush.
- shims: height gauges under the sled wings; the shim, not screw
  friction, carries the position (notch count = preset index).

Height presets - sled drop D moves the sensor chord below the axis by
d = 1.5 + D, phase = 2*atan(1.5/d), pattern radius r = hypot(1.5, d):
  90 deg: d 1.50, r 2.12 (Ø10 disc)   - shim 1 notch, D 0
  62 deg: d 2.49, r 2.90 (Ø10 disc)   - the REAL 7 mm PCB-disc ring
  31 deg: d 5.39, r 5.60 (Ø20 disc)
Firmware ellipse-cal absorbs non-90 phases; measured A/B phase is the
truth after any height change.

x chain at nominal: rear face 0, shaft tip 15.6 = disc face, paper 0.1,
air gap 1.0, sensor faces 16.7 (0.25 proud of the 16.95 wall face),
coupon front face 17.35. The coupon faces -X; the motor slides axially
in the saddle to set the gap, the disc rides through the saddle bore.
"""
import math
import pathlib

from build123d import *

from sg90 import measurements as m

# --- coupon (hardware/boards/encoder-board) [pcb] ---
BOARD_W = 25.0
BOARD_T = 1.6
SENSOR_H = 0.65            # ITR1204 body [ds]
SENSOR_BODY = (1.4, 1.9)   # body footprint: across pin pair, along dies
SENSE_CHORD = 3.0          # U1-U2 centers
CHORD_UP = 5.0             # sensor chord above the board's low edge
WIN_CLEAR = 0.3            # per side, sensor windows (small holes shrink)
POCKET_CLEAR = 0.25        # total outline growth; MK3S+ PETG pockets
                           # under-print ~0.15 -> true snug press

PAPER_T_EST = 0.1
AIR_GAP_EST = 1.0

AXIS_Z = 14.0
TIP_X = m.MOTOR_BODY_L + m.MOTOR_FRONT_BOSS_L + m.MOTOR_SHAFT_L  # 15.6
SENSE_X = TIP_X + PAPER_T_EST + AIR_GAP_EST                      # 16.7
BOARD_X = SENSE_X + SENSOR_H       # 17.35 coupon front face

# d = chord-to-axis offset; nominal 1.5 -> 90 deg
PRESETS = (("90", 1.5, 10.0), ("62", 2.487, 10.0), ("31", 5.39, 20.0))

# --- sled wall + wings ---
FACE_WALL = 0.4            # front wall = 2 clean 0.2 layers (printed
                           # flat); sensors poke 0.25 proud
WALL_X = (BOARD_X - FACE_WALL, BOARD_X + BOARD_T + 0.35)  # 16.95 .. 19.3
WALL_W = 39.0              # spans the wing footprint: sled prints flat,
                           # front face down, support-free
WALL_Z = (6.0, 34.5)       # nominal; drops by up to ~5.8
WING_Y = (12.7, 19.5)
WING_Z = (6.0, 20.0)
FIN_X = (21.3, 24.3)       # flange back face = fin front face
M3_SCREW_Y = 16.1
M3_SCREW_Z = 13.0
M3_SLOT_W = 3.3            # prints ~3.15 -> snug on the shank = registration
M3_HOLE = 3.6              # horizontal hole in the fin, sags oval
M3_NUT_AF = 6.0            # 5.5 nut + fit, horizontal-axis pocket
M3_NUT_T = 2.8             # 2.4 nut + fit

# --- disc ---
DISC_T = 2.0               # 3d-printed
DISC_BORE = 2.2            # prints ~2.0-2.1 -> broach fit on the 2.23
                           # pinion tip circle
DISC_RECESS_T = 0.4        # paper pattern seat

# --- motor saddle + clamp (M2 screws, nuts from below) ---
SADDLE_X = (1.9, 10.2)     # ~70% of the 12.3 can, slide room both ways
SADDLE_CLEAR = 0.25        # radial, sides only - floor is exact (z)
EAR_TOP = 12.0
EAR_Y = (5.6, 12.4)        # wide enough for the nut chimney
SCREW_X = (SADDLE_X[0] + SADDLE_X[1]) / 2
SCREW_Y = 9.0              # ear center
M2_HOLE = 2.4
M2_NUT_AF = 4.5            # 4.0 nut + XY shrink allowance
M2_NUT_T = 1.9             # 1.6 nut + fit
M2_NUT_Z = 8.4             # hex pocket floor
CHIMNEY_D = 5.2            # prints ~5.0; nut corner circle 4.62
CLAMP_Z = (12.3, 19.6)     # bottom floats 0.3 over the ears = travel
CLAMP_SQUEEZE = 0.1        # interference at the motor top flat

BASE_T = 4.0

# board-step -> stand frame: (X,Y,Z) -> (Z, -X, -Y) + offsets; B face
# (step z0) lands on the pocket floor at BOARD_X, J1 edge up
BOARD_LOC = (Pos(BOARD_X, 112.5, -92.5) * Rot(180, 0, 0) * Rot(90, 0, 0)
             * Rot(0, 90, 0))


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def board_outline_face() -> Face:
    """Coupon outline from the exported STEP: the substrate's B-side
    face, outer wire only (pads/vias dropped)."""
    step = pathlib.Path(__file__).parent / "build" / "encoder-board.step"
    comp = import_step(str(step))
    sub = next(s for s in comp.solids()
               if abs(s.bounding_box().size.X - BOARD_W) < 0.5
               and abs(s.bounding_box().size.Y - BOARD_W) < 0.5)
    flat = [f for f in sub.faces()
            if abs(f.normal_at(f.center()).Z) > 0.99]
    bface = min(flat, key=lambda f: f.center().Z)
    return Face(bface.outer_wire())


def build_base() -> Part:
    flat_half = m.MOTOR_BODY_FLAT / 2

    p = Pos(9.0, 0, -BASE_T) * extrude(RectangleRounded(44.0, 41.0, 4.0),
                                       BASE_T)

    # motor saddle: bore floor exact at the bottom flat, sides +0.25
    p += _box(SADDLE_X[0], SADDLE_X[1], -6.5, 6.5, 0, 12.0)
    prof = Circle(m.MOTOR_BODY_D / 2 + SADDLE_CLEAR) & Pos(0, 25 - flat_half) \
        * Rectangle(40, 50)
    p -= Pos(SADDLE_X[0] - 0.1, 0, AXIS_Z) * extrude(
        Plane.YZ * prof, SADDLE_X[1] - SADDLE_X[0] + 0.2)

    # clamp ears: solid columns, support-free; the nut rides up the
    # chimney from under the base, a cone lead-in centers it, the screw
    # draws it into the hex pocket
    for s in (1, -1):
        p += _box(SADDLE_X[0] + 0.7, SADDLE_X[1] - 0.3,
                  s * EAR_Y[0] - s * 0.2, s * EAR_Y[1], 0, EAR_TOP)
        loc = Pos(SCREW_X, s * SCREW_Y, 0)
        p -= loc * Pos(0, 0, -BASE_T - 0.1) * Cylinder(
            CHIMNEY_D / 2, M2_NUT_Z + BASE_T + 0.2,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= loc * Pos(0, 0, M2_NUT_Z - 0.01) * Cone(
            CHIMNEY_D / 2 + 0.1, M2_NUT_AF / math.sqrt(3), 0.5,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= loc * Pos(0, 0, M2_NUT_Z) * extrude(
            RegularPolygon(M2_NUT_AF / math.sqrt(3), 6), M2_NUT_T)
        p -= loc * Pos(0, 0, M2_NUT_Z) * Cylinder(
            M2_HOLE / 2, EAR_TOP - M2_NUT_Z + 0.1,
            align=(Align.CENTER, Align.CENTER, Align.MIN))

    # rear fin: M3 through-holes, hex pockets sunk from the back face
    p += _box(FIN_X[0], FIN_X[1], -19.5, 19.5, 0, WING_Z[1])
    for s in (1, -1):
        p -= Pos(FIN_X[0] - 0.1, s * M3_SCREW_Y, M3_SCREW_Z) * Rot(0, 90, 0) \
            * Cylinder(M3_HOLE / 2, FIN_X[1] - FIN_X[0] + 0.2,
                       align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= Pos(FIN_X[1] + 0.1, s * M3_SCREW_Y, M3_SCREW_Z) * Rot(0, -90, 0) \
            * extrude(RegularPolygon(M3_NUT_AF / math.sqrt(3), 6),
                      M3_NUT_T + 0.1)

    p.label = "encbench_base"
    p.color = Color(0.35, 0.38, 0.42)
    return p


def build_sled(outline: Face) -> Part:
    """Wall + wings at the NOMINAL (90 deg, topmost) position. One flat
    part: full height over the board span, wing-height extensions
    outboard (so the slots stay open at the top for drop-on)."""
    p = _box(WALL_X[0], WALL_X[1], -15.0, 15.0, WALL_Z[0], WALL_Z[1])
    for s in (1, -1):
        p += _box(WALL_X[0], WALL_X[1], s * 15.0, s * WALL_W / 2,
                  WING_Z[0], WING_Z[1])

    # pocket = projected pcb outline grown by the press fit, from the back
    grown = BOARD_LOC * offset(outline, POCKET_CLEAR / 2)
    pocket = extrude(grown, WALL_X[1] - BOARD_X + 0.1)
    if pocket.bounding_box().min.X < BOARD_X - 0.05:
        pocket = extrude(grown, -(WALL_X[1] - BOARD_X + 0.1))
    p -= pocket

    # front-wall openings: sensor windows + J1 tail relief
    chord_z = AXIS_Z - 1.5
    wy = SENSOR_BODY[0] + 2 * WIN_CLEAR
    wz = SENSOR_BODY[1] + 2 * WIN_CLEAR
    for s in (1, -1):
        p -= _box(WALL_X[0] - 0.1, BOARD_X + 0.05,
                  s * SENSE_CHORD / 2 - wy / 2, s * SENSE_CHORD / 2 + wy / 2,
                  chord_z - wz / 2, chord_z + wz / 2)
    # J1 pin tails protrude 1.4 past the sensor face (step-verified
    # -3.8..4.4 x 30.2..30.8 in stand yz)
    p -= _box(WALL_X[0] - 0.1, BOARD_X + 0.05, -4.7, 5.3, 29.5, 31.5)

    # wings: doubler pads fully backed by the widened wall (flat print);
    # open-top slots cut through the full sled - heads bear on the front
    for s in (1, -1):
        p += _box(WALL_X[1], FIN_X[0], s * WING_Y[0], s * WING_Y[1],
                  WING_Z[0], WING_Z[1])
        p -= _box(WALL_X[0] - 0.1, FIN_X[0] + 0.1,
                  s * M3_SCREW_Y - M3_SLOT_W / 2,
                  s * M3_SCREW_Y + M3_SLOT_W / 2,
                  M3_SCREW_Z - M3_SLOT_W / 2, WING_Z[1] + 0.1)

    p.label = "encbench_sled"
    p.color = Color(0.08, 0.08, 0.08)
    return p


def build_clamp() -> Part:
    flat_half = m.MOTOR_BODY_FLAT / 2
    p = _box(SADDLE_X[0] + 1.1, SADDLE_X[1] - 0.7, -EAR_Y[1], EAR_Y[1],
             CLAMP_Z[0], CLAMP_Z[1])
    prof = Circle(m.MOTOR_BODY_D / 2 + 0.35) & Pos(
        0, flat_half - CLAMP_SQUEEZE - 25) * Rectangle(40, 50)
    p -= Pos(SADDLE_X[0] + 1.0, 0, AXIS_Z) * extrude(
        Plane.YZ * prof, SADDLE_X[1] - SADDLE_X[0] + 0.2)
    for s in (1, -1):
        p -= Pos(SCREW_X, s * SCREW_Y, CLAMP_Z[0] - 0.1) * Cylinder(
            M2_HOLE / 2, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= Pos(SCREW_X, s * SCREW_Y, 15.6) * Cylinder(
            4.6 / 2, CLAMP_Z[1] - 15.6 + 0.1,
            align=(Align.CENTER, Align.CENTER, Align.MIN))  # head well
    p.label = "encbench_clamp"
    p.color = Color(0.35, 0.38, 0.42)
    return p


def build_disc(dia: float) -> Part:
    p = extrude(Circle(dia / 2), DISC_T)
    p -= Pos(0, 0, DISC_T - DISC_RECESS_T) * Cylinder(
        (dia - 0.6) / 2, DISC_RECESS_T + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Cylinder(DISC_BORE / 2, 3 * DISC_T)
    p -= Cone(DISC_BORE / 2 + 0.35, DISC_BORE / 2, 0.4,
              align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = f"encbench_disc{dia:.0f}"
    p.color = Color(0.85, 0.83, 0.78)
    return p


def build_shim(drop: float, notches: int) -> Part:
    """Sits on the base under a sled wing: height = WING_Z[0] - drop.
    The shim, not screw friction, carries the sled position. Print two."""
    h = WING_Z[0] - drop
    p = _box(WALL_X[1] + 0.2, FIN_X[0] - 0.2, 0, 6.0, 0, h)
    for i in range(notches):
        p -= Pos((WALL_X[1] + FIN_X[0]) / 2, 1.2 + 1.6 * i, h) * Box(
            2.2, 0.8, 1.0)
    p.label = f"encbench_shim_d{drop:.1f}"
    p.color = Color(0.85, 0.83, 0.78)
    return p


def write_pattern_svg(path, r_sense, recess_d, ecc=0.35, copies=4):
    """Paper pattern: black disc, white eccentric circle at the sensing
    radius. Print 100% scale, cut on the grey line, punch the center."""
    r_cut = recess_d / 2
    pitch = 2 * r_cut + 3.0
    rows = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{copies * pitch}mm"',
        f'  height="{pitch}mm" viewBox="0 0 {copies * pitch} {pitch}">',
    ]
    for i in range(copies):
        cx = pitch / 2 + i * pitch
        cy = pitch / 2
        rows += [
            f'<circle cx="{cx}" cy="{cy}" r="{r_cut}" fill="black" '
            'stroke="grey" stroke-width="0.1"/>',
            f'<circle cx="{cx + ecc}" cy="{cy}" r="{r_sense:.3f}"'
            ' fill="white"/>',
            f'<circle cx="{cx}" cy="{cy}" r="0.25" fill="grey"/>',
        ]
    rows.append('</svg>')
    with open(path, "w") as f:
        f.write("\n".join(rows))


if __name__ == "__main__":
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    parts = [(build_base(), "encbench_base"),
             (build_sled(board_outline_face()), "encbench_sled"),
             (build_clamp(), "encbench_clamp")]
    for dia in sorted({d for _, _, d in PRESETS}):
        parts.append((build_disc(dia), f"encbench_disc{dia:.0f}"))
    for i, (name, d, dia) in enumerate(PRESETS):
        parts.append((build_shim(d - 1.5, i + 1), f"encbench_shim{name}"))
        r = math.hypot(1.5, d)
        write_pattern_svg(out / f"encbench_pattern{name}.svg", r, dia - 0.6)
    for part, name in parts:
        export_step(part, str(out / f"{name}.step"))
        export_stl(part, str(out / f"{name}.stl"), tolerance=0.01)
        print(f"{name}: volume {part.volume:.0f} mm^3 -> {out}")
    print(f"patterns -> {out}")

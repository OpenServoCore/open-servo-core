"""Bench encoder rig, modular: FF-M20 clamped to the base, ITR1204
coupon in a height-adjustable wall sled, printed discs on the pinion.

Frame: motor axis = X (+X toward the coupon), Z up, base top = z0,
nominal motor rear face = x0, axis at z = AXIS_Z. Sled geometry is
modeled at the NOMINAL (topmost, 90 deg) position.

Parts (one print each, color per part):
- base: desk-clamp plate (no holes) + motor saddle + M2 clamp ears +
  rear fin with two M3 screw holes; M3 hex nuts press into pockets in
  the fin's back face, permanent once seated.
- sled (BLACK PETG - the wall is the optical shroud): rectangular wall
  with the coupon pocket stamped from the real PCB outline (projected
  from encbench/build/encoder-board.step, regenerate with `kicad-cli
  pcb export step --subst-models -o mechanical/encbench/build/encoder-board.step
  hardware/boards/encoder-board/encoder-board.kicad_pcb`), sensor
  windows + J1 tail relief in the 0.8 front wall, and two wings whose
  open-top slots hang on the fin screws. Slots are cut snug to the M3
  shank (3.3 drawn) - the screws themselves register the sled laterally.
  Topmost position (screws at the slot bottoms) = 90 deg quadrature.
  Press the board in with the sled OFF the stand, front face on a flat
  surface, until the back edge snaps behind the side leaf-spring
  detents; to remove, push evenly from the front through the windows.
- clamp: bridges the motor top flat, CLAMP_SQUEEZE, two M2 screws into
  nuts inserted from BELOW the ears. Print SIDEWAYS - see build_clamp.
  The rear of the motor stays open (terminal PCB adapter); axial
  position comes from the base's front register wall.
- discs: two-color prints (black base + eccentric white cap, filament
  swap at z 1.4): Ø10 disc2_62 (the production-geometry ring) and Ø20
  disc2_31; blind bore, seated with the press guide.
- press: disc press guide - squares the disc to the shaft; depth stop
  = shaft tip on the blind bore floor (see build_press).
- clip: neck clip that backs the pinion during the press - without it
  the broach force slides the pinion down the shaft (see build_clip).
- shims: L-shaped height gauges under the sled wings, dropped into
  fence corrals on the base; the seat, not screw friction, carries the
  position and the upright leg stops sled pitch (notch count on the
  leg top edge = preset index).

Height presets - sled drop D moves the sensor chord below the axis by
d = 1.5 + D, phase = 2*atan(1.5/d), pattern radius r = hypot(1.5, d):
  90 deg: d 1.50, r 2.12 (Ø10 disc)   - shim 1 notch, D 0
  62 deg: d 2.49, r 2.90 (Ø10 disc)   - the REAL 7 mm PCB-disc ring
  31 deg: d 5.39, r 5.60 (Ø20 disc)
Firmware ellipse-cal absorbs non-90 phases; measured A/B phase is the
truth after any height change.

x chain, motor registered front-face against the base wall at FRONT_X
10.8: shaft tip 14.1, white cap face 14.7 (press guide puts it
PRESS_CAN_TO_CAP 3.90 from the same front-face datum), gap 2.0
(DISC_GAP), sensor faces 16.7 (0.15 recessed behind the 16.55 wall
face - each window is a light well), coupon front face 17.35. The
coupon faces -X; rear can face lands at -1.55, terminal PCB in free
air behind it.
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
# Window map along the die axis, + toward J1 (up in the stand). Vector-
# measured from the ds p3 drawing (both axes agree on scale); the
# windows are NOT symmetric. PT = pads 1/2 end = away from J1 = DOWN
# (pcb nets + step-verified board map). The LED self-apertures at 0.37,
# already narrower than any useful slit - only the PT is worth slitting.
PT_OFF = -0.43             # PT window center; inner aperture 0.60
PT_WIN = 0.60
LED_OFF = 0.47             # LED window center; inner 0.37, outer 0.47
LED_WIN = 0.37
RIB = (-0.08, 0.23)        # opaque divider, offset toward the LED
SENSE_CHORD = 3.0          # U1-U2 centers
CHORD_UP = 5.0             # sensor chord above the board's low edge
WIN_CLEAR = 0.3            # per side, sensor windows (small holes shrink)
POCKET_CLEAR = 0.12        # seat growth; pockets under-print ~0.15 (0.25
                           # drawn benched gravity-loose) -> light grip
POCKET_LIP = 0.0           # detent face growth: prints ~0.075/side bite
POCKET_MOUTH = 0.6         # funnel entry: drop-in loose
LIP_BAND = 0.4             # detent flat length along insertion
SEAT_SLACK = 0.1           # detent flat behind the board back face; the
                           # seat-side ramp toe preloads the board forward
SPRING_T = 1.0             # leaf between pocket wall and slit
SLIT_W = 0.9
SPRING_Z = (21.0, 31.0)    # slit span: fixed-fixed leaf, ~0.8% strain at
                           # full detent deflection - elastic, reusable
BUMP_Z = (23.0, 29.0)
PAD_Y = 16.6               # outboard pad backs the slit rim; starts above
PAD_Z = (21.0, 33.0)       # the M3 slot mouth (shank tops out ~20.4)

PAPER_T_EST = 0.1
AIR_GAP_EST = 1.0

AXIS_Z = 14.0
TIP_X = m.MOTOR_BODY_L + m.MOTOR_FRONT_BOSS_L + m.MOTOR_SHAFT_L  # 15.6
SENSE_X = TIP_X + PAPER_T_EST + AIR_GAP_EST                      # 16.7
BOARD_X = SENSE_X + SENSOR_H       # 17.35 coupon front face

# d = chord-to-axis offset; nominal 1.5 -> 90 deg
PRESETS = (("90", 1.5, 10.0), ("62", 2.487, 10.0), ("31", 5.39, 20.0))

# --- sled wall + wings ---
SKIN_T = 0.4               # aperture skin, 2 layers: one layer pinholes
                           # IR between extrusion lines; pocket shift
                           # tracks this so face clearance is unchanged
FACE_WALL = 0.8            # front wall = 4 clean 0.2 layers (printed
                           # flat); sensors recess 0.15 -> each window is
                           # a light well; wall face clears paper by 0.85
WALL_X = (BOARD_X - FACE_WALL, BOARD_X + BOARD_T + 1.05)  # 16.95 .. 20.0;
                           # +1.05 = lip band + funnel behind the board
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
M3_NUT_AF = 5.4            # 0.1 under the 5.5 nut: steel broaches the seat.
                           # Straight-flat pockets print ~true (5.7 benched
                           # drop-out; the 0.15-under model is a round-hole
                           # artifact), so interference is drawn, not shrunk
M3_NUT_T = 2.5             # 2.4 nut pressed to floor sits 0.1 under flush

# --- disc ---
DISC_T = 2.0               # 3d-printed
DISC_BORE = 2.2            # prints ~2.0-2.1 -> broach fit on the 2.23
                           # pinion tip circle
PATTERN_ECC = 0.5          # radial edge travel of the eccentric pattern;
                           # 0.35 benched 630/840mV pk-pk - noise-limited,
                           # so trade linearity for amplitude
WHITE_T = 0.6              # 3 layers; thinner passes 940nm through to
                           # the black base
BACK_T = 1.0               # extra full-diameter back thickness: bore
                           # grows to 2.4 of the 2.65 shaft (1.4 alone
                           # = hand-rockable wobble on the tooth tips).
                           # Full OD = zero bridging; 1.0 not 1.3 so
                           # the disc clears the register wall rib

# --- motor saddle + clamp (M2 screws, nuts from below) ---
SADDLE_X = (1.9, 10.2)     # ~70% of the 12.3 can, slide room both ways
SADDLE_CLEAR = 0.25        # radial, sides only - floor is exact (z)
EAR_TOP = 12.0
EAR_Y = (5.6, 12.4)        # wide enough for the nut chimney
SCREW_X = (SADDLE_X[0] + SADDLE_X[1]) / 2
SCREW_Y = 9.0              # ear center
M2_HOLE = 2.4
M2_NUT_AF = 3.9            # 0.1 under the 4.0 nut (4.1 benched drop-out):
                           # the screw draws it in and it broaches; must
                           # resist screw-start push or it drops down the
                           # chimney
M2_NUT_T = 2.1             # 1.6 nut + 0.5 cone -> full-flat engagement
M2_NUT_Z = 8.4             # hex pocket floor
CHIMNEY_D = 5.2            # prints ~5.0; nut corner circle 4.62
CLAMP_Z = (12.3, 19.6)     # bottom floats 0.3 over the ears = travel
CLAMP_SQUEEZE = 0.2        # interference at the motor top flat (0.1
                           # benched roll-slip with the clamp tight)
DISC_GAP = 2.0             # white cap face -> sensor face; geometric
                           # via the base's front register wall. 1mm
                           # dead-zones the ecc-0.5 disc (benched H2
                           # 34-61%); ~5mm is clean but SNR-starved at
                           # 4k7 - 2.0 splits with margin for the
                           # 0.05-0.20 shaft end play
PRESS_CAN_TO_CAP = (TIP_X - m.MOTOR_BODY_L) + WHITE_T  # 3.90: front can
                           # face -> white cap face when the pinion tip
                           # bottoms on the disc's blind bore floor -
                           # the press stop. The floor is table-backed
                           # (pure compression, no doming)
FRONT_X = SENSE_X - DISC_GAP - PRESS_CAN_TO_CAP  # front can face
                           # against the base register wall; rear stays
                           # open (terminal PCB adapter lives there)
CLIP_T = 0.45              # < the 0.53 boss-to-pinion gap it enters;
                           # the press jams on it CLIP_T - 0.30 = 0.15
                           # shy of nominal depth (disc back to boss is
                           # 0.30 at full depth) - accept the 2.15 gap
                           # or pry the clip and push the last 0.15
CLIP_OD = 6.5              # 1.25 rim beyond the Ø4 boss = tweezer grab
CLIP_BORE = 1.1            # loose on the 1.0 shaft; the 0.85 mouth
CLIP_MOUTH = 0.85          # (prints tighter) snaps over it and retains
REG_WALL_T = 1.25

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

    p = Pos(9.0, 0, -BASE_T) * extrude(RectangleRounded(44.0, 46.0, 4.0),
                                       BASE_T)

    # motor saddle: bore floor exact at the bottom flat; sides +0.25
    # except two 1.2-wide stations bored at +0.06 - the tight bands
    # re-register the can in yaw (0.25 benched ~3 deg axis swing in
    # plan) while the motor still slides axially through them. Cut
    # segment-wise so the bands are part of the trough wall, not
    # added tabs (printable: no mid-air geometry)
    p += _box(SADDLE_X[0], FRONT_X, -6.5, 6.5, 0, 12.0)
    segs = ((SADDLE_X[0] - 0.1, 2.4, SADDLE_CLEAR),
            (2.4, 3.6, 0.06),
            (3.6, 8.6, SADDLE_CLEAR),
            (8.6, 9.8, 0.06),
            (9.8, FRONT_X + 0.01, SADDLE_CLEAR))
    for x0, x1, clear in segs:
        prof = Circle(m.MOTOR_BODY_D / 2 + clear) & Pos(0, 25 - flat_half) \
            * Rectangle(40, 50)
        p -= Pos(x0, 0, AXIS_Z) * extrude(Plane.YZ * prof, x1 - x0)

    # front register wall: the can front face bears on its back face -
    # slide the motor forward to the stop, tighten the clamp, and the
    # disc-to-sensor gap is DISC_GAP by construction (press guide sets
    # the disc at PRESS_CAN_TO_CAP from the same datum). Wall top is 2
    # below the axis, so the boss and shaft pass over it and the motor
    # still drops straight in; contact = the can face's lower flank.
    # Stepped profile: full thickness below the disc envelope, a 0.6
    # rib beside it (thick-disc back face spins 0.3 off the rib)
    p += _box(FRONT_X, FRONT_X + 0.6, -6.5, 6.5, 0, 12.0)
    p += _box(FRONT_X, FRONT_X + REG_WALL_T, -6.5, 6.5, 0, 3.7)

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

    # rear fin: M3 through-holes, hex pockets sunk from the back face;
    # pocket corner reaches y 19.2 - keep >=2 wall outboard or the
    # press splits the layers (19.5 edge benched crack)
    p += _box(FIN_X[0], FIN_X[1], -21.5, 21.5, 0, WING_Z[1])
    for s in (1, -1):
        p -= Pos(FIN_X[0] - 0.1, s * M3_SCREW_Y, M3_SCREW_Z) * Rot(0, 90, 0) \
            * Cylinder(M3_HOLE / 2, FIN_X[1] - FIN_X[0] + 0.2,
                       align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= Pos(FIN_X[1] + 0.1, s * M3_SCREW_Y, M3_SCREW_Z) * Rot(0, -90, 0) \
            * extrude(RegularPolygon(M3_NUT_AF / math.sqrt(3), 6),
                      M3_NUT_T + 0.1)

    # shim fence: three 1.5-tall ridges + the fin = a corral per side;
    # the L-shims drop in with 0.2 play and stand captive. 1.5 clears
    # the sled at its deepest drop (wall bottom 2.11 at preset 31)
    for s in (1, -1):
        p += _box(14.65, 15.35, s * 12.2, s * 20.4, 0, 1.5)
        p += _box(14.65, 21.35, s * 12.2, s * 12.9, 0, 1.5)
        p += _box(14.65, 21.35, s * 19.7, s * 20.4, 0, 1.5)

    p.label = "encbench_base"
    p.color = Color(0.35, 0.38, 0.42)
    return p


def build_sled(outline: Face, slit: float | None = None) -> Part:
    """Wall + wings at the NOMINAL (90 deg, topmost) position. One flat
    part: full height over the board span, wing-height extensions
    outboard (so the slots stay open at the top for drop-on).

    slit: aperture variant - a SKIN_T front skin spans each window,
    slitting ONLY the PT (a slit of this height at chord_z + PT_OFF);
    the LED gets a full clearance opening, since its own 0.37 aperture
    is narrower than any useful slit. Sensing region = overlap of the
    LED patch and the choked PT view: narrows the spot radially (the
    modulation direction). Board pocket shifts back SKIN_T so the
    coupon seats on the pocket floor, not the skin. Costs some light -
    re-pick the load R after. slit < PT_WIN (0.60) to actually choke."""
    shift = SKIN_T if slit is not None else 0.0
    wx1 = WALL_X[1] + shift
    loc = Pos(shift, 0, 0) * BOARD_LOC
    board_x = BOARD_X + shift

    p = _box(WALL_X[0], wx1, -15.0, 15.0, WALL_Z[0], WALL_Z[1])
    for s in (1, -1):
        p += _box(WALL_X[0], wx1, s * 15.0, s * WALL_W / 2,
                  WING_Z[0], WING_Z[1])

    # pocket = projected pcb outline: funnel mouth over a plain seat;
    # retention comes from the side leaf-spring detents, not the ring
    seat_f = loc * offset(outline, POCKET_CLEAR / 2)
    sgn = 1.0
    if extrude(seat_f, 0.2).bounding_box().min.X < board_x - 0.05:
        sgn = -1.0
    fun_x = BOARD_T + SEAT_SLACK + LIP_BAND
    fun_f = Pos(fun_x, 0, 0) * loc * offset(outline, POCKET_CLEAR / 2)
    mouth_f = Pos(wx1 - board_x + 0.1, 0, 0) * loc \
        * offset(outline, POCKET_MOUTH / 2)
    pocket = extrude(seat_f, sgn * (fun_x + 0.01))
    pocket += loft([fun_f, mouth_f])
    p -= pocket

    # front-wall openings: sensor windows + J1 tail relief; slit
    # variant keeps a SKIN_T skin at the front face and cuts the slit
    # through it (front face stays planar - prints face-down as before)
    chord_z = AXIS_Z - 1.5
    wy = SENSOR_BODY[0] + 2 * WIN_CLEAR
    wz = SENSOR_BODY[1] + 2 * WIN_CLEAR
    win_x0 = WALL_X[0] + shift if slit is not None else WALL_X[0] - 0.1
    for s in (1, -1):
        yc = s * SENSE_CHORD / 2
        p -= _box(win_x0, board_x + 0.05, yc - wy / 2, yc + wy / 2,
                  chord_z - wz / 2, chord_z + wz / 2)
        if slit is not None:
            zc = chord_z + PT_OFF
            p -= _box(WALL_X[0] - 0.1, WALL_X[0] + SKIN_T + 0.1,
                      yc - wy / 2, yc + wy / 2,
                      zc - slit / 2, zc + slit / 2)
            zl = chord_z + LED_OFF
            p -= _box(WALL_X[0] - 0.1, WALL_X[0] + SKIN_T + 0.1,
                      yc - wy / 2, yc + wy / 2, zl - 0.3, zl + 0.3)
            # crosstalk rib: lands on the package's center divider
            # (which is offset toward the LED) and walls off the skin-
            # to-package gap - without it LED light waveguides across
            # the 0.15 gap into the PT window (benched: DC up,
            # modulation -4x on the ribless 1-layer skin)
            p += _box(WALL_X[0] + SKIN_T, WALL_X[0] + SKIN_T + 0.15,
                      yc - wy / 2, yc + wy / 2,
                      chord_z + RIB[0], chord_z + RIB[1])
    # J1 pin tails protrude 1.4 past the sensor face (step-verified
    # -3.8..4.4 x 30.2..30.8 in stand yz)
    p -= _box(WALL_X[0] - 0.1, board_x + 0.05, -4.7, 5.3, 29.5, 31.5)

    # wings: doubler pads fully backed by the widened wall (flat print);
    # open-top slots cut through the full sled - heads bear on the front
    for s in (1, -1):
        p += _box(wx1, FIN_X[0], s * WING_Y[0], s * WING_Y[1],
                  WING_Z[0], WING_Z[1])
        p -= _box(WALL_X[0] - 0.1, FIN_X[0] + 0.1,
                  s * M3_SCREW_Y - M3_SLOT_W / 2,
                  s * M3_SCREW_Y + M3_SLOT_W / 2,
                  M3_SCREW_Z - M3_SLOT_W / 2, WING_Z[1] + 0.1)

    # side leaf springs: a slit isolates a fixed-fixed strip beside the
    # pocket; detent bumps on it snap over the board back edge, ramped
    # both ways so removal flexes the leaf instead of shearing it
    wall_y = BOARD_W / 2 + POCKET_CLEAR / 2
    bx = board_x + BOARD_T
    for s in (1, -1):
        p += _box(WALL_X[0], wx1, s * 14.9, s * PAD_Y,
                  PAD_Z[0], PAD_Z[1])
        p -= _box(WALL_X[0] - 0.1, wx1 + 0.1,
                  s * (wall_y + SPRING_T),
                  s * (wall_y + SPRING_T + SLIT_W),
                  SPRING_Z[0], SPRING_Z[1])
        yd = s * (BOARD_W / 2 + POCKET_LIP / 2)
        yw = s * (wall_y + 0.14)
        bump = Polygon((bx - 0.2, s * wall_y), (bx + SEAT_SLACK, yd),
                       (bx + SEAT_SLACK + LIP_BAND, yd),
                       (bx + SEAT_SLACK + LIP_BAND + 0.3, s * wall_y),
                       (bx + SEAT_SLACK + LIP_BAND + 0.3, yw),
                       (bx - 0.2, yw), align=None)
        p += Pos(0, 0, BUMP_Z[0]) * extrude(bump, BUMP_Z[1] - BUMP_Z[0])

    p.label = "encbench_sled" if slit is None else \
        f"encbench_sled_slit{round(slit * 10):02d}"
    p.color = Color(0.08, 0.08, 0.08)
    return p


def build_clamp() -> Part:
    """Bridges the motor top flat, CLAMP_SQUEEZE interference, two M2
    screws into nuts inserted from BELOW the ears. Axial position comes
    from the base's front register wall, so the rear stays fully open
    for the terminal PCB adapter. Print SIDEWAYS (on an ear end): the
    clamped motor pushes the bridge up, and flat/upside-down print
    orientations put that tension across the layer lines - the bridge
    root delaminates. Sideways aligns the layers with the bending."""
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


def build_disc2(dia: float, r_sense: float, name: str,
                ecc: float = PATTERN_ECC) -> Part:
    """Two-color pattern disc: black base, eccentric white cap = the
    pattern in material instead of ink. Print face up, filament swap
    at z = BACK_T + DISC_T - WHITE_T. White top sits at
    the old paper plane; the black field prints WHITE_T recessed
    (farther = darker, contrast stacks). Iron/smooth the white, leave
    the black matte. Bore is blind (unbroken white face), runs the
    2.4 of the 2.65 shaft. Seat with the press guide. Press-fit
    only - discs stay swappable for experiments."""
    p = Pos(0, 0, -BACK_T) * extrude(Circle(dia / 2),
                                     BACK_T + DISC_T - WHITE_T)
    p += Pos(ecc, 0, DISC_T - WHITE_T) * extrude(
        Circle(r_sense), WHITE_T)
    p -= Pos(0, 0, -BACK_T - 0.1) * Cylinder(
        DISC_BORE / 2, BACK_T + DISC_T - WHITE_T + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(0, 0, -BACK_T) * Cone(
        DISC_BORE / 2 + 0.35, DISC_BORE / 2, 0.4,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = f"encbench_disc2_{name}"
    p.color = Color(0.15, 0.15, 0.15)
    return p


BAND_W0 = 1.2              # band mean width; min w0-w1 = 0.5 = two clean
BAND_W1 = 0.7              # 0.25 perimeters, max 1.9 stays inside the spot


def build_disc3(dia: float, r_sense: float, name: str, cycles: int = 1,
                w0: float = BAND_W0, w1: float = BAND_W1,
                coarse: float = 0.0) -> Part:
    """Width-modulated ring disc: white band at the sensor radius,
    width w0 + w1*cos(cycles*th) (+ coarse*cos(th) for N>1 sector
    disambiguation). The spot integrates area, and a moving average
    of a sine is a sine - the signal is harmonic-free at ANY gap /
    spot shape, unlike disc2's eccentric edge (harmonics by
    construction, needs the right blur). Radial runout moves the band
    inside the spot without changing coverage - press eccentricity
    drops out to first order. Requirements: spot covers the full band
    width; spot arc < ~1/3 wavelength or the average eats the
    fundamental (Ø20 ring, N=9: wavelength 3.9). Channel phase =
    cycles x sensor separation - shim the height so it lands 90.
    Slicer: aligned seam, so the zit sits at one fixed angle the cal
    absorbs. Same base/bore/press as disc2 EXCEPT the bore is
    through: the band does not cover the center and a white
    press-floor plug would put a static (and, press-eccentric, 1/rev)
    reflector at the spot's inner tail - the clip stop is the only
    press stop. Never press past it: with no floor the next stop is
    the shaft tip finding the table through the bore."""
    assert w0 - w1 - abs(coarse) >= 0.5, "band pinches below 2 perimeters"
    p = Pos(0, 0, -BACK_T) * extrude(Circle(dia / 2),
                                     BACK_T + DISC_T - WHITE_T)
    steps = 720
    outer, inner = [], []
    for i in range(steps):
        th = 2 * math.pi * i / steps
        w = w0 + w1 * math.cos(cycles * th) + coarse * math.cos(th)
        outer.append(((r_sense + w / 2) * math.cos(th),
                      (r_sense + w / 2) * math.sin(th)))
        inner.append(((r_sense - w / 2) * math.cos(th),
                      (r_sense - w / 2) * math.sin(th)))
    band = Polygon(*outer, align=None) - Polygon(*inner, align=None)
    p += Pos(0, 0, DISC_T - WHITE_T) * extrude(band, WHITE_T)
    p -= Pos(0, 0, -BACK_T - 0.1) * Cylinder(
        DISC_BORE / 2, BACK_T + DISC_T - WHITE_T + 0.1,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(0, 0, -BACK_T) * Cone(
        DISC_BORE / 2 + 0.35, DISC_BORE / 2, 0.4,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.label = f"encbench_disc3_{name}"
    p.color = Color(0.15, 0.15, 0.15)
    return p


def build_press(dia: float, name: str) -> Part:
    """Disc press guide - depth and squareness come from the tool, not
    the thumb (hand-pressing benched visible disc wobble). Disc lies
    white-face-down on a flat table, tool over it, motor into the top
    bore. Snap the neck clip on first (build_clip) - the bore
    broaches onto the pinion at more force than the pinion's own fit
    on the shaft, so an unbacked press slides the pinion into the can
    (benched: 1mm slide, grinding); the clip backs it against the
    boss. Press until it stops firm - that is the disc back face
    pinching the clip against the boss, 0.15 shy of nominal depth
    (see CLIP_T). Either accept that gap or pry the clip off and push
    the last 0.15 until the shaft tip bottoms on the disc's blind
    bore floor; the floor is table-backed - pure compression through
    the white column, no doming - so bottoming IS the exact depth
    stop: PRESS_CAN_TO_CAP by construction. The long bore keeps the
    shaft square; the dia+0.4 pocket
    pre-centers the disc to +/-0.2 so the disc's entry cone (catch
    +/-0.35) finishes the job - one press per disc diameter. No
    internal shoulder: any shoulder that can stop the can also traps
    the pressed assembly (disc below it, can above, neither passes) -
    the straight bore lets motor + disc withdraw together. press10
    prints either way up; press20 upside down or its wide pocket roof
    is a 5mm annular bridge."""
    p = extrude(Circle((dia + 7.0) / 2), 16.0)
    p -= Pos(0, 0, 3.15) * Cylinder(
        10.3 / 2, 16.0 - 3.15 + 0.1,        # can guide: Ø10 round sides,
        align=(Align.CENTER, Align.CENTER, Align.MIN))  # flats never touch
    p -= Pos(0, 0, -0.1) * Cylinder(
        (dia + 0.4) / 2, 3.25,              # disc pocket: pre-centers,
        align=(Align.CENTER, Align.CENTER, Align.MIN))  # roof 3.15 clears t=3.0
    p.label = f"encbench_press{name}"
    p.color = Color(0.35, 0.38, 0.42)
    return p


def build_clip() -> Part:
    """Neck clip: C-washer snapped around the shaft between the
    motor's Ø4 front boss and the pinion back face for the press.
    Sandwiched, the pinion clamps it against the boss and the press
    force loops disc -> pinion -> clip -> boss -> can -> hand; the
    pinion gives up only the clamp slack (<=0.53 - CLIP_T, stays
    clear of the boss). The press jams on it just shy of nominal
    depth - see build_press for the endgame. Grab the rim past the
    boss to pry it off. The flared mouth pushes on over the shaft.
    Prints flat; expendable - print a few."""
    p = extrude(Circle(CLIP_OD / 2), CLIP_T)
    p -= Pos(0, 0, -0.1) * Cylinder(
        CLIP_BORE / 2, CLIP_T + 0.2,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    hm = CLIP_MOUTH / 2
    p -= Pos(0, 0, -0.1) * extrude(Polygon(  # ccw or extrude goes -z
        (0, -hm), (2.2, -hm), (CLIP_OD / 2 + 0.1, -0.9),
        (CLIP_OD / 2 + 0.1, 0.9), (2.2, hm), (0, hm),
        align=None), CLIP_T + 0.2)
    p.label = "encbench_clip"
    p.color = Color(0.85, 0.83, 0.78)
    return p


def build_shim(drop: float, notches: int) -> Part:
    """L-shim under a sled wing: the seat sets height = WING_Z[0] - drop
    (the shim, not screw friction, carries the position); the upright
    leg stands 0.2 in front of the sled face and stops it pitching on
    loose screws. Drops into the base fence. Notches on the leg's top
    edge = preset index, centered so the part equals its own mirror -
    print two, either side.

    The 0.8 rear blade alone snaps in handling; the forward doubler
    triples the section. Its underside slopes (apex at the rear face)
    so it prints upright without a flat overhang and rides over the
    fence cross-ridge (top 1.5) with 0.2 to spare."""
    h = WING_Z[0] - drop
    p = _box(15.55, 21.1, 13.1, 19.5, 0, h)
    p += _box(15.55, 16.35, 13.1, 19.5, 0, h + 5.0)
    prof = Polygon((14.55, 3.0), (14.55, h + 5.0), (15.55, h + 5.0),
                   (15.55, 1.7), align=None)
    p += Pos(0, 13.1, 0) * extrude(Plane.XZ * prof, 6.4)
    y0 = 16.3 - 0.8 * (notches - 1)
    for i in range(notches):
        p -= Pos(15.45, y0 + 1.6 * i, h + 5.0) * Box(2.2, 0.8, 1.2)
    p.label = f"encbench_shim_d{drop:.1f}"
    p.color = Color(0.85, 0.83, 0.78)
    return p


if __name__ == "__main__":
    out = pathlib.Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    outline = board_outline_face()
    parts = [(build_base(), "encbench_base"),
             (build_sled(outline), "encbench_sled"),
             (build_sled(outline, slit=0.3), "encbench_sled_slit03"),
             (build_sled(outline, slit=0.4), "encbench_sled_slit04"),
             (build_clamp(), "encbench_clamp"),
             (build_press(10.0, "10"), "encbench_press10"),
             (build_press(20.0, "20"), "encbench_press20"),
             (build_clip(), "encbench_clip")]
    for i, (name, d, dia) in enumerate(PRESETS):
        parts.append((build_shim(d - 1.5, i + 1), f"encbench_shim{name}"))
        if name != "90":  # 90 preset retired: firmware measures phase
            r = math.hypot(1.5, d)
            parts.append((build_disc2(dia, r, name), f"encbench_disc2_{name}"))
    # small-ecc variant for the registered 1.0 gap: the tighter spot
    # there flat-tops the 0.5 pattern (benched: H2 34% on B at ~1mm)
    parts.append((build_disc2(10.0, math.hypot(1.5, 2.487), "62e35",
                              ecc=0.35), "encbench_disc2_62e35"))
    # width-modulated rings: n1 = pure-sine drop-ins (production Ø10 +
    # bench Ø20), n9 = 9-lobe fine track (error / 9; ~80 deg native
    # phase at the 31 preset, shim toward 90)
    parts.append((build_disc3(10.0, math.hypot(1.5, 2.487), "62n1"),
                  "encbench_disc3_62n1"))
    parts.append((build_disc3(20.0, math.hypot(1.5, 5.39), "31n1"),
                  "encbench_disc3_31n1"))
    parts.append((build_disc3(20.0, math.hypot(1.5, 5.39), "31n9",
                              cycles=9), "encbench_disc3_31n9"))
    from blueprint import blueprint
    for part, name in parts:
        export_step(part, str(out / f"{name}.step"))
        export_stl(part, str(out / f"{name}.stl"), tolerance=0.01)
        blueprint(part, str(out / f"{name}_bp.svg"), name=name)
        print(f"{name}: volume {part.volume:.0f} mm^3 -> {out}")

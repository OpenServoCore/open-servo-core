"""Measured SG90 dimensions - the datasheet for parts that have none.

Donor: Miuzei-brand SG90 clone (Amazon B072V529YD), NOT TowerPro. Clone
families differ (gears, motor, pot, case); a brand switch re-opens every
[cal] entry.

Provenance tags: [cal] donor calipers, [dwg] NFP FF-M20SA outline drawing,
[lst] Amazon listing spec, [der] derived from other entries. Unverified
guesses are named *_EST and must be replaced by measurement before a part
is trusted.
"""

# --- M10 motor (flat can; donor = short variant of the FF-M20SA dwg,
# sold as "M10 motor": 8x10 can, dual-rated 3V/18k + 5V/30k, Kv 6000) ---
MOTOR_BODY_L = 12.3        # [cal] donor short variant (15mm-type dwg: 15.0)
MOTOR_BODY_D = 10.0        # [dwg], [cal] "two circular tip" 10.0
MOTOR_BODY_FLAT = 8.0      # [dwg], [cal] across flats
MOTOR_FRONT_PLATE_D = 7.0  # [dwg] 7.0 +/-0.1
MOTOR_FRONT_PLATE_L = 0.0  # donor has no plate stage (dwg 15mm-type: 0.6)
MOTOR_FRONT_BOSS_D = 4.0   # [dwg]
MOTOR_FRONT_BOSS_L = 0.6   # [cal] donor (dwg 15mm-type: 1.4 + plate)
MOTOR_SHAFT_D = 1.0        # [dwg]
MOTOR_SHAFT_L = 2.7        # [der] 16.6 total - 13.9 boss-to-boss
MOTOR_REAR_BOSS_D = 3.96   # [cal] (dwg says 4.0)
MOTOR_REAR_BOSS_L = 1.0    # [cal] donor: 0.6 brass + 0.4 plastic collar
MOTOR_TERM_SPAN = 7.3      # [dwg] c-c along the round axis
MOTOR_TERM_W = 1.5         # [dwg]
MOTOR_TERM_T = 0.3         # [dwg]
MOTOR_TERM_L = 1.0         # [der] 13.6 incl solder - 12.3 can = 1.3;
                           # bare tab visually flush with the 1.0 rear
                           # boss, ~0.3 is solder on the donor's tabs
MOTOR_TERM_HOLE = 0.5      # [dwg]
MOTOR_END_PLAY = (0.05, 0.20)  # [dwg] shaft end play spec

# Miuzei donor motor is a SHORTER variant than the NFP 15mm-type drawing.
# Raw caliper spans (definitions being confirmed; motor.py still builds
# the 15mm drawing geometry until these resolve):
MOTOR_RAW_HEIGHT = 12.32       # [cal] "motor height" - span definition TBC
MOTOR_RAW_SHAFT_TO_BOSS = 21.23  # [cal] shaft tip to rear boss tip
MOTOR_RAW_BOSS_TO_BOSS = 14.00   # [cal] front boss tip to rear boss tip
MOTOR_RAW_FBOSS_TO_REAR = 12.91  # [cal] front boss tip to rear can face
MOTOR_RAW_FRONT_TO_BOSS = 13.90  # [cal] front can face to rear boss tip
MOTOR_RAW_FRONT_TO_TERM = 13.6   # [cal] front face to terminal tip,
                                 # donor tabs carry solder (13.55 earlier)
# RESOLVED: front boss 0.6, can 12.3, rear boss 1.0 (0.6 brass +
# 0.4 plastic), total 13.9; total incl shaft 16.6 -> stickout 2.7.
# The 21.23 span reading was an outlier. MOTOR_* above hold the donor
# values; the NFP 15mm-type originals noted in comments.
MOTOR_TOTAL_L = 16.6           # [cal] shaft tip to rear boss tip
# motor-mount PCB conclusion: 1.0mm board -> rear face sits on the board,
# boss (1.0) flush with the bottom face, tabs (1.0) flush in their slots

# --- feedback pot ---
POT_BODY_D = 9.92          # [cal]
POT_BODY_L = 4.30          # [cal]
POT_SHAFT_D = 1.35         # [cal] TIP round section (shaft is stepped)
POT_RIVET_D = 2.6          # [cal] flare on back face
POT_LEG_R = 3.5            # [cal] tab holes, all three
POT_LEG_ANGLE = 48.0       # [cal] outer legs +/-48 deg from wiper leg
POT_LEG_L = 4.78           # [cal]
POT_TAB_HOLE = 0.5         # [cal]
POT_TAB_W = 1.5            # [cal] photo-estimated, tab width
POT_TAB_T = 0.3            # photo-estimated
POT_FRONT_SHAFT_L_EST = 2.5  # UNVERIFIED - shaft stub above front face
# stock-board leg forming: outer holes 6.0 c-c, wiper 3.33 from each
# (1.45 behind their line); all on r~4.1 about the axis [cal+der]
POT_FORMED_LEG_R = 4.1
POT_WALL_EST = 0.8           # UNVERIFIED - cup side wall thickness
POT_FLOOR_T_EST = 1.0        # UNVERIFIED - cup back floor thickness
POT_RECESS_D_EST = 4.6       # UNVERIFIED - back-face recess dia around rivet
POT_RECESS_DEPTH_EST = 0.4   # UNVERIFIED - back-face recess depth
POT_MOLD_HOLE_D_EST = 1.4    # UNVERIFIED - back-face mold hole dia
POT_MOLD_HOLE_DEPTH_EST = 0.9   # UNVERIFIED - mold hole depth
POT_TAB_L_EST = 2.2          # UNVERIFIED - terminal tab radial length
POT_LEG_W_EST = 0.8          # UNVERIFIED - leg width below the tab
POT_TRACK_OD_EST = 8.2       # UNVERIFIED - resistive track outer dia
POT_TRACK_ID_EST = 4.6       # UNVERIFIED - resistive track inner dia
POT_TRACK_T_EST = 0.1        # UNVERIFIED - track layer thickness
POT_BUSH_D_EST = 3.0         # UNVERIFIED - brass center bushing dia
POT_BUSH_H_EST = 1.3         # UNVERIFIED - bushing height above cup floor
POT_BUSH_FLANGE_D_EST = 4.4  # UNVERIFIED - bushing base flange dia (wiper return)
POT_ROTOR_D_EST = 6.8        # UNVERIFIED - rotor disc dia
POT_ROTOR_H_EST = 2.5        # UNVERIFIED - rotor total height, disc + hub
POT_ROTOR_HUB_D_EST = 3.6    # UNVERIFIED - rotor front hub dia
POT_WASHER_OD_EST = 4.6      # UNVERIFIED - wave washer OD
POT_WASHER_ID_EST = 2.0      # UNVERIFIED - wave washer ID
POT_WASHER_T_EST = 0.15      # UNVERIFIED - wave washer stock thickness
POT_WASHER_WAVE_EST = 0.25   # UNVERIFIED - wave washer wave amplitude
POT_SHAFT_SLOT_W_EST = 0.5   # UNVERIFIED - shaft tip screwdriver slot width
POT_SHAFT_SLOT_DEPTH_EST = 0.7  # UNVERIFIED - shaft tip slot depth

# --- gear train (heights are part heights, not positions) ---
PINION_D = 2.23            # [cal] incl teeth, brass, press-fit
PINION_H = 2.17            # [cal]
GEAR1_D = 9.52             # [cal] big wheel
GEAR1_H = 1.21             # [cal] big wheel thickness
GEAR1_PINION_D = 2.92      # [cal] small stage
GEAR1_TOTAL_H = 3.01       # [cal]
GEAR2_D = 9.66             # [cal] (pot shaft)
GEAR2_H = 1.40             # [cal]
GEAR2_PINION_D = 2.98      # [cal]
GEAR2_TOTAL_H = 4.11       # [cal]
GEAR3_D = 9.80             # [cal] (central shaft, atop gear1)
GEAR3_H = 2.00             # [cal]
GEAR3_PINION_D = 3.55      # [cal]
GEAR3_TOTAL_H = 5.98       # [cal]
GEAR4_D = 9.78             # [cal] output gear (pot shaft, atop gear2)
GEAR4_H = 3.05             # [cal]
GEAR4_BOSS_D = 4.99        # [cal] center boss = horn spline OD, incl teeth
GEAR4_TOTAL_H = 8.11       # [cal] excl 180-deg stop tab
SPLINE_T = 21              # [lst] SG90-class standard; not yet counted on donor
# tooth counts [cal, counted]
PINION_T = 10
GEAR1_T = 47
GEAR1_PINION_T = 10
GEAR2_T = 38
GEAR2_PINION_T = 8   # FIELD FAILURE: strips 1-3 teeth under STALL
                     # (3/3 failed donors, same stage); std root would
                     # sit inside the 1.9 bore -> shallow teeth on a
                     # ~0.12mm wall
GEAR3_T = 32
GEAR3_PINION_T = 7
GEAR4_T = 23
GEAR_RATIO = (47 * 38 * 32 * 23) / (10 * 10 * 8 * 7)  # 234.73:1 = 41078/175

# design intent [der]: meshes 2/3/4 share the pot-arbor axis pair, and
# standard modules 0.25/0.30/0.40 all give exactly 6.00 c-c; mesh 1 at
# module 0.20 gives 5.70 (measured 5.65). Measured ODs read under the
# nominal od = m*(N+2): tip shortening on low-count pinions + caliper
# squeeze on plastic. Use modules + centers as authoritative.
MESH_MODULES = (0.20, 0.25, 0.30, 0.40)
AXIS_ARBOR_MOTOR_DESIGN = 5.70
AXIS_POT_ARBOR_DESIGN = 6.00  # re-measured 5.865, within jaw-squeeze of
                              # design; three standard-module meshes pin 6.00

# --- case (bottom shell) ---
CASE_OUT_L = 23.0          # [cal] bottom case outer
CASE_OUT_W = 12.0          # [cal]
CASE_WALL = 1.0            # [cal]
CASE_SCREW_DIAG = 3.5      # [cal] screw column face from outer corner tip
CASE_DECK_T = 1.0          # [cal] gear-chamber floor
CASE_DECK_HOLE_D = 4.0     # [cal] pinion/boss hole = motor locating fit
CASE_MOTOR_POCKET_H = 6.0  # [cal] motor compartment wall depth below deck
GEAR_FLOOR_CLEAR = 0.1     # [cal] pinion and gear1 ride this above the deck

# --- case (whole) ---
CASE_TOTAL_H = 22.447      # [cal] excl output boss
CASE_TOTAL_H_BOSS = 26.85  # [cal] incl output boss
CASE_BOSS_H = 4.40         # [der] 26.85 - 22.447
CASE_TOP_H = 6.57          # [cal] top section, excl boss
CASE_MID_H = 10.30         # [cal]
CASE_BOT_H = 5.39          # [cal] sections sum 22.26 vs 22.447 total:
                           # 0.19 lives in the joint lips, resolve in case.py
EAR_SPAN = 32.12           # [cal] across both mounting ears
EAR_T = 2.47               # [cal] ear thickness
EAR_HOLE_INNER_SPAN = 26.0 # [cal] hole inner-edge to inner-edge
EAR_HOLE_D = 2.5           # [cal]
EAR_HOLE_CC = 28.5         # [der] 26.0 + 2.5
# wire exit slot: pot-end wall (opposite the motor); donor slot was
# enlarged with a soldering iron - dims unknown, model nominal
SCREW_L = 20.32            # [cal] case screw total
SCREW_THREAD_L = 5.25      # [cal] threaded portion
SCREW_SHANK_D = 0.77       # [cal] shank; thread OD not yet measured
SCREW_CC_L = 19.69         # [der] counterbore outer edges 17.99/21.39 mean
SCREW_CC_W = 8.80          # [der] 7.03/10.57 mean

# --- case estimates (case.py) ---
CASE_CORNER_R_EST = 0.5      # UNVERIFIED - outer shell corner round
CASE_BOSS_D_EST = 11.5       # UNVERIFIED - output boss dia; reference STEP
                             # reads 12.0, truncated flush at the case end
CASE_BOSS_HOLE_D_EST = 6.0   # UNVERIFIED - output opening through the boss
EAR_W_EST = 12.0             # UNVERIFIED - ear plan width = case width
                             # (reference STEP agrees)
CASE_WIRE_SLOT_W_EST = 4.0   # UNVERIFIED - donor slot melted, nominal
CASE_WIRE_SLOT_H_EST = 2.5   # UNVERIFIED - open to the middle bottom edge
CASE_SCREW_PILOT_D_EST = 1.0    # UNVERIFIED - screw column pilot bore
CASE_SCREW_CBORE_D_EST = 1.7    # UNVERIFIED - (21.39 - 17.99) / 2 edge spans
CASE_SCREW_CBORE_DEPTH_EST = 1.2  # UNVERIFIED - head recess depth
CASE_LIP_H_EST = 1.0         # UNVERIFIED - joint lip engagement into caps
CASE_LIP_T_EST = 0.6         # UNVERIFIED - joint lip wall thickness
CASE_LIP_CLEAR_EST = 0.05    # UNVERIFIED - lip-to-cap fit per side
CASE_POT_POCKET_D_EST = 10.1    # UNVERIFIED - pot bore, POT_BODY_D + fit
CASE_POT_SEAT_ID_EST = 8.6   # UNVERIFIED - pot seat ring opening (legs pass)
CASE_POT_SEAT_T_EST = 0.8    # UNVERIFIED - pot seat ring thickness
CASE_MOTOR_CLEAR_EST = 0.15  # UNVERIFIED - motor pocket clearance per side
CASE_ARBOR_BOSS_D_EST = 3.0  # UNVERIFIED - deck boss, arbor lower end
CASE_ARBOR_BOSS_H_EST = 0.8  # UNVERIFIED - deck boss height above deck top
CASE_ARBOR_HOLE_D_EST = 1.15    # UNVERIFIED - arbor press bore (ARBOR_D 1.20)
CASE_TOP_ARBOR_BOSS_D_EST = 4.0   # UNVERIFIED - ceiling boss, arbor tip
CASE_TOP_ARBOR_BOSS_H_EST = 1.5   # UNVERIFIED - drop below ceiling
CASE_TOP_ARBOR_HOLE_D_EST = 1.3   # UNVERIFIED - arbor tip clearance bore
CASE_OUT_GUIDE_D_EST = 8.5   # UNVERIFIED - ceiling ring around output opening
CASE_OUT_GUIDE_H_EST = 1.0   # UNVERIFIED - drop below ceiling
CASE_BOSS_CAVITY_D_EST = 10.4    # UNVERIFIED - boss bore, open to the
                                 # ceiling plane; gear4 9.78 + clear
                                 # (reference STEP: 11.0 in a 12.0 boss)
CASE_BOSS_CAP_T_EST = 0.75       # UNVERIFIED - boss/lobe end-cap left above
                                 # the cavity (reference STEP reads 0.75)
CASE_ARBOR_LOBE_D_EST = 5.2      # UNVERIFIED - arbor-side boss lobe OD
                                 # (reference STEP: 5.0 OD / 4.0 bore at
                                 # 6.0 from the output axis, full height)
CASE_ARBOR_LOBE_HOLE_D_EST = 4.2 # UNVERIFIED - lobe cavity, gear3 pinion
                                 # 3.55 + clear

# --- shaft axes (spans measured jaw-outside-both-rods) ---
ARBOR_D = 1.20             # [cal] central steel arbor
AXIS_POT_ARBOR = 5.865     # [der] re-measured span 7.39 - (1.85 + 1.20)/2
                           # (first span reading 7.13 discarded)
AXIS_ARBOR_MOTOR = 5.65    # [der] 6.75 - (1.20 + 1.00)/2

# --- pot pocket ears (middle shell, pot z-datum) ---
CASE_POT_EAR_L = 3.0        # [cal] 2x3 tab; 3.0 = circumferential (split assumed)
CASE_POT_EAR_W = 2.0        # [cal] radial
CASE_POT_EAR_PAIR_DEG = 180 # [cal] two ears, opposite
CASE_POT_EAR_H_EST = 0.8    # [cal] eyeballed 0.5-1, midpoint
CASE_POT_EAR_ANGLE_EST = 0  # UNVERIFIED - assumed +/-X: must dodge the
                            # leg zone (-90 +/- 48 deg, wiper at -Y)

# --- pot shaft, disassembled (supersedes assembled-state guesses) ---
POT_SHAFT_TOTAL_L = 12.79  # [cal] incl rear magnet boss
POT_SHAFT_MAIN_D = 1.85    # [cal] gear2's z-stop shoulder (z-stop requires
                           # gear2 wheel bore < 1.85); 1.35 above is nominal
                           # (confirmed across donors), gear4 D-bore mates
                           # there. Gear2 bore UNMEASURED and load-bearing:
                           # the old 1.9 figure may be derived, likely a
                           # STEPPED bore (wheel ~1.4 / pinion relief ~1.9);
                           # either way pinion wall ~0.12 (8T m0.30 root
                           # 1.65), the tooth-strip stage - NEVER ream
POT_SHAFT_TIP_D = 1.35     # [cal] tip round section (gear4 D-bore)
POT_SHAFT_TIP_FLAT = 1.00  # [cal] across the D flat
# step position along the shaft: not yet measured
POT_BODY_TO_BUSH = 4.5     # [cal] front face to rear bushing tip
POT_REAR_BOSS_STICKOUT = 0.2   # [der] 4.5 - 4.3 body; magnet glue surface
POT_FRONT_STUB_L = 8.29    # [der] 12.79 - 4.5, carries gear2 + gear4

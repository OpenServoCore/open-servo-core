"""Tier-2 carrier parameters.

Replaces the SG90 feedback pot: printed stator body in the pot envelope,
steel pin shaft, brass tube bearing, printed magnet cup, coupon shelf.
Purchased metal is AliExpress stock; donor envelope imports from
sg90.measurements. Same *_EST convention: unverified until measured or
bench-fit (press/running fits get a printed ladder before they are
trusted).

z layout: carrier back face (ear seating plane) = 0, front face up,
matching sg90/pot.py. Wiper-leg direction = -Y; shell ears assumed at
+/-X (CASE_POT_EAR_ANGLE_EST), hanger arms at +/-Y in the ear gaps.
"""
from sg90 import measurements as m

# --- purchased metal ---
PIN_D = 1.4            # H62 brass rod (donor shaft class)
PIN_L = 14.0           # cut length; donor shaft spans 12.79
PIN_FLAT_ACROSS = 1.02 # D-flat filed on the top gear4 zone (jig-defined
                       # depth, steel wear strips): form closure, NOT
                       # friction - gear4 is the load-bearing gear and
                       # POM stress relaxation loosens press fits, which
                       # would read as encoder zero drift
PIN_FLAT_L = 3.5       # matches gear4 engagement
BUSH_OD = 2.5          # brass tube, 0.5 wall stock
BUSH_ID = 1.5          # pin running fit; pick stock that spins free
G2_SLEEVE_OD = 1.8     # gear2 journal sleeve - CONTINGENT on the gear2
G2_SLEEVE_ID = 1.5     # bore measurement (likely stepped, wheel ~1.4 /
                       # pinion relief ~1.9): bore >=1.45 -> DELETE, gear2
                       # rides the pin; bore ~1.9 -> keep. NEVER ream
                       # gear2 (~0.12 pinion wall, the tooth-strip stage)
MAG_D = 3.0            # D3x1.5 diametral NdFeB
MAG_H = 1.5

# --- axial layout ---
BODY_D = m.POT_BODY_D          # 9.92 keeps the shell pocket fit
BODY_H = m.POT_BODY_L          # 4.30 keeps the shell stack unchanged
BUSH_L = 4.0                   # cut length; pressed until FLUSH with the
                               # front face (flat plate against the face
                               # = its own depth stop), so the ~0.3
                               # bottom recess is pure clearance - tube
                               # axial position is NON-critical, the
                               # tube is a radial bearing only
BUSH_TOP_Z = BODY_H
BUSH_Z = BODY_H - BUSH_L
SEAT_BOSS_D = 3.0              # printed gear2 thrust seat on the front
SEAT_BOSS_H_EST = 0.2          # UNVERIFIED - donor shoulder height is a
                               # PRECISION copy target (+0.3 ear-inset
                               # offset): the shoulder plane = gear1 wheel
                               # top, sets gear2's mesh engagement; the
                               # donor stacks every gear face-to-face off
                               # the shaft steps (D-start = gear2 pinion
                               # top -> gear4 press-jig depth, same rule)
GEAR2_SEAT_Z = BODY_H + SEAT_BOSS_H_EST
PIN_TOP_Z = 12.1               # inside the gear4 D-bore
PIN_BOT_Z = PIN_TOP_Z - PIN_L  # -1.9, cup grips this end
GEAR4_BORE_DEPTH_EST = 3.5     # UNVERIFIED - sets PIN_TOP_Z for real

# --- fits (bench ladder) ---
BUSH_PRESS = 0.08      # bore = BUSH_OD - this
CUP_RIB_GRIP = 0.10    # rib crest dia = PIN_D - this
CUP_HUB_CLEAR = 0.12   # cup hub in the body bore

# --- magnet cup (rotor) ---
CUP_OD = 4.6
CUP_HUB_D = BUSH_OD - BUSH_PRESS - CUP_HUB_CLEAR
CUP_HUB_TOP_Z = 0.10           # labyrinth stub only, never a thrust
                               # face; >=0.1 clear of the tube bottom
CUP_TOP_Z = 0.0                # shoulder plane = up-thrust against the
                               # body back face (printed, z-exact);
                               # down-stop = gear stack on the seat
                               # boss; end float is baked in by the two
                               # hard-stop presses, no press-by-feel
CUP_WEB_T = 0.3                # between pin end and magnet
CUP_NUB_H = 0.13               # snap nubs under the magnet mouth
CUP_NUB_D = MAG_D - 0.3        # nub crest opening (0.15/side engagement)
CUP_SLOT_W = 0.8               # petal slots: 3 cuts make the rim 3
                               # cantilevers so the nubs can deflect
                               # (~0.15 tip travel, few % strain);
                               # closed rim is hoop-stiff, would crack
CUP_POCKET_RIB = 0.10          # petal-spring radial ribs: crest dia =
                               # MAG_D - this, petals preload the magnet
                               # sideways after snap-in (centers it and
                               # friction-locks rotation)
CUP_BUMP_H = 0.06              # ceiling crush bumps: anvil press yields
                               # them, eats z tolerance, leaves the
                               # magnet clamped bumps-vs-nubs, zero float
MAG_TOP_Z = PIN_BOT_Z - 0.1 - CUP_WEB_T - CUP_BUMP_H  # nominal magnet top
CUP_RIM_Z = MAG_TOP_Z - MAG_H - 0.02 - CUP_NUB_H - 0.05

# --- shell interface ---
EAR_H = m.CASE_POT_EAR_H_EST
WEB_TOP_Z = -EAR_H
WEB_BOT_Z = -EAR_H - m.CASE_POT_SEAT_T_EST
OPENING_R = m.CASE_POT_SEAT_ID_EST / 2   # 4.3, everything hanging below
                                         # must insert through this
# the shell EARS are the anti-rotation key: two insets in the bottom
# face at +/-X drop over them, so the ears give both the z-datum (inset
# floor seats on ear top) and the rotational lock (inset side walls);
# a protruding key stub had no room in the shell
EAR_INSET_L = m.CASE_POT_EAR_L + 0.4   # circumferential clearance
EAR_INSET_W = m.CASE_POT_EAR_W + 0.4   # radial clearance
EAR_INSET_DEPTH = 0.3  # MUST stay < min ear height (0.5 eyeball) or the
                       # back face bottoms on the pocket floor web and
                       # the z-datum goes ambiguous

# --- arc shells + round coupon, bayonet mount ---
# two arc walls hang from the body (gaps at +/-X, under the ear
# insets); round board pushes up through the gaps and twists ~35deg so
# its two rim tabs ride under the arcs to an end stop - board top lands
# on a printed ledge, so the magnet-sensor gap is one printed dimension
ARM_R_IN = 3.55
ARM_R_OUT = 4.15
ARM_BOT_Z = -6.3
GAP_HALF_DEG = 20.0    # gap sectors at +/-X, tabs pass through here
HOOK_T = 0.35          # snap bump past the opening edge
HOOK_H = 0.5
HOOK_TOP_Z = WEB_BOT_Z - 0.05
FINGER_W = 2.4         # snap finger freed from the arc (arcs are
FINGER_SLOT = 0.7      # hoop-stiff, same lesson as the cup rim)
FINGER_SLOT_BOT_Z = -4.2
COUPON_D = 7.0         # round board, fits inside the arcs
COUPON_T = 1.0
COUPON_TOP_Z = -4.95   # = ledge underside and bayonet ceiling
TAB_W = 2.0            # board rim tabs at 180 deg
TAB_R_OUT = 4.2
BAY_HALF_DEG = 50.0    # bayonet band from each gap; travel ~35, stop 50
BAY_TOP_Z = COUPON_TOP_Z
BAY_BOT_Z = COUPON_TOP_Z - COUPON_T - 0.03
LEDGE_R_IN = 3.3       # board seat ring under the arc inner face
LEDGE_H = 0.5
SENSOR_SZ = 1.6        # SC4251 DFN-6 body
SENSOR_H = 0.4

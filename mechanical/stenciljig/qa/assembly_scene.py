"""Export assembly glbs for the two-part stack:
- assembly_stacked.glb: full jig locked onto the stocked box (knobs
  turned, hooks latched), box keyed blue for transparency
- assembly_apart.glb: working layout - jig standing on its feet
  beside the open box
Colors are keys for render_assembly.py: box (0.45,0.58,0.75) renders
see-through, side plates (0.35,0.37,0.40) half-transparent (ghost
scenes only - "real" keeps every plastic its true print color)."""
import pathlib

from bd_warehouse.fastener import ButtonHeadScrew, HexNut
from build123d import *

from stenciljig.adapter import build_adapter, AD_CBORE
from stenciljig.base import build_base
from stenciljig.box import (build_box, AD_D, AD_XS, AD_Z, BAY_D,
                            BAY_Y1, BOX_FOOT_POSE, BOX_H, EXT,
                            FLOOR_Z, PKT, SHEET, SHEET_Z, SQ_XY0)
from stenciljig.clamp_plate import build_clamp_plate
from stenciljig.common import (foot_pt, AD_SCREW_Y, BASE_T, CAM_XY,
                               CAP_T, CLAMP_T, COL_X, COL_Y, FOOT_H,
                               FOOT_INSET, FOOT_POSE, FOOT_SCREW_OFF,
                               LEDGE, PCB_TOP, PCB_W, PLATE_H,
                               PLATE_YC, SIDE_W, SLOT_X0,
                               SLOT_Y, BASE_D)
from stenciljig.foot import build_foot
from stenciljig.knob import build_knob, HANDLE_T, SEAT_Z
from stenciljig.nutbar import build_nutbar
from stenciljig.pad import build_pad, PAD_PROUD
from stenciljig.side_plate import build_side_plate, EAR_T
from stenciljig.squeegee import build_squeegee, SQ_SIZES
from stenciljig.tray import build_tray

OUT = pathlib.Path("stenciljig/build")
BOX_C = Color(0.45, 0.58, 0.75)
PLATE_C = Color(0.35, 0.37, 0.40)
STEEL = Color(0.75, 0.76, 0.78)

box = build_box()
box.color = BOX_C
knob = build_knob()
pad = build_pad()

_SCREWS = {}


def screw(x, y, z_seat, length, up=False):
    """ISO 7380 M3 button head (bd_warehouse, threaded): seat plane
    lands at z_seat, shank down (up=True flips it shank-up for the
    from-below foot screws)."""
    if length not in _SCREWS:
        _SCREWS[length] = ButtonHeadScrew(
            size="M3-0.5", length=length,
            fastener_type="iso7380_1", simple=False)
    q = Pos(x, y, z_seat) * (Rot(180, 0, 0) if up else Rot(0, 0, 0)) \
        * _SCREWS[length]
    q.color = STEEL
    return q


NUT = HexNut(size="M3-0.5", fastener_type="iso4032")


def nut(pose):
    q = pose * NUT
    q.color = STEEL
    return q


def jig_parts(locked, real=False):
    """Base + feet + plates + bars + clamp + knobs + screws + foot
    pads, in the standard frame (base underside z=0). Ghost scenes
    key the side plates half-transparent so the captive nuts read."""
    parts = [build_base()]
    for b in (build_side_plate, build_nutbar):
        q = b()
        if b is build_side_plate and not real:
            q.color = PLATE_C
        m = mirror(q, Plane.YZ)
        m.color = q.color
        parts += [q, m]
    # planks rotate (not mirror) to the left hand - same edge inboard,
    # labels stay right-reading
    q = build_adapter(1.6, 1.0)
    m = Pos(0, 2 * PLATE_YC, 0) * Rot(0, 0, 180) * q
    m.color = q.color
    parts += [q, m]
    parts.append(build_clamp_plate())
    if locked:
        for s in (1, -1):
            at = Pos(s * CAM_XY[0], CAM_XY[1], BASE_T)
            k = at * Rot(0, 0, -90) * knob
            k.color = knob.color
            parts.append(k)
            parts.append(screw(s * CAM_XY[0], CAM_XY[1],
                               BASE_T + SEAT_Z, 16))
    foot, hookfoot = build_foot(), build_foot(True)
    for x, y, a, hook in FOOT_POSE:
        f = Pos(x, y, 0) * Rot(0, 0, a) * (hookfoot if hook else foot)
        f.color = foot.color
        parts.append(f)
        q = Pos(x, y, -FOOT_H - PAD_PROUD) * pad
        q.color = pad.color
        parts.append(q)
        sx, sy = foot_pt(x, y, a, FOOT_SCREW_OFF)
        parts.append(screw(sx, sy, -4.7, 10, up=True))
    xc = PCB_W / 2 - LEDGE + SIDE_W / 2
    for s in (1, -1):
        for y in SLOT_Y:
            parts.append(screw(s * SLOT_X0, y, BASE_T + EAR_T, 16))
            # nut-bar nut, pressed up under the 2 web
            parts.append(nut(Pos(s * SLOT_X0, y, -4.4)))
        for y in COL_Y:
            parts.append(screw(s * COL_X, y,
                               PCB_TOP + CLAMP_T - 1.7, 16))
            # clamp nut seated in its bottom-entry press band
            parts.append(nut(Pos(s * COL_X, y, PCB_TOP - 5.0)))
        # plank screws down into the broached-up pocket nuts
        for y in AD_SCREW_Y:
            parts.append(screw(s * xc, y, PCB_TOP - AD_CBORE, 10))
            parts.append(nut(Pos(s * xc, y, 14.3 - 2.4)))
    return parts


def contents(with_knobs=False):
    parts = []
    if with_knobs:
        # between stackings the knobs live head-down in the bin
        for x in (32.0, 55.0):
            at = Pos(x, 16.0, FLOOR_Z + HANDLE_T) * Rot(180, 0, 0)
            k = at * knob
            k.color = knob.color
            parts.append(k)
            q = at * Pos(0, 0, SEAT_Z) * screw(0, 0, 0, 16)
            q.color = STEEL
            parts.append(q)
    # the 6 unmounted planks on edge, 3 in each wall-side crate (the
    # 1.6/1.0 pair rides the jig); pairs kept together per side
    for x0, (ta, tb) in zip(AD_XS, ((1.2, 0.8), (1.2, 0.8),
                                    (0.6, 0.4), (0.6, 0.4),
                                    (2.0, 1.6), (2.0, 1.6))):
        ad = Rot(0, 90, 0) * build_adapter(ta, tb)
        bb = ad.bounding_box()
        q = Pos(x0 + PKT - bb.min.X,
                BAY_Y1 - AD_D + PKT - bb.min.Y, AD_Z - bb.min.Z) * ad
        q.color = ad.color
        parts.append(q)
    x0, y0 = SQ_XY0
    for w, k in SQ_SIZES:
        sq = build_squeegee(w, k)
        bb = sq.bounding_box()
        q = Pos(x0 + PKT - bb.min.X, y0 + PKT - bb.min.Y,
                FLOOR_Z - bb.min.Z) * sq
        q.color = sq.color
        parts.append(q)
        x0 += (w - 1 + 2 * 2.5 + 2 * PKT) + 2.4
    for i in range(4):
        sh = Pos(-BAY_D / 2 + PKT, BAY_Y1 - SHEET - PKT,
                 SHEET_Z + i * 0.15) * Box(
            SHEET, SHEET, 0.12, align=(Align.MIN, Align.MIN, Align.MIN))
        sh.color = Color(0.75, 0.76, 0.78)
        parts.append(sh)
    tr = build_tray()
    q = Pos(0, BAY_Y1 - BAY_D / 2, FLOOR_Z) * tr
    q.color = tr.color
    parts.append(q)
    # the box stands on the same screw-on feet as the jig
    foot = build_foot()
    for x, y, a in BOX_FOOT_POSE:
        f = Pos(x, y, -BOX_H) * Rot(0, 0, a) * foot
        f.color = foot.color
        parts.append(f)
        q = Pos(x, y, -BOX_H - FOOT_H - PAD_PROUD) * pad
        q.color = pad.color
        parts.append(q)
        sx, sy = foot_pt(x, y, a, FOOT_SCREW_OFF)
        parts.append(screw(sx, sy, -BOX_H - 4.7, 10, up=True))
    return parts


def export(name, parts):
    comp = Compound(children=parts)
    export_gltf(comp, str(OUT / name), binary=True)
    print(f"{name}: {len(parts)} parts")


export("assembly_stacked.glb", [box] + contents() + jig_parts(True))


def apart_layout(box_part, real=False):
    """Working layout: jig on its feet beside the open box. Bench
    plane = the box feet's pad bottoms; the jig's own stance is
    identical, so the side-by-side offset is exactly -BOX_H."""
    parts = [box_part] + contents(with_knobs=True)
    for q in jig_parts(False, real):
        m = Pos(2 * EXT + 40.0, 0, -BOX_H) * q
        m.color = q.color
        parts.append(m)
    return parts


export("assembly_apart.glb", apart_layout(box))
# true print colors: orange box, black plates
export("assembly_real.glb", apart_layout(build_box(), real=True))

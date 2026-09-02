"""2D blueprint SVG from a part: third-angle front/top/right views,
hidden lines dashed, overall dimensions. python blueprint.py part.step
[out.svg], or blueprint(part, path) from a build script."""
import sys

from build123d import GeomType, import_step

D = 1000.0                 # parallel projection - distance is arbitrary

# view: (label, viewport_origin offset, up) in the part frame; screen
# axes come out as front=(X,Z), top=(X,Y), right=(Y,Z)
VIEWS = (
    ("FRONT", (0, -D, 0), (0, 0, 1)),
    ("TOP", (0, 0, D), (0, 1, 0)),
    ("RIGHT", (D, 0, 0), (0, 0, 1)),
)
MARGIN = 12.0
GAP = 14.0                 # between views; dims live in it
FONT = 3.0
DIM_OFF = 7.0              # dimension line offset from the view


def _polyline(e):
    n = 1 if e.geom_type == GeomType.LINE else 48
    return [(p.X, p.Y) for p in (e @ (i / n) for i in range(n + 1))]


def _project(part, origin, up):
    c = part.center()
    vis, hid = part.project_to_viewport(
        (c.X + origin[0], c.Y + origin[1], c.Z + origin[2]),
        viewport_up=up, look_at=c)
    vis = [_polyline(e) for e in vis]
    hid = [_polyline(e) for e in hid]
    xs = [x for pl in vis + hid for x, _ in pl]
    ys = [y for pl in vis + hid for _, y in pl]
    return vis, hid, (min(xs), min(ys), max(xs), max(ys))


def _paths(rows, polylines, dx, dy, scale, sheet_h, cls):
    for pl in polylines:
        d = " ".join(f"{'M' if i == 0 else 'L'}"
                     f"{(x * scale + dx):.2f},{(sheet_h - (y * scale + dy)):.2f}"
                     for i, (x, y) in enumerate(pl))
        rows.append(f'<path class="{cls}" d="{d}"/>')


def _hdim(rows, x0, x1, y, sheet_h, label):
    y = sheet_h - y
    rows += [
        f'<path class="d" d="M{x0:.2f},{y:.2f} L{x1:.2f},{y:.2f}"/>',
        f'<path class="d" d="M{x0:.2f},{y - 1.2:.2f} L{x0:.2f},{y + 1.2:.2f}'
        f' M{x1:.2f},{y - 1.2:.2f} L{x1:.2f},{y + 1.2:.2f}"/>',
        f'<text x="{(x0 + x1) / 2:.2f}" y="{y - 1.0:.2f}" '
        f'text-anchor="middle">{label}</text>',
    ]


def _vdim(rows, y0, y1, x, sheet_h, label):
    ya, yb = sheet_h - y1, sheet_h - y0
    rows += [
        f'<path class="d" d="M{x:.2f},{ya:.2f} L{x:.2f},{yb:.2f}"/>',
        f'<path class="d" d="M{x - 1.2:.2f},{ya:.2f} L{x + 1.2:.2f},{ya:.2f}'
        f' M{x - 1.2:.2f},{yb:.2f} L{x + 1.2:.2f},{yb:.2f}"/>',
        f'<text x="{x - 1.0:.2f}" y="{(ya + yb) / 2:.2f}" text-anchor="middle"'
        f' transform="rotate(-90 {x - 1.0:.2f} {(ya + yb) / 2:.2f})">'
        f'{label}</text>',
    ]


def blueprint(part, out_path, name=None):
    views = {lbl: _project(part, o, up) for lbl, o, up in VIEWS}
    bb = part.bounding_box()
    w, dep, h = bb.size.X, bb.size.Y, bb.size.Z
    scale = next(s for s in (5, 2, 1, 0.5)
                 if max(w, dep) * s <= 180 and (h + dep) * s <= 190)

    def sized(lbl):
        _, _, (x0, y0, x1, y1) = views[lbl]
        return (x1 - x0) * scale, (y1 - y0) * scale

    fw, fh = sized("FRONT")
    tw, th = sized("TOP")
    rw, rh = sized("RIGHT")
    # third angle: TOP above FRONT (X shared), RIGHT beside FRONT (Z shared)
    fx, fy = MARGIN + DIM_OFF, MARGIN + DIM_OFF
    tx, ty = fx, fy + fh + GAP
    rx, ry = fx + fw + GAP, fy
    sheet_w = max(rx + rw, tx + tw) + MARGIN
    sheet_h = ty + th + MARGIN + 2 * FONT

    rows = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{sheet_w:.0f}mm" '
        f'height="{sheet_h:.0f}mm" viewBox="0 0 {sheet_w:.2f} {sheet_h:.2f}">',
        '<style>path{fill:none;stroke:black}'
        '.v{stroke-width:0.35}.h{stroke-width:0.18;stroke-dasharray:1.2 0.8}'
        '.d{stroke-width:0.13}'
        f'text{{font:{FONT}px sans-serif;fill:black}}</style>',
        f'<rect width="{sheet_w:.2f}" height="{sheet_h:.2f}" fill="white"/>',
    ]
    for lbl, (ox, oy) in (("FRONT", (fx, fy)), ("TOP", (tx, ty)),
                          ("RIGHT", (rx, ry))):
        vis, hid, (x0, y0, _, _) = views[lbl]
        dx, dy = ox - x0 * scale, oy - y0 * scale
        _paths(rows, hid, dx, dy, scale, sheet_h, "h")
        _paths(rows, vis, dx, dy, scale, sheet_h, "v")
        rows.append(f'<text x="{ox:.2f}" '
                    f'y="{sheet_h - (oy - 1.5):.2f}">{lbl}</text>')
    _hdim(rows, fx, fx + fw, fy - DIM_OFF + 2.5, sheet_h, f"{w:.2f}")
    _vdim(rows, fy, fy + fh, fx - DIM_OFF + 2.5, sheet_h, f"{h:.2f}")
    _hdim(rows, rx, rx + rw, ry - DIM_OFF + 2.5, sheet_h, f"{dep:.2f}")
    title = name or getattr(part, "label", "") or "part"
    rows.append(
        f'<text x="{sheet_w - MARGIN:.2f}" y="{sheet_h - 3:.2f}" '
        f'text-anchor="end">{title} | mm | '
        f'{scale:g}:1 | third angle</text>')
    rows.append("</svg>")
    with open(out_path, "w") as f:
        f.write("\n".join(rows))
    return out_path


if __name__ == "__main__":
    step = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else step.rsplit(".", 1)[0] + "_bp.svg"
    part = import_step(step)
    print("saved", blueprint(part, out, name=step.rsplit("/", 1)[-1]))

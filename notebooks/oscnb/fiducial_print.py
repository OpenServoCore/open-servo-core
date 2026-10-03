"""Printable sheets for the fiducial angle rig, US letter, as vector PDF.

  uv run python -m oscnb.fiducial_print <out dir>

writes fiducial-tags.pdf (the horn and reference tags, each with a 50.00 mm
caliper square and 50 mm rulers along X and Y, plus spares) and
fiducial-board.pdf (the ChArUco board for camera calibration). Print both at
100% ("Actual size"), never "Fit to page".
"""

import sys
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

from . import fiducial as fd  # noqa: E402

PAGE_MM = (215.9, 279.4)
MM_PER_IN = 25.4
LINE_MM = 0.5
TICK_MM = 0.25


def page():
    fig = plt.figure(figsize=(PAGE_MM[0] / MM_PER_IN, PAGE_MM[1] / MM_PER_IN))
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, PAGE_MM[0])
    ax.set_ylim(PAGE_MM[1], 0)
    ax.set_axis_off()
    return fig, ax


def box(ax, x, y, w, h, color="k"):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=color, edgecolor="none", linewidth=0))


def tag(ax, tag_id, cx, cy, side=fd.TAG_MM):
    """The tag's black square centred on (cx, cy), page X across, Y down, a
    dashed cut line around its white quiet zone and its name under it."""
    d = fd.dictionary()
    cells = d.markerSize + 2
    bits = cv2.aruco.generateImageMarker(d, tag_id, cells, borderBits=1)
    c = side / cells
    x0, y0 = cx - side / 2, cy - side / 2
    for r in range(cells):
        k = 0
        while k < cells:
            if bits[r, k] == 0:
                run = k
                while run < cells and bits[r, run] == 0:
                    run += 1
                box(ax, x0 + k * c, y0 + r * c, (run - k) * c, c)
                k = run
            else:
                k += 1
    q = side / 2 + 1.5 * c
    ax.add_patch(Rectangle((cx - q, cy - q), 2 * q, 2 * q, fill=False, edgecolor="0.6",
                           linewidth=0.4, linestyle=(0, (3, 3))))
    return q


def check_square(ax, x, y, side=fd.CHECK_MM):
    """A frame whose OUTER edges are side mm apart along X and along Y."""
    box(ax, x, y, side, LINE_MM)
    box(ax, x, y + side - LINE_MM, side, LINE_MM)
    box(ax, x, y, LINE_MM, side)
    box(ax, x + side - LINE_MM, y, LINE_MM, side)


def ruler(ax, x, y, length=fd.CHECK_MM, vertical=False):
    """mm ticks; the OUTER edges of the two end ticks are length mm apart."""
    for mm in range(int(length) + 1):
        tall = 4.0 if mm % 10 == 0 else 2.5 if mm % 5 == 0 else 1.5
        pos = mm - (TICK_MM if mm == length else 0.0 if mm == 0 else TICK_MM / 2)
        if vertical:
            box(ax, x, y + pos, tall, TICK_MM)
        else:
            box(ax, x + pos, y, TICK_MM, tall)
    if vertical:
        box(ax, x, y, 0.4, length)
    else:
        box(ax, x, y, length, 0.4)


def tag_row(ax, tag_id, name, y):
    cx = 30.0
    tag(ax, tag_id, cx, y + 25)
    ax.text(cx, y + 47, f"{name}  (id {tag_id})", ha="center", va="top", fontsize=8)
    sx = 66.0
    check_square(ax, sx, y)
    ruler(ax, sx, y + 53)
    ruler(ax, sx + 54, y, vertical=True)
    ax.text(sx + 25, y + 61, "X: 50.00 mm", ha="center", va="top", fontsize=6)
    ax.text(sx + 61, y + 25, "Y: 50.00 mm", ha="left", va="center", fontsize=6, rotation=-90)


def tag_sheet(path):
    fig, ax = page()
    ax.text(12, 10, "OpenServoCore output-angle tags   ArUco DICT_4X4_50, 25.00 mm black square",
            fontsize=9, va="top", weight="bold")
    ax.text(12, 15, "Print at 100% (Actual size). Measure each frame's OUTSIDE edges along X and Y, and each "
            "ruler between the outer\nedges of its end ticks, with calipers. Record the numbers. "
            "Keep the tags flat: dry glue or tape, no wet paste.", fontsize=6.5, va="top")
    tag_row(ax, fd.HORN_ID, "HORN", 30)
    tag_row(ax, fd.REF_ID, "REFERENCE", 100)
    ax.text(12, 172, "spares", fontsize=7, va="top")
    for i, (tid, name) in enumerate([(fd.HORN_ID, "HORN"), (fd.REF_ID, "REF"),
                                     (fd.HORN_ID, "HORN"), (fd.REF_ID, "REF")]):
        cx = 30 + i * 46
        tag(ax, tid, cx, 200)
        ax.text(cx, 222, f"{name} (id {tid})", ha="center", va="top", fontsize=6)
    ax.annotate("", xy=(195, 240), xytext=(175, 240), arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.text(185, 243, "page X", ha="center", va="top", fontsize=6)
    fig.savefig(path)
    plt.close(fig)


def board_sheet(path, px_per_mm=20):
    fig, ax = page()
    cols, rows = fd.BOARD_SQUARES
    w, h = cols * fd.BOARD_SQUARE_MM, rows * fd.BOARD_SQUARE_MM
    x0, y0 = (PAGE_MM[0] - w) / 2, (PAGE_MM[1] - h) / 2 + 3
    img = fd.charuco_board().generateImage((int(w * px_per_mm), int(h * px_per_mm)), marginSize=0, borderBits=1)
    ax.imshow(img, cmap="gray", vmin=0, vmax=255, interpolation="none", extent=(x0, x0 + w, y0 + h, y0))
    ax.set_xlim(0, PAGE_MM[0])
    ax.set_ylim(PAGE_MM[1], 0)
    ax.text(x0, 6, f"ChArUco {cols} x {rows}, DICT_5X5_100, squares {fd.BOARD_SQUARE_MM:.2f} mm, markers "
            f"{fd.BOARD_MARKER_MM:.2f} mm. Print at 100%. Check: 5 squares = "
            f"{5 * fd.BOARD_SQUARE_MM:.2f} mm along X and along Y. Mount flat on a rigid board.",
            fontsize=6, va="top")
    fig.savefig(path)
    plt.close(fig)


def main(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    tag_sheet(out / "fiducial-tags.pdf")
    board_sheet(out / "fiducial-board.pdf")
    print(f"wrote {out / 'fiducial-tags.pdf'} and {out / 'fiducial-board.pdf'}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")

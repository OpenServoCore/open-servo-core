"""Headless STL preview: python -m sg90.render build/part.stl [out.png]"""
import sys

import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def render(stl_path: str, png_path: str, views=((20, -60), (-25, 120))):
    mesh = trimesh.load(stl_path)
    fig = plt.figure(figsize=(6 * len(views), 6))
    light = np.array([0.4, -0.6, 0.7])
    light /= np.linalg.norm(light)
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        shade = 0.25 + 0.75 * np.clip(mesh.face_normals @ light, 0, 1)
        colors = plt.cm.Greys(0.25 + 0.6 * (1 - shade))
        ax.add_collection3d(Poly3DCollection(
            mesh.vertices[mesh.faces], facecolors=colors, edgecolor="none"))
        lo, hi = mesh.bounds
        c, r = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az)
        ax.set_axis_off()
        ax.set_box_aspect((1, 1, 1))
    plt.tight_layout()
    plt.savefig(png_path, dpi=110, facecolor="white")
    plt.close(fig)
    return png_path


if __name__ == "__main__":
    stl = sys.argv[1]
    png = sys.argv[2] if len(sys.argv) > 2 else stl.rsplit(".", 1)[0] + ".png"
    print("saved", render(stl, png))

"""Exact-vertex edge-manifold audit on the exported STLs: every edge
shared by exactly 2 triangles = watertight. OCC T-junction seams
report as open edges - slicers stitch those, so nonzero counts only
matter for parts that must be strictly watertight (reported holes).
Run from mechanical/ after `python -m stenciljig`:
.venv/bin/python stenciljig/qa/check_watertight.py"""
import pathlib
import struct

BUILD = pathlib.Path(__file__).parent.parent / "build"


def read_stl(path):
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:300]:
        tris, vs = [], []
        for line in data.decode(errors="ignore").splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                vs.append(tuple(float(x) for x in line.split()[1:4]))
                if len(vs) == 3:
                    tris.append(tuple(vs))
                    vs = []
        return tris
    n = struct.unpack_from("<I", data, 80)[0]
    tris = []
    off = 84
    for _ in range(n):
        v = struct.unpack_from("<12f", data, off)
        tris.append(((v[3], v[4], v[5]), (v[6], v[7], v[8]),
                     (v[9], v[10], v[11])))
        off += 50
    return tris


for path in sorted(BUILD.glob("*.stl")):
    tris = read_stl(path)
    edges = {}
    for a, b, c in tris:
        for e in ((a, b), (b, c), (c, a)):
            k = tuple(sorted(e))
            edges[k] = edges.get(k, 0) + 1
    open_e = sum(1 for n in edges.values() if n != 2)
    tag = "watertight" if open_e == 0 else f"{open_e} OPEN EDGES"
    print(f"{path.stem:28s} {len(tris):6d} tris  {tag}")

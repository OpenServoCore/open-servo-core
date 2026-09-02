"""Headless Blender render of STLs to PNG:

    blender -b -P render_blender.py -- part.stl [more.stl ...] [out.png]

Workbench engine with studio light + cavity shading: fast, crisp edges,
reads like a technical viewport. Multiple STLs render as one scene in
their modeled positions (pass an exploded/assembly export for those
views). Output defaults to first STL's name with .png.
"""
import math
import pathlib
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
stls = [a for a in argv if a.lower().endswith(".stl")]
outs = [a for a in argv if a.lower().endswith(".png")]
if not stls:
    sys.exit(__doc__)
out = outs[0] if outs else str(pathlib.Path(stls[0]).with_suffix(".png"))

bpy.ops.wm.read_factory_settings(use_empty=True)
for f in stls:
    bpy.ops.wm.stl_import(filepath=str(pathlib.Path(f).resolve()))
meshes = [o for o in bpy.data.objects if o.type == "MESH"]
for o in meshes:
    with bpy.context.temp_override(object=o):
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(30))

pts = [o.matrix_world @ Vector(c) for o in meshes for c in o.bound_box]
lo = Vector((min(p[i] for p in pts) for i in range(3)))
hi = Vector((max(p[i] for p in pts) for i in range(3)))
center, size = (lo + hi) / 2, max(hi - lo)

cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
bpy.context.scene.collection.objects.link(cam)
cam.data.lens = 85
direction = Vector((1, -1, 0.8)).normalized()
dist = size / 2 / math.tan(cam.data.angle / 2) + size
cam.location = center + direction * dist
cam.rotation_euler = direction.to_track_quat("Z", "Y").to_euler()
cam.data.clip_end = dist + size * 4
bpy.context.scene.camera = cam

scene = bpy.context.scene
scene.render.engine = "BLENDER_WORKBENCH"
shading = scene.display.shading
shading.light = "STUDIO"
shading.show_cavity = True
shading.cavity_type = "BOTH"
shading.color_type = "SINGLE"
shading.single_color = (0.75, 0.75, 0.78)
scene.display.render_aa = "16"
scene.render.resolution_x = 1600
scene.render.resolution_y = 1200
scene.render.film_transparent = False
scene.world = bpy.data.worlds.new("w")
scene.world.color = (1, 1, 1)
scene.render.filepath = str(pathlib.Path(out).resolve())
bpy.ops.render.render(write_still=True)
print("saved", scene.render.filepath)

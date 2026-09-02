"""Blender headless renders of the chassis assembly glbs with a
see-through sleeve (and half-transparent drawer) so the wheel /
ledge / snap mechanism reads. Two shots per pose: front-right and
rear-right. Run:
blender -b -P render_assembly.py -- <outdir> <glb> [<glb> ...]"""
import math
import sys

import bpy
from mathutils import Vector

args = sys.argv[sys.argv.index("--") + 1:]
outdir, glbs = args[0], args[1:]

# (rgb to match, alpha, metallic, roughness)
KEYS = (
    ((0.45, 0.58, 0.75), 0.28, 0.0, 0.35),   # box key - see-through
    ((0.35, 0.37, 0.40), 0.55, 0.0, 0.5),    # plates smoke key - half
    ((0.75, 0.76, 0.78), 1.0, 1.0, 0.2),     # fastener steel
)

for glb in glbs:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=glb)

    lo = Vector((1e9,) * 3)
    hi = Vector((-1e9,) * 3)
    for ob in bpy.data.objects:
        if ob.type != "MESH":
            continue
        for c in ob.bound_box:
            w = ob.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    center = (lo + hi) / 2
    size = max(hi - lo)

    # the exporter stores linear values - compare keys through ^2.2.
    # "real" scenes keep every plastic opaque (true print colors);
    # only the steel key still applies
    ghost = "real" not in glb
    for m in bpy.data.materials:
        b = m.node_tree.nodes.get("Principled BSDF")
        if not b:
            continue
        c = b.inputs["Base Color"].default_value
        # unkeyed = printed plastic: matte, low specular - big soft
        # lamps otherwise wash dark colors into sheen-grey
        alpha, metal, rough = 1.0, 0.0, 0.75
        for (kr, kg, kb), ka, km, krf in KEYS:
            if (abs(c[0] - kr ** 2.2) + abs(c[1] - kg ** 2.2)
                    + abs(c[2] - kb ** 2.2)) < 0.05:
                if ka < 1.0 and not ghost:
                    continue
                alpha, metal, rough = ka, km, krf
        b.inputs["Metallic"].default_value = metal
        b.inputs["Roughness"].default_value = rough
        b.inputs["Alpha"].default_value = alpha
        if metal == 0.0:
            for spec in ("Specular IOR Level", "Specular"):
                if spec in b.inputs:
                    b.inputs[spec].default_value = 0.25
                    break
        try:
            m.blend_method = "BLEND"
        except Exception:
            pass

    bpy.ops.mesh.primitive_plane_add(
        size=size * 30, location=(center.x, center.y, lo.z - 0.0002))
    floor = bpy.context.object
    fm = bpy.data.materials.new("floor")
    fm.use_nodes = True
    fb = fm.node_tree.nodes["Principled BSDF"]
    fb.inputs["Base Color"].default_value = (0.55, 0.55, 0.56, 1)
    fb.inputs["Roughness"].default_value = 0.9
    floor.data.materials.append(fm)

    # AgX (the default) lifts blacks and pastels saturated plastics;
    # bare Standard clips - white parts blow to paper cutouts. PBR
    # Neutral keeps filament colors true with highlight rolloff
    for vt in ("Khronos PBR Neutral", "Standard"):
        try:
            bpy.context.scene.view_settings.view_transform = vt
            break
        except Exception:
            pass
    bpy.context.scene.view_settings.exposure = -0.3

    world = bpy.data.worlds.new("w")
    bpy.context.scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (
        0.55, 0.58, 0.63, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.3

    def lamp(name, dx, dy, dz, power, sz):
        bpy.ops.object.light_add(
            type="AREA",
            location=(center.x + dx * size, center.y + dy * size,
                      lo.z + dz * size))
        li = bpy.context.object
        li.name = name
        li.data.energy = power * size * size * 12
        li.data.size = sz * size
        li.rotation_mode = "QUATERNION"
        li.rotation_quaternion = (
            Vector((center.x, center.y, lo.z + 0.1 * size)) -
            li.location).to_track_quat("-Z", "Y")
        return li

    lamp("key", -0.9, -0.9, 1.6, 13, 1.2)
    lamp("fill", 1.2, -0.6, 1.0, 5, 1.6)
    lamp("rim", 0.3, 1.4, 1.2, 7, 1.0)

    target = bpy.data.objects.new("target", None)
    bpy.context.collection.objects.link(target)
    cam_data = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    tr = cam.constraints.new("TRACK_TO")
    tr.target = target

    sc = bpy.context.scene
    sc.camera = cam
    sc.render.engine = "CYCLES"
    # Metal GPU + denoiser-carried low samples: CPU Cycles at 200
    # samples pegged every core for minutes per frame
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "METAL"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    sc.cycles.device = "GPU"
    sc.cycles.samples = 64
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.use_denoising = True
    sc.render.resolution_x = 1920
    sc.render.resolution_y = 1280

    stem = glb.rsplit("/", 1)[-1].rsplit(".", 1)[0]
    SHOTS = {
        f"{stem}_front": ((0, 0, -0.02), 36, 18, 1.9, 55),
        f"{stem}_rear": ((0, 0.05, -0.04), 152, 14, 1.8, 55),
    }
    for name, (toff, az, el, dist, lens) in SHOTS.items():
        target.location = center + Vector(toff) * size
        a, e = math.radians(az), math.radians(el)
        cam.location = target.location + Vector((
            math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e),
            math.sin(e))) * dist * size
        cam_data.lens = lens
        sc.render.filepath = f"{outdir}/{name}.png"
        bpy.ops.render.render(write_still=True)
        print("wrote", sc.render.filepath)

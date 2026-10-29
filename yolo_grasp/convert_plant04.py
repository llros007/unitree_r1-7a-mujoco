# SPDX-License-Identifier: AGPL-3.0-or-later
"""Run with Blender: blender -b -t 2 --python yolo_grasp/convert_plant04.py.

Bake the downloaded Blender model to textured Z-up OBJ, uniformly scaled to
60 mm outer pot diameter. Runtime uses these baked assets without Blender.
"""
import json
from pathlib import Path

import bpy
from mathutils import Vector

ASSETS = Path(__file__).resolve().parent / 'assets' / 'plant04'
bpy.ops.wm.open_mainfile(filepath=str(ASSETS / 'potted_plant_04_4k.blend'))
depsgraph = bpy.context.evaluated_depsgraph_get()
pot = bpy.data.objects['potted_plant_04_pot'].evaluated_get(depsgraph)
corners = [pot.matrix_world @ v.co for v in pot.data.vertices]
low = Vector([min(p[i] for p in corners) for i in range(3)])
high = Vector([max(p[i] for p in corners) for i in range(3)])
scale = .06 / max(high.x - low.x, high.y - low.y)
origin = (low + high) / 2
parts = []
for obj in bpy.data.objects:
    if obj.type != 'MESH':
        continue
    obj = obj.evaluated_get(depsgraph)
    mesh = obj.to_mesh()
    mesh.calc_loop_triangles()
    vertices = [(obj.matrix_world @ v.co - origin) * scale for v in mesh.vertices]
    uv = mesh.uv_layers.active.data
    path = ASSETS / (obj.name + '.obj')
    with path.open('w') as out:
        out.write('# Poly Haven potted_plant_04; baked from downloaded Blender asset\n')
        for v in vertices:
            out.write('v %.9f %.9f %.9f\n' % tuple(v))
        for loop in uv:
            out.write('vt %.9f %.9f\n' % tuple(loop.uv))
        for triangle in mesh.loop_triangles:
            out.write('f ' + ' '.join(f'{v + 1}/{l + 1}' for v, l in zip(triangle.vertices, triangle.loops)) + '\n')
    parts.append(obj.name)
    obj.to_mesh_clear()
metadata = dict(model='potted_plant_04', pot_diameter_m=.06,
                pot_height_m=(high.z - low.z) * scale, scale=scale,
                source_pot_bounds_m=[list(low), list(high)], parts=parts)
(ASSETS / 'model.json').write_text(json.dumps(metadata, indent=2) + '\n')
image = bpy.data.images.load(str(ASSETS / 'textures' / 'potted_plant_04_diff_4k.jpg'), check_existing=False)
image.filepath_raw = str(ASSETS / 'textures' / 'potted_plant_04_diff_4k.png')
image.file_format = 'PNG'
image.save()
print(metadata)

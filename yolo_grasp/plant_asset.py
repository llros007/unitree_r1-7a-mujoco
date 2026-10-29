# SPDX-License-Identifier: AGPL-3.0-or-later
"""Shared 60 mm potted_plant_04 asset baked from the user's downloaded Blender ZIP."""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).resolve().parent / 'assets' / 'plant04'
METADATA = json.loads((ASSETS / 'model.json').read_text())
POT_RADIUS = METADATA['pot_diameter_m'] / 2
POT_HEIGHT = METADATA['pot_height_m']
SCALE = METADATA['scale']


def replace_cube(root, scene_dir=None):
    """Keep legacy body/joint names for the shared dynamics and contact API."""
    body = root.find(".//body[@name='red_cube']")
    body.set('pos', f'0.12 0.25 {0.60 + POT_HEIGHT / 2 + .001}')
    body.remove(body.find('geom'))
    mass = .08
    inertial = body.find('inertial')
    inertial.set('mass', str(mass))
    transverse = mass * (3 * POT_RADIUS**2 + POT_HEIGHT**2) / 12
    axial = mass * POT_RADIUS**2 / 2
    inertial.set('diaginertia', f'{transverse} {transverse} {axial}')
    asset = root.find('asset')
    scene_dir = Path(scene_dir) if scene_dir is not None else ASSETS.parent
    mesh_dir = scene_dir / root.find('compiler').get('meshdir', '')
    texture = ASSETS / 'textures' / 'potted_plant_04_diff_4k.png'
    ET.SubElement(asset, 'texture', name='plant04_tex', type='2d',
                  file=os.path.relpath(texture, scene_dir))
    ET.SubElement(asset, 'material', name='plant04_mat', texture='plant04_tex',
                  texuniform='false', specular='.1', shininess='.1')
    for name in METADATA['parts']:
        ET.SubElement(asset, 'mesh', name=name,
                      file=os.path.relpath(ASSETS / (name + '.obj'), mesh_dir), inertia='shell')
        physical = name == 'potted_plant_04_pot'
        ET.SubElement(body, 'geom', name='cube' if physical else name,
                      type='mesh', mesh=name, material='plant04_mat',
                      contype='1' if physical else '0', conaffinity='7' if physical else '0',
                      friction='2.5 .03 .002', condim='4')

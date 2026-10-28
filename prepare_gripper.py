"""Extract XLeRobot's SO101 PLA holders and TPU95A fin from its print project.

The generated meshes preserve the source geometry at 1:1 scale (mm -> m),
with rigid coordinate transforms for the R1 simulation adapter.
"""
import argparse
import hashlib
from pathlib import Path
import shutil
import struct
import subprocess
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import numpy as np

ROOT = Path(__file__).resolve().parent
DEST = ROOT / 'assets' / 'gripper'
DEFAULT_SOURCE = ROOT.parent / 'XLeRobot'


def write_stl(path, vertices, faces):
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-15)
    records = np.zeros(len(faces), dtype=[('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    records['normal'], records['vertices'] = normals, triangles
    path.write_bytes(b'XLeRobot SO101 mesh; transformed millimeters to meters'.ljust(80, b' ') + struct.pack('<I', len(faces)) + records.tobytes())


def prepare(source=DEFAULT_SOURCE):
    project = source / 'hardware' / 'XLeRobot_0_3_0.3mf'
    with ZipFile(project) as archive:
        document = ET.fromstring(archive.read('3D/3dmodel.model'))
    objects = {o.get('id'): o for o in document.findall('.//{*}object')}
    DEST.mkdir(parents=True, exist_ok=True)
    # Each transform is a proper rigid rotation, followed by a translation in m.
    swap = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.]])
    fixed_rotation = np.array([[0, 0, -1.], [1., 0, 0], [0, -1., 0]])
    specs = [
        ('fixed_holder', '47', fixed_rotation, [0, 0, .03260]),
        ('moving_holder', '39', swap, [0, 0, .012]),
        ('fixed_fin', '49', swap, [-.032, 0, .09286]),
        ('moving_fin', '49', np.diag([-1., -1., 1.]) @ swap, [.006, 0, .06286]),
    ]
    for name, oid, rotation, offset in specs:
        obj = objects[oid]
        vertices = np.array([[float(v.get(k)) for k in ['x', 'y', 'z']] for v in obj.findall('.//{*}vertex')])
        faces = np.array([[int(t.get(k)) for k in ['v1', 'v2', 'v3']] for t in obj.findall('.//{*}triangle')])
        vertices = vertices * .001 @ rotation.T + offset
        write_stl(DEST / f'{name}.stl', vertices, faces)
    shutil.copyfile(source / 'LICENSE', DEST / 'LICENSE')
    commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    (DEST / 'SOURCE.md').write_text(
        '# XLeRobot SO101 TPU95A Fin Ray gripper assets\n\n'
        f'Source: https://github.com/Vector-Wangel/XLeRobot/tree/{commit}/hardware\n\n'
        f'Local source commit: `{commit}`\n\n'
        'Project: `hardware/XLeRobot_0_3_0.3mf`, mesh objects 47 (fixed PLA holder), '
        '39 (moving PLA holder), 49 (TPU Fin Ray finger).\n\n'
        f'Source SHA256: `{hashlib.sha256(project.read_bytes()).hexdigest()}`\n\n'
        'Modified: extracted triangle meshes, converted mm to m, and applied rigid rotations/translations '
        'for a simulation-only R1 flange adapter. No scaling of the physical dimensions. '
        'The two fin meshes are two placements of the same source finger.\n\n'
        'Motion uses one fixed finger and one servo-driven rotary jaw. The adapter, '
        'TCP, collision proxies, masses and contact parameters are simulation approximations, '
        'not a manufacturer-provided R1 mounting design or measured TPU material model. '
        'Contact compliance does not simulate Fin Ray rib deformation.\n\n'
        'License: Apache-2.0, see LICENSE.\n', encoding='utf-8')
    print(DEST)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    prepare(parser.parse_args().source)

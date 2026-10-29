"""Shared plant/cube scene, XLeRobot gripper and D435 rig for both entry points."""
import json
from pathlib import Path
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parent
CAMERA_NAME = 'd435_rgb'
FOVY = 42.5
MIN_DEPTH = .175
MAX_DEPTH = 3.0


def build_scene(config_path=None, object_kind='plant', include_camera=True):
    if object_kind not in ('plant', 'cube'):
        raise ValueError(f'Unknown object: {object_kind}')
    if not (ROOT / 'assets/scene.xml').exists():
        from build_scene import build_scene as build_base_scene
        build_base_scene()
    tree = ET.parse(ROOT / 'assets/scene.xml')
    root = tree.getroot()
    # Refine dry friction to suppress solver creep in a sustained soft-pad
    # grasp. This remains ordinary unilateral contact, without attachment.
    root.find('option').set('noslip_iterations', '5')
    root.set('model', f'R1-7A shared SO101 {object_kind} scene')
    hand = root.find(".//body[@name='hand']")
    for name, pos, size in [('plate', '0 -0.028 0.016', '0.027 0.004 0.027'),
                            ('riser', '0 -0.038 0.008', '0.027 0.008 0.004')]:
        ET.SubElement(hand, 'geom', name=f'd435_mount_{name}', type='box', pos=pos, size=size,
                      rgba='0.68 0.82 0.18 1', contype='0', conaffinity='0')
    rig = ET.SubElement(hand, 'body', name='d435_camera', pos='0 -0.050 0.018', euler='-0.261799 0 0')
    ET.SubElement(rig, 'inertial', pos='0 0 0', mass='0.09', diaginertia='0.000010 0.000066 0.000066')
    ET.SubElement(rig, 'geom', name='d435_case', type='box', size='.045 .0125 .0125',
                  rgba='.72 .74 .77 1', contype='2', conaffinity='1')
    ET.SubElement(rig, 'geom', name='d435_front', type='box', pos='0 0 .0126',
                  size='.042 .010 .0006', rgba='.025 .028 .032 1', contype='0', conaffinity='0')
    for name, x, radius in [('left_ir', -.025, .0045), ('projector', -.008, .005),
                             ('right_ir', .025, .0045), ('rgb', .035, .0035)]:
        ET.SubElement(rig, 'geom', name=f'd435_{name}_lens', type='cylinder', pos=f'{x} 0 .0134',
                      size=f'{radius} .0005', rgba='.035 .12 .17 1', contype='0', conaffinity='0')
    ET.SubElement(rig, 'camera', name=CAMERA_NAME, pos='.035 0 .014', xyaxes='1 0 0 0 -1 0', fovy=str(FOVY), ipd='0')
    path = Path(config_path) if config_path is not None else ROOT / 'end_effector.json'
    parameters = json.loads(path.read_text())
    if set(parameters) != {'hand', 'moving_jaw', 'd435_camera'}:
        raise ValueError('End-effector config requires hand, moving_jaw and d435_camera')
    for name, values in parameters.items():
        if set(values) - {'mass_kg', 'com_m', 'diaginertia_kg_m2'}:
            raise ValueError(f'Unknown end-effector parameters for {name}')
        inertial = root.find(f".//body[@name='{name}']/inertial")
        mass = float(values['mass_kg'])
        com = np.asarray(values['com_m'], dtype=float)
        if not np.isfinite(mass) or mass <= 0 or com.shape != (3,) or not np.isfinite(com).all():
            raise ValueError(f'{name} requires positive mass and a finite three-value COM')
        default_inertia = np.fromstring(inertial.get('diaginertia'), sep=' ') * mass / float(inertial.get('mass'))
        inertia = np.asarray(values.get('diaginertia_kg_m2', default_inertia), dtype=float)
        if inertia.shape != (3,) or not np.isfinite(inertia).all() or np.any(inertia <= 0):
            raise ValueError(f'{name} requires positive diagonal inertia')
        inertial.set('mass', str(mass))
        inertial.set('pos', ' '.join(map(str, com)))
        inertial.set('diaginertia', ' '.join(map(str, inertia)))
    if not include_camera:
        hand.remove(rig)
        for name in ['d435_mount_plate', 'd435_mount_riser']:
            hand.remove(hand.find(f"geom[@name='{name}']"))
    if object_kind == 'plant':
        from yolo_grasp.plant_asset import replace_cube
        replace_cube(root, ROOT / 'assets')
    suffix = '' if include_camera else '_no_camera'
    output = ROOT / 'assets' / f'scene_shared_{object_kind}{suffix}.xml'
    ET.indent(root, space='  ')
    # Both applications may launch together; never expose a half-written XML.
    with NamedTemporaryFile(dir=output.parent, suffix='.xml', delete=False) as temporary:
        tree.write(temporary, encoding='utf-8', xml_declaration=True)
    Path(temporary.name).replace(output)
    return output

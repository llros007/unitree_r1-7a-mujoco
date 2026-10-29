"""Generate a standalone MJCF scene from the sibling SDK's original URDF."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
SCENE = ROOT / 'assets' / 'scene.xml'


def build_scene():
    sources = list((ROOT.parent / 'r1-7a_sdk' / 'urdf').glob('**/R1_Robotic_Arm.urdf'))
    if not sources:
        raise FileNotFoundError('Cannot find r1-7a_sdk/urdf/**/R1_Robotic_Arm.urdf')
    source = sources[0]
    robot = ET.parse(source).getroot()
    meshes = SCENE.parent / 'meshes'
    meshes.mkdir(parents=True, exist_ok=True)
    root = ET.Element('mujoco', model='R1-7A keyboard grasp')
    ET.SubElement(root, 'compiler', angle='radian', meshdir='meshes', inertiafromgeom='false')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast', cone='elliptic', iterations='80', gravity='0 0 -9.81')
    ET.SubElement(root, 'size', njmax='2000', nconmax='400')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='900')
    ET.SubElement(visual, 'headlight', ambient='0.4 0.4 0.4', diffuse='0.7 0.7 0.7')
    defaults = ET.SubElement(root, 'default')
    ET.SubElement(defaults, 'joint', damping='0.1', armature='0.015')
    ET.SubElement(defaults, 'geom', friction='1.2 0.01 0.001', solref='0.006 1', solimp='0.95 0.99 0.001', condim='4')
    assets = ET.SubElement(root, 'asset')
    ET.SubElement(assets, 'texture', name='grid', type='2d', builtin='checker', rgb1='0.12 0.16 0.22', rgb2='0.18 0.23 0.30', width='512', height='512')
    ET.SubElement(assets, 'material', name='floor_mat', texture='grid', texrepeat='6 6', reflectance='0.1')
    # Visual approximation of the silver housing in Unitree's R1-ARM photos:
    # https://www.unitree.com/R1-ARM/ (no official numeric color is published).
    ET.SubElement(assets, 'material', name='arm_silver', rgba='0.70 0.73 0.78 1', specular='0.45', shininess='0.5', reflectance='0.12')
    ET.SubElement(assets, 'material', name='arm_interface', rgba='0.10 0.11 0.13 1', specular='0.25', shininess='0.3')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -0.5 2.5', dir='0 0 -1', diffuse='0.8 0.8 0.8', castshadow='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='2 2 0.1', material='floor_mat', conaffinity='7')
    ET.SubElement(world, 'geom', name='table', type='box', pos='0 0.15 0.565', size='0.50 0.45 0.035', rgba='0.36 0.43 0.51 1', conaffinity='7')
    for x in [-0.42, 0.42]:
        for y in [-0.22, 0.52]:
            ET.SubElement(world, 'geom', type='box', pos=f'{x} {y} 0.265', size='0.025 0.025 0.265', rgba='0.15 0.19 0.24 1', conaffinity='7')
    ET.SubElement(world, 'geom', name='base_pedestal', type='cylinder', pos='0 0 0.625', size='0.083 0.025', rgba='0.12 0.17 0.23 1', conaffinity='7')
    links = {link.get('name'): link for link in robot.findall('link')}
    bodies = {'base_link': ET.SubElement(world, 'body', name='base_link', pos='0 0 0.676')}
    for j in robot.findall('joint'):
        child = j.find('child').get('link')
        origin, limit = j.find('origin'), j.find('limit')
        body = ET.SubElement(bodies[j.find('parent').get('link')], 'body', name=child, pos=origin.get('xyz'), euler=origin.get('rpy'))
        ET.SubElement(body, 'joint', name=j.get('name'), type='hinge', axis=j.find('axis').get('xyz'), range=f"{limit.get('lower')} {limit.get('upper')}")
        bodies[child] = body
    for name, body in bodies.items():
        link = links[name]
        inertia = link.find('inertial')
        i = inertia.find('inertia')
        ET.SubElement(body, 'inertial', pos=inertia.find('origin').get('xyz'), mass=inertia.find('mass').get('value'), fullinertia=' '.join(i.get(k) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']))
        mesh_file = Path(link.find('visual/geometry/mesh').get('filename')).name
        shutil.copyfile(source.parent.parent / 'meshes' / mesh_file, meshes / mesh_file)
        ET.SubElement(assets, 'mesh', name=name, file=mesh_file)
        material = 'arm_interface' if name == 'Link7' else 'arm_silver'
        ET.SubElement(body, 'geom', name=f'{name}_mesh', type='mesh', mesh=name, material=material, contype='2', conaffinity='1')
    cube = ET.SubElement(world, 'body', name='red_cube', pos='0.12 0.25 0.621')
    ET.SubElement(cube, 'freejoint', name='cube_free')
    ET.SubElement(cube, 'inertial', pos='0 0 0', mass='0.055', diaginertia='0.000014667 0.000014667 0.000014667')
    ET.SubElement(cube, 'geom', name='cube', type='box', size='0.02 0.02 0.02', mass='0.055', rgba='0.9 0.045 0.035 1', friction='2.5 0.03 0.002', conaffinity='7')
    ET.SubElement(world, 'site', name='drop_zone', type='box', pos='-0.12 0.25 0.601', size='0.06 0.06 0.001', rgba='0.1 0.8 0.55 0.4')
    target = ET.SubElement(world, 'body', name='target', mocap='true', pos='0.12 0.25 0.79')
    ET.SubElement(target, 'site', name='target_site', type='sphere', size='0.008', rgba='1 0.75 0.15 0.5')
    actuators = ET.SubElement(root, 'actuator')
    for j in robot.findall('joint'):
        limit = j.find('limit')
        effort = limit.get('effort')
        ET.SubElement(actuators, 'motor', name=f"{j.get('name')}_motor", joint=j.get('name'), gear='1', ctrllimited='true', forcelimited='true', ctrlrange=f'-{effort} {effort}', forcerange=f'-{effort} {effort}')
    from xle_gripper import add_gripper
    add_gripper(bodies['Link7'], assets, actuators)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(SCENE, encoding='utf-8', xml_declaration=True)
    shutil.copyfile(ROOT.parent / 'r1-7a_sdk' / 'LICENSE', SCENE.parent / 'SDK_LICENSE')
    return SCENE


if __name__ == '__main__':
    print(build_scene())

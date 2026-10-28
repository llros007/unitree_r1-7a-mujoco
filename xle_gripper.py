"""SO101 single rotary jaw with official XLeRobot TPU95A Fin Ray meshes.

TPU is approximated by compliant contacts on segmented collision proxies.
The mesh itself remains rigid; this is not a calibrated deformable solid.
"""
import xml.etree.ElementTree as ET

OPEN_ANGLE = 0.8
CLOSE_ANGLE = -0.13
TCP_HEIGHT = 0.106


def add_gripper(link7, assets, actuators):
    for name in ['fixed_holder', 'moving_holder', 'fixed_fin', 'moving_fin']:
        ET.SubElement(assets, 'mesh', name=f'xle_{name}', file=f'../gripper/{name}.stl')
    hand = ET.SubElement(link7, 'body', name='hand', pos='0 0 0.04', gravcomp='1')
    ET.SubElement(hand, 'inertial', pos='0 0 0.032', mass='0.14', diaginertia='0.00012 0.00012 0.00010')
    ET.SubElement(hand, 'geom', name='flange_adapter', type='cylinder', size='0.028 0.008', pos='0 0 -0.005', rgba='0.22 0.27 0.34 1', contype='2', conaffinity='1')
    ET.SubElement(hand, 'geom', name='fixed_holder_visual', type='mesh', mesh='xle_fixed_holder', rgba='0.18 0.22 0.28 1', contype='0', conaffinity='0')
    ET.SubElement(hand, 'geom', name='fixed_fin_visual', type='mesh', mesh='xle_fixed_fin', rgba='0.88 0.92 0.96 1', contype='0', conaffinity='0')
    ET.SubElement(hand, 'geom', name='servo_case', type='box', size='0.014 0.021 0.018', pos='0 0 0.022', rgba='0.1 0.11 0.14 1', contype='2', conaffinity='1')
    ET.SubElement(hand, 'site', name='tcp', pos=f'0 0 {TCP_HEIGHT}', size='0.004', rgba='0.2 1 0.5 1')
    jaw = ET.SubElement(hand, 'body', name='moving_jaw', pos='0.026 0 0.03', gravcomp='1')
    ET.SubElement(jaw, 'joint', name='gripper_hinge', type='hinge', axis='0 1 0', range='-0.20 1.05', damping='0.08', armature='0.0005')
    ET.SubElement(jaw, 'inertial', pos='0.005 0 0.05', mass='0.035', diaginertia='0.000025 0.000025 0.000005')
    ET.SubElement(jaw, 'geom', name='moving_holder_visual', type='mesh', mesh='xle_moving_holder', rgba='0.18 0.22 0.28 1', contype='0', conaffinity='0')
    ET.SubElement(jaw, 'geom', name='moving_fin_visual', type='mesh', mesh='xle_moving_fin', rgba='0.88 0.92 0.96 1', contype='0', conaffinity='0')
    # Thin pads follow the flat inner surface, rather than filling the lattice
    # with a single convex hull. Four sections approximate the contact surface.
    for index, z in enumerate([.075, .089, .103, .117]):
        for side, body, x, height in [('fixed', hand, -.022, z), ('moving', jaw, -.004, z - .03)]:
            ET.SubElement(body, 'geom', name=f'{side}_pad_{index}', type='box', pos=f'{x} 0 {height}', size='0.002 0.011 0.007', rgba='0.8 0.85 0.9 0', group='3', contype='4', conaffinity='1', friction='1.8 0.03 0.002', solref='0.012 1', solimp='0.90 0.98 0.001')
    ET.SubElement(actuators, 'position', name='gripper_servo', joint='gripper_hinge', kp='8', kv='0.3', ctrlrange='-0.20 1.05', forcerange='-0.8 0.8')

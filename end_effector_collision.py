"""Conservative tool/arm self-contact pairs for the custom wrist assembly.

Visual mesh contacts use MuJoCo convex hulls. This is not a full arm planner.
"""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

TOOL_CLEARANCE = .001  # metre, beyond the physical contact surface
CLEARANCE_Q = np.deg2rad([0., 0., 0., 45., 0., 0., 0.])
PAIR_PREFIX = 'tool_arm_'


def add_tool_arm_contacts(root):
    """Explicit pairs bypass old masks without changing object/pad contacts."""
    contact = root.find('contact')
    if contact is None:
        contact = ET.SubElement(root, 'contact')
    hand = root.find(".//body[@name='hand']")
    tools = [g.get('name') for g in hand.iter('geom')]
    arms = ['base_link_mesh'] + [f'Link{i}_mesh' for i in range(1, 8)]
    for tool in tools:
        for arm in arms:
            # Only the fixed Link7 mounting interface is exempt. Adjacent
            # Link5/6 housings must remain clear after correcting the flange.
            if arm == 'Link7_mesh':
                continue
            ET.SubElement(contact, 'pair', name=f'{PAIR_PREFIX}{tool}_{arm}',
                          geom1=tool, geom2=arm, condim='3', margin='0',
                          friction='.6 .6 .005 .0001 .0001')


class ToolArmClearance:
    def __init__(self, model):
        self.model = model
        ids = [i for i in range(model.npair)
               if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_PAIR, i) or '').startswith(PAIR_PREFIX)]
        self.pairs = [(int(model.pair_geom1[i]), int(model.pair_geom2[i])) for i in ids]

    def violation(self, data, clearance=TOOL_CLEARANCE):
        for a, b in self.pairs:
            distance = mujoco.mj_geomDistance(self.model, data, a, b, clearance, None)
            if distance < clearance - 1e-9:
                return self.model.geom(a).name, self.model.geom(b).name, float(distance)
        return None

"""Desktop comparison regression with a synthetic feedback bias; never opens hardware."""
import json
import time
from pathlib import Path
from unittest.mock import patch

import glfw
import mujoco
import numpy as np
from PIL import Image

from auto_joint_test import SimulationBus, parse_args
from joint_test_viewer import ComparisonViewer, run_viewer


def main():
    output = Path('outputs/joint_compare_gui_' + time.strftime('%Y%m%d_%H%M%S'))
    args = parse_args(['--view', '--cycles', '1', '--move-seconds', '1', '--hold-seconds', '.5',
                       '--output', str(output)])
    assert not args.hardware
    rendered, independent, screenshot = [False], [False], [False]
    original_render = ComparisonViewer.render
    original_swap = glfw.swap_buffers
    began = time.monotonic()

    class BiasedBus(SimulationBus):
        def send(self, *command):
            super().send(*command)
            self.q[2] += .001  # Synthetic sensor bias ONLY; never injected into the independent plant.

    def render(viewer):
        if viewer.latest is not None:
            rendered[0] = True
            independent[0] |= abs(viewer.latest['q'][2] - viewer.latest['sim_q'][2]) > .0009
        original_render(viewer)
        if viewer.test_status.startswith('FINISHED'):
            glfw.set_window_should_close(viewer.window, True)
        if time.monotonic() - began > 30:
            raise TimeoutError('viewer test exceeded 30s')

    def swap(window):
        if output.exists() and rendered[0] and independent[0] and not screenshot[0]:
            w, h = glfw.get_framebuffer_size(window)
            rgb = np.zeros((h, w, 3), dtype=np.uint8)
            mujoco.mjr_readPixels(rgb, None, mujoco.MjrRect(0, 0, w, h), viewer_context[0])
            Image.fromarray(rgb[::-1]).save(output / 'comparison.png')
            screenshot[0] = True
        original_swap(window)

    viewer_context = [None]
    original_init = ComparisonViewer.__init__
    def initialize(viewer, *values):
        original_init(viewer, *values)
        viewer_context[0] = viewer.context

    with patch('auto_joint_test.SimulationBus', BiasedBus), \
            patch.object(ComparisonViewer, '__init__', initialize), \
            patch.object(ComparisonViewer, 'render', render), patch('glfw.swap_buffers', swap):
        code = run_viewer(args)
    assert code == 0, code
    assert all((rendered[0], independent[0], screenshot[0]))
    analysis = json.loads((output / 'analysis.json').read_text())
    assert .05 < analysis['real_minus_sim']['rmse_deg'][2] < .07
    print('PASS: visible independent comparison, biased feedback isolation, report and clean close')
    print(output.resolve())


if __name__ == '__main__':
    main()

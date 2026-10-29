# SPDX-License-Identifier: AGPL-3.0-or-later
"""Wrist search, measured settling and fresh RGB-D localization before picking."""
import numpy as np
from scipy.spatial.transform import Rotation

from motion import PickPlace


class SearchPickPlace(PickPlace):
    def __init__(self, env, probe, localize, speed=.5):
        super().__init__(env, speed)
        self.probe, self.localize = probe, localize
        # Overlapping oblique views over the calibrated tabletop pick workspace.
        self.views = [np.array([x, y, .86]) for y in (.08, .16)
                      for x in (.12, .04, .20)]
        self.search_log = []

    def begin_search(self):
        if self.active:
            return
        self.cancel()
        self.env.gripper_open = True
        self.search_log = []
        self.view_index = 0
        self._next_view()

    def _next_view(self):
        if self.view_index >= len(self.views):
            self.fail('Search exhausted: no stable, valid RGB-D pot localization')
            self.report['search'] = self.search_log
            return
        self.phase = 'scan_move'
        self.elapsed = self.stable_time = 0.
        self.start_position = self.env.target_position.copy()
        self.start_rotation = self.env.target_rotation.copy()
        self.duration = 2. / self.speed
        self.destination = self.views[self.view_index]
        self.rotation_delta = Rotation.from_matrix(self.env.home_rotation @ self.start_rotation.T).as_rotvec()
        self.view_index += 1
        print(f'[search] observation {self.view_index}/{len(self.views)}', flush=True)

    def tick(self, dt=.01):
        if not self.phase.startswith('scan_'):
            super().tick(dt)
            return
        if self.env.paused:
            return
        self.elapsed += dt
        if self.phase == 'scan_move':
            t = min(1., self.elapsed / self.duration)
            blend = t * t * (3 - 2 * t)
            point = self.start_position + blend * (self.destination - self.start_position)
            rotation = Rotation.from_rotvec(blend * self.rotation_delta).as_matrix() @ self.start_rotation
            self.env.move_target(point - self.env.target_position,
                                 Rotation.from_matrix(rotation @ self.env.target_rotation.T).as_rotvec())
            if self.elapsed >= self.duration:
                self.phase, self.elapsed = 'scan_settle', 0.
            return
        # Hold a fixed target and require 0.5 s of low measured joint velocity,
        # position and orientation error. A timer alone does not prove settling.
        position_error = np.linalg.norm(self.env.tcp_position - self.env.target_position)
        rotation_error = np.linalg.norm(Rotation.from_matrix(
            self.env.target_rotation @ self.env.data.site_xmat[self.env.tcp_id].reshape(3, 3).T).as_rotvec())
        settled = (np.max(np.abs(self.env.data.qvel[self.env.dofs])) < .025
                   and position_error < .003 and rotation_error < .03)
        self.stable_time = self.stable_time + dt if settled else 0.
        if self.stable_time < .5:
            if self.elapsed > 4.:
                self.search_log.append(dict(view=self.view_index, event='settle_timeout'))
                self._next_view()
            return
        if self.phase == 'scan_settle':
            # Search detection is discarded; it never supplies grasp coordinates.
            found = self.probe()
            self.search_log.append(dict(view=self.view_index, event='detected' if found else 'empty',
                                        timestamp_s=float(self.env.data.time)))
            if not found:
                self._next_view()
                return
            self.phase, self.elapsed, self.stable_time = 'scan_reacquire', 0., 0.
            return
        try:
            grasp = self.localize()  # New RGB/depth/extrinsics after another stable hold.
        except ValueError as exc:
            self.search_log.append(dict(view=self.view_index, event='localization_rejected', reason=str(exc)))
            self._next_view()
            return
        self.search_log.append(dict(view=self.view_index, event='localized',
                                    timestamp_s=float(self.env.data.time)))
        self.phase = 'idle'
        super().start(grasp)
        self.report['search'] = self.search_log.copy()

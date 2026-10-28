#!/usr/bin/env python3
"""Keyboard teleoperation and headless verification for the R1-7A scene."""
import argparse
import json
from pathlib import Path
import time

import glfw
import mujoco
import numpy as np

from environment import ArmEnvironment

HELP = """
R1-7A + XLeRobot SO101 TPU95A MuJoCo 键盘抓取环境
  W/S: 世界坐标 Y +/-     A/D: 世界坐标 X -/+
  Q/E: 世界坐标 Z +/-     Shift: 慢速微调
  I/K: 绕 X 轴旋转       J/L: 绕 Y 轴旋转       U/O: 绕 Z 轴旋转
  Space: 夹爪开/合       R: 重置场景          P: 暂停/继续
  H: 显示帮助            Esc: 退出
  鼠标左键拖动: 旋转视角   右键拖动: 平移视角   滚轮: 缩放
抓取: 移到红色方块正上方 -> E 下降到方块中心 -> Space 合拢 -> Q 抬起。
默认位置已对准方块；绿色区域可用于放置。所有距离单位为米。
"""


class KeyboardViewer:
    def __init__(self, env):
        self.env = env
        if not glfw.init():
            raise RuntimeError('GLFW 初始化失败，请在桌面会话运行，或使用 --headless --demo')
        self.window = glfw.create_window(1280, 900, 'R1-7A + SO101 TPU95A | Keyboard Cartesian Control', None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError('无法创建 OpenGL 窗口，请检查 DISPLAY 和显卡驱动')
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0., 0.16, 0.76]
        self.camera.distance = 1.35
        self.camera.azimuth = 135
        self.camera.elevation = -24
        self.option = mujoco.MjvOption()
        self.scene = mujoco.MjvScene(env.model, maxgeom=2000)
        self.context = mujoco.MjrContext(env.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.keys = set()
        self.mouse_pos = glfw.get_cursor_pos(self.window)
        glfw.set_key_callback(self.window, self.on_key)
        glfw.set_cursor_pos_callback(self.window, self.on_mouse)
        glfw.set_scroll_callback(self.window, self.on_scroll)
        glfw.set_window_focus_callback(self.window, lambda window, focused: self.keys.clear() if not focused else None)

    def on_key(self, window, key, scancode, action, mods):
        if action == glfw.RELEASE:
            self.keys.discard(key)
            return
        self.keys.add(key)
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
        elif key == glfw.KEY_SPACE:
            self.env.gripper_open = not self.env.gripper_open
        elif key == glfw.KEY_R:
            self.env.reset()
            self.keys.clear()
        elif key == glfw.KEY_P:
            self.env.paused = not self.env.paused
        elif key == glfw.KEY_H:
            print(HELP, flush=True)

    def on_mouse(self, window, x, y):
        dx, dy = x - self.mouse_pos[0], y - self.mouse_pos[1]
        self.mouse_pos = (x, y)
        height = max(glfw.get_window_size(window)[1], 1)
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        else:
            return
        mujoco.mjv_moveCamera(self.env.model, action, dx / height, dy / height, self.scene, self.camera)

    def on_scroll(self, window, x, y):
        mujoco.mjv_moveCamera(self.env.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0., -0.05 * y, self.scene, self.camera)

    def controls(self, dt):
        if self.env.paused:
            return
        key = lambda k: float(k in self.keys)
        slow = glfw.KEY_LEFT_SHIFT in self.keys or glfw.KEY_RIGHT_SHIFT in self.keys
        speed = 0.025 if slow else 0.10
        translation = np.array([key(glfw.KEY_D) - key(glfw.KEY_A), key(glfw.KEY_W) - key(glfw.KEY_S), key(glfw.KEY_Q) - key(glfw.KEY_E)])
        rotation = np.array([key(glfw.KEY_I) - key(glfw.KEY_K), key(glfw.KEY_J) - key(glfw.KEY_L), key(glfw.KEY_U) - key(glfw.KEY_O)])
        if np.any(translation) or np.any(rotation):
            self.env.move_target(translation * speed * dt, rotation * (0.25 if slow else 0.75) * dt)

    def render(self):
        width, height = glfw.get_framebuffer_size(self.window)
        if not width or not height:
            return
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(self.env.model, self.env.data, self.option, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)
        pos, cube = self.env.tcp_position, self.env.cube_position
        grip = 'OPEN' if self.env.gripper_open else 'CLOSED'
        status = 'PAUSED' if self.env.paused else ('CUBE LIFTED!' if self.env.success else 'RUNNING')
        labels = 'R1-7A + SO101 TPU95A\nState\nTCP xyz (m)\nTarget xyz (m)\nCube xyz (m)\nGripper\nIK residual'
        angle = np.degrees(self.env.data.qpos[self.env.gripper_q])
        values = f'\n{status}\n{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:.3f}\n' + ' '.join(f'{v:+.3f}' for v in self.env.target_position) + f'\n{cube[0]:+.3f} {cube[1]:+.3f} {cube[2]:.3f}\n{grip} {angle:.1f} deg | contacts {len(self.env.contact_sides())}/2\n{self.env.ik_error:.4f}'
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport, labels, values, self.context)
        help_text = 'MOVE  W/S: Y +/-   A/D: X -/+   Q/E: Z +/-   SHIFT: fine\nROTATE  I/K: X   J/L: Y   U/O: Z\nSPACE: gripper   R: reset   P: pause   ESC: exit\nMouse: left orbit | right pan | wheel zoom\nPinch red cube, lift, then release over green zone.'
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, viewport, help_text, '', self.context)
        glfw.swap_buffers(self.window)

    def run(self, seconds=None):
        print(HELP, flush=True)
        start = previous = time.monotonic()
        accumulator = 0.
        try:
            while not glfw.window_should_close(self.window):
                now = time.monotonic()
                if seconds is not None and now - start > seconds:
                    break
                # Fixed 100 Hz controller and 500 Hz physics, independent of rendering.
                accumulator += min(now - previous, 0.1)
                previous = now
                glfw.poll_events()
                while accumulator >= 0.01:
                    self.controls(0.01)
                    self.env.step(0.01)
                    accumulator -= 0.01
                self.render()
                time.sleep(0.001)
        finally:
            self.context.free()
            glfw.destroy_window(self.window)
            glfw.terminate()


def snapshot(env, path):
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0., 0.16, 0.76]
    camera.distance = 1.35
    camera.azimuth = 135
    camera.elevation = -24
    from PIL import Image
    with mujoco.Renderer(env.model, height=900, width=1280) as renderer:
        renderer.update_scene(env.data, camera=camera)
        Image.fromarray(renderer.render()).save(path)


def main():
    parser = argparse.ArgumentParser(description=HELP, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--headless', action='store_true', help='不打开窗口')
    parser.add_argument('--demo', action='store_true', help='运行物理抓取、搬运、放置验证')
    parser.add_argument('--snapshot', type=Path, help='保存场景 PNG（需要 Pillow）')
    parser.add_argument('--seconds', type=float, default=None, help='窗口自动关闭时间，便于冒烟测试')
    args = parser.parse_args()
    env = ArmEnvironment()
    if args.demo:
        def observe(phase):
            print(f'{phase:10s} TCP={env.tcp_position.round(4)} cube={env.cube_position.round(4)} contacts={len(env.contact_sides())}', flush=True)
            if args.snapshot and phase == 'lift':
                args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                snapshot(env, args.snapshot)
        result = env.run_demo(observe)
        print(json.dumps(result, indent=2))
        if not result['passed']:
            raise SystemExit(1)
    elif args.headless:
        for _ in range(200):
            env.step()
        print(f'Model OK: nq={env.model.nq}, nv={env.model.nv}, nu={env.model.nu}; TCP={env.tcp_position.round(4)}')
    if args.snapshot and not args.demo:
        args.snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot(env, args.snapshot)
    if not args.headless:
        KeyboardViewer(env).run(args.seconds)


if __name__ == '__main__':
    main()

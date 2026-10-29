"""Offline round trip through the sibling SDK's IM6014 command representation.

Source: ArmMotorBus coordinate transforms and MotorProtocol::buildControlPacket.
This module never opens a serial port. Ratios follow the SDK's default IDs 0..6.
"""
from pathlib import Path

import numpy as np


class SdkProtocol:
    ratios = np.array([32., 32., 38. / 3., 38. / 3., 38. / 3., 16., 16.])

    def __init__(self, calibration_path):
        self.path = Path(calibration_path).resolve()
        calibration = np.loadtxt(self.path, comments='#')
        if calibration.shape != (7, 4) or not np.isfinite(calibration).all():
            raise ValueError('SDK calibration needs seven rows: joint dir scale offset')
        calibration = calibration[np.argsort(calibration[:, 0])]
        if not np.array_equal(calibration[:, 0], np.arange(7)):
            raise ValueError('SDK calibration joint indices must be 0..6')
        if not np.isin(calibration[:, 1], [-1, 1]).all() or np.any(calibration[:, 2] <= 0):
            raise ValueError('SDK calibration requires dir +/-1 and positive scale')
        # The bus API and its arithmetic use float, before double ratio math.
        self.axis = (calibration[:, 1] * calibration[:, 2]).astype(np.float32)
        self.offset = calibration[:, 3].astype(np.float32)
        self.steps = np.array([2 * np.pi / (32768 * self.ratios),
                               2 * np.pi / (64 * self.ratios),
                               self.ratios / 2560,
                               self.ratios**2 / 12800,
                               self.ratios**2 / 51200])
        self.raw = None

    def round_trip(self, q, dq, tau, kp, kd):
        q, dq, tau, kp, kd = [np.asarray(x, dtype=np.float32) for x in (q, dq, tau, kp, kd)]
        motor_values = np.array([
            np.clip((q - self.offset) / self.axis, np.float32(-32508.539391), np.float32(32508.539376)),
            np.clip(dq / self.axis, np.float32(-253.972964), np.float32(253.965213)),
            np.clip(tau / self.axis, np.float32(-162.133333), np.float32(162.128385)),
            np.clip(kp, 0, np.float32(410.725243)),
            np.clip(kd, 0, np.float32(102.681311)),
        ], dtype=float)
        scaled = motor_values / self.steps
        # std::round ties away from zero; np.round uses ties to even.
        raw = np.copysign(np.floor(np.abs(scaled) + .5), scaled).astype(np.int64)
        bounds = np.array([2147483647, 32767, 32767, 32767, 32767])[:, None]
        lower = -bounds - 1
        # SDK casts before clamping. Reject overflow instead of emulating an
        # implementation-dependent narrowing conversion as a physical behavior.
        if np.any(raw > bounds) or np.any(raw < lower):
            raise ValueError('Command overflows SDK wire representation')
        self.raw = raw.copy()
        decoded = raw * self.steps
        decoded[0] = decoded[0] * self.axis + self.offset
        decoded[1:3] *= self.axis
        return tuple(decoded)

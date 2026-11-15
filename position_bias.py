"""Optional bounded hold-position integral; disabled unless explicitly configured."""
import numpy as np


class PositionBias:
    def __init__(self, ki, limit, slew):
        self.ki, self.limit, self.slew = (np.asarray(x, dtype=float).copy() for x in (ki, limit, slew))
        for value in (self.ki, self.limit, self.slew):
            if value.shape != (7,) or not np.isfinite(value).all() or np.any(value < 0):
                raise ValueError('position integral parameters require seven finite nonnegative values')
        if np.any(self.limit > 2.) or np.any(self.slew > .5):
            raise ValueError('position integral is limited to 2 Nm and 0.5 Nm/s')
        active = self.ki > 0
        if np.any(active & ((self.limit <= 0) | (self.slew <= 0))):
            raise ValueError('enabled position integral requires positive limit and slew')
        self.enabled = bool(np.any(active))
        self.bias = np.zeros(7)

    def reset(self):
        self.bias[:] = 0.

    def update(self, error, dt, integrate=True, unwind=False):
        if not self.enabled:
            return self.bias.copy()
        error = np.asarray(error, dtype=float)
        if error.shape != (7,) or not np.isfinite(error).all() or not np.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError('invalid feedback or timestep for position integral')
        unwind = np.broadcast_to(np.asarray(unwind, dtype=bool), (7,))
        if integrate:
            # Ignore very small feedback fluctuations; clip state itself for anti-windup.
            error = np.where(np.abs(error) < np.deg2rad(.02), 0., error)
            increment = np.clip(self.ki * error, -self.slew, self.slew) * dt
            self.bias = np.clip(self.bias + np.where(unwind, 0., increment), -self.limit, self.limit)
        self.bias += np.where(unwind, np.clip(-self.bias, -self.slew*dt, self.slew*dt), 0.)
        return self.bias.copy()

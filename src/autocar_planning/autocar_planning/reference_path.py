import math

import numpy as np


class ReferencePath:
    def __init__(self, xs, ys, yaws):
        self.xs = np.asarray(xs, dtype=float)
        self.ys = np.asarray(ys, dtype=float)
        self.yaws = np.asarray(yaws, dtype=float)

    @property
    def empty(self):
        return len(self.xs) == 0

    @classmethod
    def from_path2d(cls, msg):
        return cls(
            [pose.x for pose in msg.poses],
            [pose.y for pose in msg.poses],
            [pose.theta for pose in msg.poses],
        )

    def offset(self, lateral_offset):
        nx = -np.sin(self.yaws)
        ny = np.cos(self.yaws)
        return (
            self.xs + lateral_offset * nx,
            self.ys + lateral_offset * ny,
            self.yaws.copy(),
        )

    def curvatures(self):
        if len(self.xs) < 3:
            return np.zeros(len(self.xs))

        dx = np.gradient(self.xs)
        dy = np.gradient(self.ys)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        denom = np.power(dx * dx + dy * dy, 1.5)
        denom = np.where(denom < 1e-6, 1e-6, denom)
        return (dx * ddy - dy * ddx) / denom

    def length_remaining_cost(self):
        if len(self.xs) < 2:
            return 0.0
        dx = np.diff(self.xs)
        dy = np.diff(self.ys)
        return float(np.sum(np.hypot(dx, dy)))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

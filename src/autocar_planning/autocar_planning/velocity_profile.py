import numpy as np


def constant_profile(count, target_speed):
    if count <= 0:
        return np.array([], dtype=float)
    return np.ones(count, dtype=float) * float(target_speed)


def acceleration_profile(velocities, ds):
    if len(velocities) < 2:
        return np.zeros(len(velocities), dtype=float)
    dt = max(ds / max(float(np.max(np.abs(velocities))), 0.1), 0.05)
    accel = np.gradient(velocities, dt)
    return accel

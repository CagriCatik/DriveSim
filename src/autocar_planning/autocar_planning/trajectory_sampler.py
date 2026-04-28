from dataclasses import dataclass

import numpy as np

from autocar_planning.collision_checker import collides, min_distance
from autocar_planning.velocity_profile import acceleration_profile, constant_profile


@dataclass
class CandidateTrajectory:
    xs: object
    ys: object
    yaws: object
    velocities: object
    accelerations: object
    curvatures: object
    lateral_offset: float
    speed: float
    cost: float
    collision: bool


def sample_candidates(reference_path, lateral_offsets, target_speeds, objects, collision_radius):
    candidates = []
    curvatures = reference_path.curvatures()

    for lateral_offset in lateral_offsets:
        xs, ys, yaws = reference_path.offset(lateral_offset)
        for speed in target_speeds:
            velocities = constant_profile(len(xs), speed)
            accelerations = acceleration_profile(velocities, ds=0.2)
            collision = collides(xs, ys, objects, collision_radius)
            obstacle_distance = min_distance(xs, ys, objects)

            cost = 0.0
            cost += abs(lateral_offset) * 2.0
            cost += float(np.max(np.abs(curvatures), initial=0.0)) * 5.0
            cost += float(np.max(np.abs(accelerations), initial=0.0)) * 0.5
            if obstacle_distance < 10.0:
                cost += (10.0 - obstacle_distance) * 2.0
            if collision:
                cost += 1e6

            candidates.append(CandidateTrajectory(
                xs=xs,
                ys=ys,
                yaws=yaws,
                velocities=velocities,
                accelerations=accelerations,
                curvatures=curvatures,
                lateral_offset=lateral_offset,
                speed=speed,
                cost=cost,
                collision=collision,
            ))

    return candidates

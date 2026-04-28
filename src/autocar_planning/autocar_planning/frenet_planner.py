from autocar_planning.trajectory_sampler import sample_candidates


class FrenetPlanner:
    def __init__(self, lateral_offsets, speed_factors, collision_radius):
        self.lateral_offsets = lateral_offsets
        self.speed_factors = speed_factors
        self.collision_radius = collision_radius

    def plan(self, reference_path, nominal_speed, objects):
        speeds = [max(0.0, nominal_speed * factor) for factor in self.speed_factors]
        candidates = sample_candidates(
            reference_path,
            self.lateral_offsets,
            speeds,
            objects,
            self.collision_radius,
        )
        feasible = [candidate for candidate in candidates if not candidate.collision]
        if feasible:
            return min(feasible, key=lambda candidate: candidate.cost)
        if candidates:
            return min(candidates, key=lambda candidate: candidate.cost)
        return None

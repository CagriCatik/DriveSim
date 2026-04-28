import math


def object_xy(obj):
    return obj.pose.position.x, obj.pose.position.y


def collides(xs, ys, objects, radius):
    radius_sq = radius * radius
    for obj in objects:
        ox, oy = object_xy(obj)
        for x, y in zip(xs, ys):
            dx = x - ox
            dy = y - oy
            if dx * dx + dy * dy <= radius_sq:
                return True
    return False


def min_distance(xs, ys, objects):
    if not objects:
        return math.inf

    best = math.inf
    for obj in objects:
        ox, oy = object_xy(obj)
        for x, y in zip(xs, ys):
            best = min(best, math.hypot(x - ox, y - oy))
    return best

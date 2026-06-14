import math

import numpy as np


def nearest_path_index(path, pose_xy):
    pts = np.asarray(path, dtype=float)
    dists = np.linalg.norm(pts[:, :2] - np.asarray(pose_xy), axis=1)
    return int(np.argmin(dists))


def lookahead_point(path, pose_xy, lookahead_m):
    if len(path) == 0:
        return None
    start = nearest_path_index(path, pose_xy)
    px, py = pose_xy
    for point in path[start:]:
        if math.hypot(point[0] - px, point[1] - py) >= lookahead_m:
            return point
    return path[-1]


def pure_pursuit_steering(path, pose, lookahead_m, wheelbase_m, reverse=False):
    target = lookahead_point(path, pose[:2], lookahead_m)
    if target is None:
        return 0.0, None

    x, y, yaw = pose
    dx = target[0] - x
    dy = target[1] - y
    local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
    local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
    if reverse:
        local_x = -local_x
        local_y = -local_y

    ld = max(math.hypot(local_x, local_y), 1e-6)
    alpha = math.atan2(local_y, local_x)
    delta = math.atan2(2.0 * wheelbase_m * math.sin(alpha), ld)
    steering = float(np.clip(delta / (math.pi / 4.0), -1.0, 1.0))
    if reverse:
        steering = -steering
    return steering, target


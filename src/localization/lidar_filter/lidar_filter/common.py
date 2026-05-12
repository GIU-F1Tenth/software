import numpy as np


def scan_to_world_points(ranges, angles, pose_x, pose_y, pose_yaw):
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    if ranges.shape != angles.shape:
        raise ValueError(
            f"ranges and angles must have the same shape, got {ranges.shape} vs {angles.shape}"
        )
    finite = np.isfinite(ranges)
    world_angles = pose_yaw + angles
    px = pose_x + ranges * np.cos(world_angles)
    py = pose_y + ranges * np.sin(world_angles)
    return ranges, finite, px, py


def run_pipeline(filters, ranges, angles, pose_x, pose_y, pose_yaw):
    ranges = np.asarray(ranges, dtype=np.float64)
    if not filters:
        return np.ones(ranges.shape, dtype=bool)
    results = [f.filter_scan(ranges, angles, pose_x, pose_y, pose_yaw) for f in filters]
    return np.logical_and.reduce(results)

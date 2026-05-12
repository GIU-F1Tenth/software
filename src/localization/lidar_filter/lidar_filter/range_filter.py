import numpy as np

from lidar_filter.common import scan_to_world_points
from lidar_filter.filter_base import Filter


class RangeFilter(Filter):
    def __init__(self, max_range):
        self.max_range = float(max_range)

    def filter_scan(self, ranges, angles, pose_x, pose_y, pose_yaw) -> np.ndarray:
        ranges = np.asarray(ranges, dtype=np.float64)
        _, finite, px, py = scan_to_world_points(ranges, angles, pose_x, pose_y, pose_yaw)
        dist = np.hypot(px - pose_x, py - pose_y)
        keep = finite & (dist <= self.max_range)
        out = ranges.copy()
        out[~keep] = np.inf
        return out

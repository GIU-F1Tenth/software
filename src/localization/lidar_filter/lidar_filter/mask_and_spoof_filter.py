import json
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

from lidar_filter.common import scan_to_world_points
from lidar_filter.filter_base import Filter


class MaskAndSpoofFilter(Filter):
    def __init__(self, mask, resolution, origin):
        self.mask = np.asarray(mask, dtype=bool)
        if self.mask.ndim != 2:
            raise ValueError(f"mask must be 2D, got shape {self.mask.shape}")
        self.resolution = float(resolution)
        origin = np.asarray(origin, dtype=float).ravel()
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2]) if origin.size >= 3 else 0.0
        self._cos_yaw = float(np.cos(self.origin_yaw))
        self._sin_yaw = float(np.sin(self.origin_yaw))
        self.height, self.width = self.mask.shape
        self.should_spoof = False

    @classmethod
    def from_json(cls, path):
        path = Path(path).expanduser().resolve()
        with open(path) as f:
            payload = json.load(f)
        return cls(
            mask=payload["mask"],
            resolution=payload["resolution"],
            origin=payload.get("origin", [0.0, 0.0, 0.0]),
        )

    def __points_to_keep(self, points_x, points_y):
        points_x = np.asarray(points_x, dtype=np.float64)
        points_y = np.asarray(points_y, dtype=np.float64)
        dx = points_x - self.origin_x
        dy = points_y - self.origin_y
        local_x = self._cos_yaw * dx + self._sin_yaw * dy
        local_y = -self._sin_yaw * dx + self._cos_yaw * dy
        cols = np.floor(local_x / self.resolution).astype(np.int64)
        rows = (self.height - 1) - np.floor(local_y / self.resolution).astype(np.int64)
        in_bounds = (
            (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        )
        keep = np.zeros(points_x.shape, dtype=bool)
        keep[in_bounds] = self.mask[rows[in_bounds], cols[in_bounds]]
        return keep

    def filter_scan(self, ranges, angles, pose_x, pose_y, pose_yaw) -> np.ndarray:
        ranges = np.asarray(ranges, dtype=np.float64)
        angles = np.asarray(angles, dtype=np.float64)
        _, finite, px, py = scan_to_world_points(ranges, angles, pose_x, pose_y, pose_yaw)
        out = ranges.copy()
        keep = np.zeros(ranges.shape, dtype=bool)
        if finite.any():
            keep[finite] = self.__points_to_keep(px[finite], py[finite])
        valid = finite & keep
        spoof_targets = finite & ~keep
        out[~valid] = np.inf
        if valid.sum() < 4 or not spoof_targets.any() or not self.should_spoof:
            return out
        spline = CubicSpline(angles[valid][10::3], ranges[valid][10::3], bc_type="natural", extrapolate=False)
        spoofed = spline(angles[spoof_targets])
        idx = np.where(spoof_targets)[0]
        ok = np.isfinite(spoofed)
        out[idx[ok]] = spoofed[ok]
        return out

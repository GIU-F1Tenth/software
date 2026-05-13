import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.interpolate import CubicSpline

from lidar_filter.common import scan_to_world_points
from lidar_filter.filter_base import Filter


class MaskAndSpoofFilter(Filter):
    def __init__(self, mask, resolution, origin, map_image=None, map_free_threshold=250):
        self.mask = np.asarray(mask, dtype=bool)
        if self.mask.ndim != 2:
            raise ValueError(f"mask must be 2D, got shape {self.mask.shape}")
        if map_image is not None:
            map_arr = np.asarray(map_image)
            if map_arr.ndim != 2:
                raise ValueError(f"map must be 2D grayscale, got shape {map_arr.shape}")
            if map_arr.shape != self.mask.shape:
                raise ValueError(
                    f"map shape {map_arr.shape} does not match mask shape {self.mask.shape}"
                )
            self.mask = self.mask & (map_arr >= int(map_free_threshold))
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
    def from_json(cls, path, map_path=None):
        path = Path(path).expanduser().resolve()
        with open(path) as f:
            payload = json.load(f)
        map_image = None
        if map_path:
            map_image = np.array(
                Image.open(Path(map_path).expanduser().resolve()).convert("L")
            )
        return cls(
            mask=payload["mask"],
            resolution=payload["resolution"],
            origin=payload.get("origin", [0.0, 0.0, 0.0]),
            map_image=map_image,
        )

    def _world_to_pixel(self, x, y):
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        dx = x - self.origin_x
        dy = y - self.origin_y
        local_x = self._cos_yaw * dx + self._sin_yaw * dy
        local_y = -self._sin_yaw * dx + self._cos_yaw * dy
        cols = np.floor(local_x / self.resolution).astype(np.int64)
        rows = (self.height - 1) - np.floor(local_y / self.resolution).astype(np.int64)
        return cols, rows

    def __lines_keep(self, sx, sy, ex, ey):
        ex = np.atleast_1d(ex).astype(np.int64)
        ey = np.atleast_1d(ey).astype(np.int64)
        if ex.size == 0:
            return np.zeros(0, dtype=bool)
        dx = ex - sx
        dy = ey - sy
        n_steps = np.maximum(np.abs(dx), np.abs(dy))
        max_n = int(n_steps.max())
        if max_n == 0:
            in_bounds = (0 <= sx < self.width) and (0 <= sy < self.height)
            return np.full(ex.shape, bool(in_bounds and self.mask[sy, sx]), dtype=bool)
        norm = np.where(n_steps == 0, 1, n_steps).astype(np.float64)
        t = np.arange(max_n + 1, dtype=np.float64)
        s = np.minimum(t[None, :] / norm[:, None], 1.0)
        xs = np.rint(sx + s * dx[:, None]).astype(np.int64)
        ys = np.rint(sy + s * dy[:, None]).astype(np.int64)
        in_bounds = (xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height)
        safe_x = np.clip(xs, 0, self.width - 1)
        safe_y = np.clip(ys, 0, self.height - 1)
        valid = self.mask[safe_y, safe_x] & in_bounds
        return valid.all(axis=1)

    def __points_to_keep(self, pose_x, pose_y, points_x, points_y):
        sx_arr, sy_arr = self._world_to_pixel(pose_x, pose_y)
        sx = int(sx_arr)
        sy = int(sy_arr)
        ex, ey = self._world_to_pixel(points_x, points_y)
        return self.__lines_keep(sx, sy, ex, ey)

    def filter_scan(self, ranges, angles, pose_x, pose_y, pose_yaw) -> np.ndarray:
        ranges = np.asarray(ranges, dtype=np.float64)
        angles = np.asarray(angles, dtype=np.float64)
        _, finite, px, py = scan_to_world_points(ranges, angles, pose_x, pose_y, pose_yaw)
        out = ranges.copy()
        keep = np.zeros(ranges.shape, dtype=bool)
        if finite.any():
            keep[finite] = self.__points_to_keep(pose_x, pose_y, px[finite], py[finite])
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

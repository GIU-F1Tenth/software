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
        self.spoof_method = "none"

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

    def _spoof_cubic_spline(self, ranges, angles, valid, spoof_targets, out):
        if valid.sum() < 4:
            return
        spline = CubicSpline(
            angles[valid][10::3], ranges[valid][10::3], bc_type="natural", extrapolate=False
        )
        spoofed = spline(angles[spoof_targets])
        idx = np.where(spoof_targets)[0]
        ok = np.isfinite(spoofed)
        out[idx[ok]] = spoofed[ok]

    def _spoof_straight_line(self, ranges, angles, valid, spoof_targets, out):
        spoof_idx = np.where(spoof_targets)[0]
        valid_idx = np.where(valid)[0]
        if spoof_idx.size == 0 or valid_idx.size == 0:
            return
        groups = np.split(spoof_idx, np.where(np.diff(spoof_idx) != 1)[0] + 1)
        for group in groups:
            left = valid_idx[valid_idx < group[0]]
            right = valid_idx[valid_idx > group[-1]]
            if left.size == 0 or right.size == 0:
                continue
            i_l, i_r = left[-1], right[0]
            r1, a1 = ranges[i_l], angles[i_l]
            r2, a2 = ranges[i_r], angles[i_r]
            x1, y1 = r1 * np.cos(a1), r1 * np.sin(a1)
            x2, y2 = r2 * np.cos(a2), r2 * np.sin(a2)
            a = angles[group]
            denom = np.cos(a) * (y2 - y1) - np.sin(a) * (x2 - x1)
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (x1 * y2 - x2 * y1) / denom
            ok = np.isfinite(t) & (t > 0)
            out[group[ok]] = t[ok]

    def _spoof(self, ranges, angles, valid, spoof_targets, out):
        method = self._SPOOF_METHODS.get(self.spoof_method)
        if method is None:
            return
        method(self, ranges, angles, valid, spoof_targets, out)

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
        if spoof_targets.any():
            self._spoof(ranges, angles, valid, spoof_targets, out)
        return out

    _SPOOF_METHODS = {
        "cubic_spline": _spoof_cubic_spline,
        "straight_line": _spoof_straight_line,
    }

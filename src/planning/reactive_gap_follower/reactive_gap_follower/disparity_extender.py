"""Disparity-extender reactive gap follower.

Algorithm (Nayar/Hall, F1TENTH community canonical form):

1. Preprocess a LaserScan range array: replace NaN/inf with max_range,
   clip to [min_range, max_range], optionally restrict to a forward
   angular window (e.g. [-90, +90] deg).
2. Find disparities: consecutive indices where |range[i+1] - range[i]| is
   larger than `disparity_threshold`. Each disparity is a near edge
   followed by a far edge (or vice versa).
3. For every disparity, compute the angular extent that the near range
   needs to be smeared over so the car (modeled as a disk of radius
   `car_half_width + safety_margin`) cannot collide:
        n_samples = ceil(asin(r / d_near) / angle_increment)
   Overwrite the `n_samples` rays *away from the obstacle and onto the
   side of the gap* with `d_near`. This is the "extension".
4. After all disparities are extended, pick the ray with the maximum
   remaining range. That ray's angle is the steering goal.
5. Steering is `kp * goal_angle` clamped to `max_steer`. Speed is scaled
   by the chosen ray's range so the car slows in tight corridors.

This module has NO ROS dependency -- it's pure numpy and is unit-tested
without launching anything.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


@dataclass
class DisparityConfig:
    """All knobs for the disparity extender.

    Distances are meters, angles are radians.
    """
    # Car geometry & safety
    car_half_width: float = 0.15           # 0.30 m total width -> 0.15 m half
    safety_margin: float = 0.05            # extra clearance beyond car_half_width

    # Scan preprocessing
    min_range: float = 0.10                # clip ranges below this to this
    max_range: float = 10.0                # cap ranges above this to this
    fov_half: float = np.deg2rad(90.0)     # only look at rays within +/- this from forward

    # Disparity detection
    disparity_threshold: float = 0.30      # |dr| above this counts as a disparity [m]

    # Steering & speed mapping
    kp_steer: float = 0.8                  # steering = kp * goal_angle (clamped)
    max_steer: float = 0.40                # rad
    min_speed: float = 1.0                 # speed when corridor is shortest [m/s]
    max_speed: float = 5.0                 # speed when corridor is longest [m/s]
    speed_range_for_max: float = 6.0       # range value at which we hit max_speed [m]

    # Behavior when no scan / all rays blocked
    safe_stop_speed: float = 0.0


@dataclass
class DisparityResult:
    """Output of one planning step."""
    steering_angle: float
    speed: float
    goal_angle: float                       # raw chosen angle [rad], pre-kp
    goal_index: int                         # index into the *original* ranges
    goal_range: float                       # range at goal_index after extension
    processed_ranges: np.ndarray            # post-extension ranges, sized like input
    valid: bool                             # False if no usable scan
    reason: str                             # short tag for logs / diagnostics


def _angular_indices_for_radius(
    range_at_disparity: float,
    inflation_radius: float,
    angle_increment: float,
) -> int:
    """How many ray indices to extend on the gap-side of a disparity.

    Geometry: at distance `d_near`, an obstacle of radius `r` subtends a half-angle
    of asin(r / d_near). We convert that to a ray count using the scan's
    `angle_increment`. Clamp d_near away from zero so we don't divide by ~0.
    """
    d = max(range_at_disparity, inflation_radius * 1.01)
    half_angle = np.arcsin(min(1.0, inflation_radius / d))
    return int(np.ceil(half_angle / max(angle_increment, 1e-6)))


class DisparityExtender:
    """Stateless planner. Call `plan(ranges, angle_min, angle_increment)` each tick."""

    def __init__(self, config: DisparityConfig):
        self.config = config

    @property
    def inflation_radius(self) -> float:
        return self.config.car_half_width + self.config.safety_margin

    def plan(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
    ) -> DisparityResult:
        cfg = self.config
        if ranges.size == 0 or angle_increment <= 0:
            return DisparityResult(
                steering_angle=0.0,
                speed=cfg.safe_stop_speed,
                goal_angle=0.0,
                goal_index=-1,
                goal_range=0.0,
                processed_ranges=np.zeros(0),
                valid=False,
                reason="empty_scan",
            )

        # ---- 1. Preprocess ----
        # Replace NaN/inf with max_range, clip to [min_range, max_range].
        clean = np.where(np.isfinite(ranges), ranges, cfg.max_range)
        clean = np.clip(clean, cfg.min_range, cfg.max_range)

        # Build the angle array matching `clean`.
        n = clean.size
        angles = angle_min + np.arange(n) * angle_increment

        # Mask outside the FOV: replace with 0 so they're never picked as goal.
        in_fov = np.abs(angles) <= cfg.fov_half
        if not np.any(in_fov):
            return DisparityResult(
                steering_angle=0.0,
                speed=cfg.safe_stop_speed,
                goal_angle=0.0,
                goal_index=-1,
                goal_range=0.0,
                processed_ranges=clean,
                valid=False,
                reason="fov_empty",
            )

        # ---- 2 & 3. Find disparities and extend the near range ----
        processed = clean.copy()
        diffs = np.diff(processed)
        # Indices where a disparity occurs. diffs[i] = ranges[i+1] - ranges[i].
        disparity_idx = np.where(np.abs(diffs) > cfg.disparity_threshold)[0]
        for i in disparity_idx:
            r_left = processed[i]
            r_right = processed[i + 1]
            if r_left < r_right:
                # Near object on the LEFT of the disparity (index i).
                # Extend processed[i] forward onto the gap side (i+1, i+2, ...).
                near = r_left
                k = _angular_indices_for_radius(near, self.inflation_radius,
                                                angle_increment)
                end = min(i + 1 + k, n)
                processed[i + 1:end] = np.minimum(processed[i + 1:end], near)
            else:
                # Near object on the RIGHT of the disparity (index i+1).
                # Extend processed[i+1] backward onto the gap side (i, i-1, ...).
                near = r_right
                k = _angular_indices_for_radius(near, self.inflation_radius,
                                                angle_increment)
                start = max(0, i + 1 - k)
                processed[start:i + 1] = np.minimum(processed[start:i + 1], near)

        # ---- 4. Pick the longest remaining ray inside the FOV ----
        # Zero out the rays outside the FOV so argmax can't choose them.
        scored = np.where(in_fov, processed, -1.0)
        if not np.any(scored > 0.0):
            return DisparityResult(
                steering_angle=0.0,
                speed=cfg.safe_stop_speed,
                goal_angle=0.0,
                goal_index=-1,
                goal_range=0.0,
                processed_ranges=processed,
                valid=False,
                reason="all_blocked",
            )
        # Break argmax ties toward the forward ray by adding a tiny
        # forward-preference term. Magnitude < typical lidar resolution
        # (1 mm) so it never beats a real longer ray.
        forward_pref = -np.abs(angles) * 1e-4
        goal_idx = int(np.argmax(scored + forward_pref))
        goal_angle = float(angles[goal_idx])
        goal_range = float(processed[goal_idx])

        # ---- 5. Map to steering + speed ----
        steer = float(np.clip(cfg.kp_steer * goal_angle, -cfg.max_steer, cfg.max_steer))

        # Speed: linear interpolation from min_speed -> max_speed over
        # [min_range, speed_range_for_max]. Saturate at both ends.
        if goal_range <= cfg.min_range:
            speed = cfg.min_speed
        elif goal_range >= cfg.speed_range_for_max:
            speed = cfg.max_speed
        else:
            t = (goal_range - cfg.min_range) / max(
                cfg.speed_range_for_max - cfg.min_range, 1e-6
            )
            speed = cfg.min_speed + t * (cfg.max_speed - cfg.min_speed)

        # Slow further when we are steering hard (sharp corners): scale by
        # cos(steer) so the car never carries max_speed into a sharp turn.
        speed *= max(0.2, np.cos(steer))

        return DisparityResult(
            steering_angle=steer,
            speed=float(speed),
            goal_angle=goal_angle,
            goal_index=goal_idx,
            goal_range=goal_range,
            processed_ranges=processed,
            valid=True,
            reason="ok",
        )

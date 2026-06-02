"""Unit tests for the disparity extender. No ROS dependency."""
import time

import numpy as np
import pytest

from reactive_gap_follower.disparity_extender import (
    DisparityConfig,
    DisparityExtender,
)


def _scan(n: int = 1080, angle_min: float = -np.pi, angle_max: float = np.pi,
          fill: float = 10.0):
    angles = np.linspace(angle_min, angle_max, n)
    ranges = np.full(n, fill)
    increment = float(angles[1] - angles[0])
    return ranges, angles, angle_min, increment


def test_open_corridor_goes_straight():
    """No obstacles -> steering ~ 0, goal angle near forward."""
    ranges, _, amin, inc = _scan(fill=8.0)
    cfg = DisparityConfig()
    p = DisparityExtender(cfg)
    r = p.plan(ranges, amin, inc)
    assert r.valid
    assert abs(r.goal_angle) < 1e-2, f"expected straight ahead, got {r.goal_angle}"
    assert abs(r.steering_angle) < 1e-2


def test_wall_on_left_steers_right():
    """Obstacle band on the left of forward should push steering right (negative)."""
    ranges, angles, amin, inc = _scan(fill=8.0)
    mask = (angles > np.deg2rad(20)) & (angles < np.deg2rad(60))
    ranges[mask] = 1.0
    cfg = DisparityConfig()
    p = DisparityExtender(cfg)
    r = p.plan(ranges, amin, inc)
    assert r.valid
    assert r.goal_angle < np.deg2rad(10), (
        f"goal {np.rad2deg(r.goal_angle):.1f} deg should be right of obstacle"
    )
    assert r.steering_angle <= 0.0


def test_disparity_inflation_blocks_thin_gap():
    """A tiny pocket between two obstacles should be closed by inflation."""
    ranges, angles, amin, inc = _scan(n=2160, fill=10.0)
    left_mask = (angles > np.deg2rad(30.3)) & (angles < np.deg2rad(40))
    right_mask = (angles > np.deg2rad(20)) & (angles < np.deg2rad(29.7))
    ranges[left_mask] = 1.0
    ranges[right_mask] = 1.0
    cfg = DisparityConfig(car_half_width=0.15, safety_margin=0.05)
    p = DisparityExtender(cfg)
    r = p.plan(ranges, amin, inc)
    assert r.valid
    assert not (np.deg2rad(29.7) < r.goal_angle < np.deg2rad(30.3)), (
        f"chose to drive through a closed pocket: {np.rad2deg(r.goal_angle):.2f} deg"
    )


def test_slalom_picks_a_wide_gap():
    """40 cm spaced cones in front: planner should still find a feasible side."""
    n = 2160
    ranges, angles, amin, inc = _scan(n=n, fill=10.0)
    for cone_deg in (-23.0, -11.5, 11.5, 23.0):
        mask = np.abs(angles - np.deg2rad(cone_deg)) < np.deg2rad(0.5)
        ranges[mask] = 2.0
    cfg = DisparityConfig(car_half_width=0.15, safety_margin=0.05,
                          disparity_threshold=0.3)
    p = DisparityExtender(cfg)
    r = p.plan(ranges, amin, inc)
    assert r.valid
    assert r.goal_range > 3.0, f"goal range {r.goal_range:.2f} m"


def test_nan_inf_handled_as_max_range():
    ranges, _, amin, inc = _scan(fill=8.0)
    ranges[100] = np.nan
    ranges[200] = np.inf
    ranges[300] = -np.inf
    p = DisparityExtender(DisparityConfig())
    r = p.plan(ranges, amin, inc)
    assert r.valid
    assert r.processed_ranges[100] >= 1.0


def test_empty_scan_safe_stop():
    p = DisparityExtender(DisparityConfig())
    r = p.plan(np.zeros(0), -np.pi, 0.0)
    assert not r.valid
    assert r.speed == 0.0
    assert r.steering_angle == 0.0


def test_speed_scales_with_corridor():
    cfg = DisparityConfig(min_speed=1.0, max_speed=5.0, speed_range_for_max=6.0)
    p = DisparityExtender(cfg)
    short_ranges, _, amin, inc = _scan(fill=1.0)
    long_ranges, _, _, _ = _scan(fill=8.0)
    r_short = p.plan(short_ranges, amin, inc)
    r_long = p.plan(long_ranges, amin, inc)
    assert r_short.speed < r_long.speed


def test_fov_window_excludes_rear():
    """Rays behind the car must never be picked as goal."""
    ranges, angles, amin, inc = _scan(fill=2.0)
    behind = np.argmin(np.abs(angles - np.pi))
    ranges[behind] = 10.0
    cfg = DisparityConfig(fov_half=np.deg2rad(90))
    p = DisparityExtender(cfg)
    r = p.plan(ranges, amin, inc)
    assert r.valid
    assert abs(r.goal_angle) <= np.deg2rad(90)


def test_runs_fast_enough():
    """Hot loop budget: planner under 2 ms on a 1080-ray scan."""
    ranges, _, amin, inc = _scan(n=1080, fill=4.0)
    p = DisparityExtender(DisparityConfig())
    p.plan(ranges, amin, inc)
    n_iter = 200
    t0 = time.perf_counter()
    for _ in range(n_iter):
        p.plan(ranges, amin, inc)
    avg_ms = (time.perf_counter() - t0) / n_iter * 1e3
    assert avg_ms < 2.0, f"planner avg {avg_ms:.2f} ms > 2 ms"

"""Waypoint generators for simulation and testing."""
import numpy as np
from typing import List, Dict


def straight_track(length: float = 50.0, v_ref: float = 3.0,
                   n_points: int = 200) -> List[Dict]:
    """Straight line along x-axis."""
    xs = np.linspace(0, length, n_points)
    return [{'x': x, 'y': 0.0, 'theta': 0.0, 'v': v_ref} for x in xs]


def curve_track(radius: float = 3.0, sweep_deg: float = 90.0,
                v_ref: float = 2.0, n_points: int = 100,
                start_x: float = 0.0, start_y: float = 0.0,
                start_theta: float = 0.0) -> List[Dict]:
    """
    Circular arc.
    Center is radius away, perpendicular-left of the current heading.
    """
    sweep = np.radians(sweep_deg)
    # Center is radius away, perpendicular-left of the current heading
    cx = start_x + radius * np.cos(start_theta + np.pi / 2)
    cy = start_y + radius * np.sin(start_theta + np.pi / 2)
    # Arc starts pointing back toward start_x,start_y from the center
    start_angle = start_theta - np.pi / 2
    angles = np.linspace(start_angle, start_angle + sweep, n_points)
    waypoints = []
    for a in angles:
        x = cx + radius * np.cos(a)
        y = cy + radius * np.sin(a)
        theta = a + np.pi / 2
        waypoints.append({'x': x, 'y': y, 'theta': theta, 'v': v_ref})
    return waypoints


def mixed_track() -> List[Dict]:
    """
    Straight → curve → straight → curve → straight (chicane).
    Used for full FSM simulation.
    """
    track = []

    # Straight 1: 20m along x
    s1 = straight_track(length=20.0, v_ref=3.0, n_points=100)
    track.extend(s1)

    # Curve 1: 90-deg left, radius=4m
    last = s1[-1]
    c1 = curve_track(radius=4.0, sweep_deg=90.0, v_ref=2.0, n_points=80,
                     start_x=last['x'], start_y=last['y'],
                     start_theta=last['theta'])
    track.extend(c1)

    # Straight 2: 15m
    last = c1[-1]
    dx, dy = np.cos(last['theta']), np.sin(last['theta'])
    s2 = [{'x': last['x'] + dx * i * 0.2,
            'y': last['y'] + dy * i * 0.2,
            'theta': last['theta'], 'v': 3.0} for i in range(75)]
    track.extend(s2)

    # Curve 2: 90-deg right, radius=4m
    last = s2[-1]
    c2 = curve_track(radius=4.0, sweep_deg=90.0, v_ref=2.0, n_points=80,
                     start_x=last['x'], start_y=last['y'],
                     start_theta=last['theta'])
    track.extend(c2)

    # Straight 3: 20m finish
    last = c2[-1]
    dx, dy = np.cos(last['theta']), np.sin(last['theta'])
    s3 = [{'x': last['x'] + dx * i * 0.25,
            'y': last['y'] + dy * i * 0.25,
            'theta': last['theta'], 'v': 3.0} for i in range(80)]
    track.extend(s3)

    return track

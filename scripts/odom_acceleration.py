#!/usr/bin/env python3
"""
Reads a ROS2 odometry bag file and reports:
  1. Absolute acceleration: from first non-zero velocity to peak velocity.
  2. Point-by-point acceleration between consecutive samples.
  3. (Optional) Driving / braking forces when --mass is provided.
     F_drive = m*a + F_drag   where F_drag = dragcoeff * v²
     F_brake = m*|a| - F_drag

Usage:
    python3 odom_acceleration.py <bag_dir> [--topic /odom] [--threshold 0.01]
                                           [--mass 3.37] [--dragcoeff 0.075]
"""

import argparse
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def open_reader(bag_path: str, topic: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader


def read_odom(bag_path: str, topic: str):
    """Return list of (time_s, speed_xy) tuples."""
    reader = open_reader(bag_path, topic)
    records = []
    while reader.has_next():
        _, data, timestamp_ns = reader.read_next()
        msg: Odometry = deserialize_message(data, Odometry)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        records.append((timestamp_ns * 1e-9, math.hypot(vx, vy)))
    return records


def find_motion_start(records, threshold: float):
    for i, (_, speed) in enumerate(records):
        if speed > threshold:
            return i
    return None


def find_peak(records, start_idx: int):
    peak_idx = start_idx
    for i in range(start_idx, len(records)):
        if records[i][1] > records[peak_idx][1]:
            peak_idx = i
    return peak_idx


def drag_force(dragcoeff: float, v: float) -> float:
    return dragcoeff * v * v


def net_force(mass: float, dragcoeff: float, accel: float, v: float) -> float:
    """Total force the drivetrain must produce (positive = drive, negative = brake)."""
    return mass * accel + drag_force(dragcoeff, v)


def print_sep(char="─", width=72):
    print(char * width)


def main():
    parser = argparse.ArgumentParser(description="Odom bag acceleration & force analyser")
    parser.add_argument("bag_path", help="Path to the ROS2 bag directory")
    parser.add_argument("--topic",     default="/odom",  help="Odometry topic (default: /odom)")
    parser.add_argument("--threshold", type=float, default=0.01,
                        help="Speed threshold (m/s) for motion start detection (default: 0.01)")
    parser.add_argument("--mass",      type=float, default=None,
                        help="Vehicle mass [kg] — enables force output")
    parser.add_argument("--dragcoeff", type=float, default=0.0,
                        help="Drag coefficient [kg·m²/m³]  F_drag = dragcoeff·v²  (default: 0)")
    args = parser.parse_args()

    use_force = args.mass is not None

    print(f"Reading bag  : {args.bag_path}")
    print(f"Topic        : {args.topic}")
    print(f"Threshold    : {args.threshold} m/s")
    if use_force:
        print(f"Mass         : {args.mass} kg")
        print(f"Drag coeff   : {args.dragcoeff} kg·m²/m³")
    print()

    records = read_odom(args.bag_path, args.topic)
    if not records:
        print("ERROR: no messages found on topic", args.topic)
        sys.exit(1)

    print(f"Total samples: {len(records)}")
    print(f"Duration     : {records[-1][0] - records[0][0]:.3f} s\n")

    # ── 1. Absolute acceleration & peak driving force ─────────────────────────
    print_sep()
    print("ABSOLUTE ACCELERATION  (start of motion → peak velocity)")
    print_sep()

    start_idx = find_motion_start(records, args.threshold)
    if start_idx is None:
        print("Robot never exceeded the speed threshold — no motion detected.")
        accel_abs = None
    else:
        peak_idx = find_peak(records, start_idx)
        t_start, v_start = records[start_idx]
        t_peak,  v_peak  = records[peak_idx]
        dt = t_peak - t_start

        if dt <= 0:
            print("Start and peak are the same sample — cannot compute acceleration.")
            accel_abs = None
        else:
            accel_abs = (v_peak - v_start) / dt
            print(f"  Motion start  : t = {t_start:.4f} s,  speed = {v_start:.4f} m/s  (sample {start_idx})")
            print(f"  Peak velocity : t = {t_peak:.4f} s,  speed = {v_peak:.4f} m/s  (sample {peak_idx})")
            print(f"  Δt            : {dt:.4f} s")
            print(f"  Δv            : {v_peak - v_start:.4f} m/s")
            print(f"  Acceleration  : {accel_abs:.4f} m/s²")
            if use_force:
                f_inertia = args.mass * accel_abs
                f_d       = drag_force(args.dragcoeff, v_peak)
                f_total   = f_inertia + f_d
                print()
                print(f"  ── Force breakdown at peak (v = {v_peak:.4f} m/s) ──")
                print(f"  Inertial force (m·a)  : {f_inertia:+.4f} N")
                print(f"  Aero drag (c·v²)      : {f_d:.4f} N")
                print(f"  Total drive force     : {f_total:+.4f} N  ← F_drive = m·a + c·v²")

    # ── 2. Point-by-point acceleration ───────────────────────────────────────
    print()
    print_sep()
    print("POINT-BY-POINT ACCELERATION  (consecutive samples)")
    print_sep()

    if use_force:
        header = (f"{'Sample':>7}  {'t (s)':>10}  {'speed (m/s)':>12}  "
                  f"{'Δt (s)':>8}  {'accel (m/s²)':>13}  {'F_net (N)':>10}")
    else:
        header = (f"{'Sample':>7}  {'t (s)':>10}  {'speed (m/s)':>12}  "
                  f"{'Δt (s)':>8}  {'accel (m/s²)':>13}")
    print(header)
    print_sep("·")

    pointwise = []  # (accel, v_mid, force_net)
    for i in range(1, len(records)):
        t0, v0 = records[i - 1]
        t1, v1 = records[i]
        dt = t1 - t0
        if dt <= 0:
            accel_str = "        N/A"
            force_str = "       N/A"
            pointwise.append((None, None, None))
        else:
            a = (v1 - v0) / dt
            v_mid = (v0 + v1) / 2.0
            accel_str = f"{a:+13.4f}"
            if use_force:
                fn = net_force(args.mass, args.dragcoeff, a, v_mid)
                force_str = f"{fn:+10.4f}"
            pointwise.append((a, v_mid, net_force(args.mass, args.dragcoeff, a, v_mid) if use_force else None))

        row = f"{i:>7}  {t1:>10.4f}  {v1:>12.4f}  {dt:>8.4f}  {accel_str}"
        if use_force:
            row += f"  {force_str}"
        print(row)

    # ── 3. Summary ────────────────────────────────────────────────────────────
    print()
    print_sep()
    print("SUMMARY")
    print_sep()

    speeds = [s for _, s in records]
    valid  = [(a, v, f) for a, v, f in pointwise if a is not None]
    accels = [a for a, _, _ in valid]

    print(f"  Max speed              : {max(speeds):.4f} m/s")
    print(f"  Mean speed             : {sum(speeds)/len(speeds):.4f} m/s")
    if accels:
        a_max = max(accels)
        a_min = min(accels)
        print(f"  Max acceleration       : {a_max:+.4f} m/s²")
        print(f"  Max deceleration       : {a_min:+.4f} m/s²")
        print(f"  Mean |acceleration|    : {sum(abs(a) for a in accels)/len(accels):.4f} m/s²")

        if use_force:
            forces = [f for _, _, f in valid]
            f_max  = max(forces)
            f_min  = min(forces)
            # Speed at max accel sample for breakdown
            idx_max_a = accels.index(a_max)
            a_peak, v_peak_a, _ = valid[idx_max_a]
            f_in   = args.mass * a_peak
            f_d    = drag_force(args.dragcoeff, v_peak_a)
            print()
            print(f"  Max drive force        : {f_max:+.4f} N")
            print(f"    (at a = {a_peak:.4f} m/s²,  v ≈ {v_peak_a:.4f} m/s)")
            print(f"    m·a = {f_in:.4f} N,  c·v² = {f_d:.4f} N")
            print(f"  Max brake force (mag.) : {abs(f_min):.4f} N")


if __name__ == "__main__":
    main()

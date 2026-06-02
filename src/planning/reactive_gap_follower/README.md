# reactive_gap_follower

Disparity-extender reactive controller for the GIU Berlin F1TENTH stack.
**Purely reactive: subscribes to `/scan` only — no odometry, no map, no
AMCL, no waypoints.** Designed for the case where localization is unreliable
and the car still needs to drive through a dense obstacle field (40 cm
spaced slalom cones, etc.).

This is a separate package from the existing `gap_follower` (which uses
edge detection + sigmoid mapping). The two are interchangeable as far as
`control_gateway` is concerned — they publish drive commands on different
topics and the FSM picks one.

## Algorithm: Disparity Extender

Compared to "classic" Follow-The-Gap (find the largest gap, aim at its
midpoint), the disparity extender deals with **dense obstacle arrangements
where the largest gap isn't actually the safest one**.

Per tick, on the raw `/scan`:

1. **Preprocess.** Replace `NaN`/`+inf`/`-inf` with `max_range`, clip into
   `[min_range, max_range]`, mask out rays outside the forward FOV
   (`±fov_half_deg`).
2. **Find disparities.** Iterate adjacent ray ranges. Any jump larger than
   `disparity_threshold` is a disparity — it means the near return is the
   edge of a real obstacle and the far return is in the gap behind it.
3. **Inflate near ranges across the disparity.** For every disparity, set
   the next `k` rays on the *gap side* to the *near* range, where
   `k = ceil( asin(r / d_near) / angle_increment )` and
   `r = car_half_width + safety_margin`. Geometrically: the car at distance
   `d_near` cannot fit through a corridor narrower than `2r`, so we paint
   over those rays.
4. **Pick the longest remaining ray** inside the FOV (with a tiny forward
   tiebreak so a perfectly uniform wall produces steering = 0).
5. **Steering** = `clip(kp_steer * goal_angle, ±max_steer)`.
   **Speed** scales linearly between `min_speed` (tight corridor) and
   `max_speed` (open track), then multiplied by `cos(steer)` so the car
   doesn't carry full speed into a sharp turn.

## Topics

| Direction | Topic                                       | Type                                | Notes |
|---|---|---|---|
| sub | `/scan`                                     | `sensor_msgs/LaserScan`              | raw Hokuyo, sensor QoS |
| sub | `/control_selector`                         | `std_msgs/String`                    | FSM-published; drive only emitted when `data == "reactive_gap_follower"` (configurable) |
| pub | `/reactive_gap_follower/drive`              | `ackermann_msgs/AckermannDriveStamped` | routed by `control_gateway` |
| pub | `/reactive_gap_follower/scan_processed`     | `sensor_msgs/LaserScan`              | post-extension ranges; great for RViz |
| pub | `/reactive_gap_follower/goal`               | `visualization_msgs/Marker`          | arrow at chosen ray, in laser frame |
| pub | `/reactive_gap_follower/diagnostics`        | `std_msgs/Float64MultiArray`         | `[compute_ms, goal_angle, goal_range, steer, speed, valid]` |

## Parameters

All knobs are in [`config/reactive_gap_follower.yaml`](config/reactive_gap_follower.yaml).
Highlights:

- **`car_half_width` + `safety_margin`** sum to the inflation radius. For a
  30 cm car with the default 5 cm margin, the planner refuses to plan
  through any corridor narrower than 40 cm.
- **`disparity_threshold`** decides what counts as an obstacle edge.
- **`fov_half_deg`** restricts the planner to the forward hemisphere.
- **`kp_steer` and `max_steer`** are the only steering knobs.

## Tuning quick reference

| Symptom | Try |
|---|---|
| Clips a cone | Increase `safety_margin`, or lower `disparity_threshold` |
| Stops dead in narrow but passable corridor | Decrease `car_half_width` or `safety_margin` |
| Wobbles on straights | Lower `kp_steer`, raise `disparity_threshold` |
| Drives into the rear of obstacles | Lower `max_speed` or `speed_range_for_max` |
| Takes the long way around at high speed | Raise `kp_steer` |

## Running standalone

```bash
colcon build --packages-select reactive_gap_follower
source install/setup.bash
ros2 launch reactive_gap_follower reactive_gap_follower.launch.py
```

## Tests

No ROS dependency:

```bash
cd src/planning/reactive_gap_follower
python3 -m pytest test/
```

Includes a hot-loop benchmark that asserts the planner stays under 2 ms
per scan on a 1080-ray Hokuyo input.

## Integration into f1tenth_stack (follow-up)

1. Add a `use_reactive_gap_follower` launch arg + `Node(...)` block in
   `bringup_launch.py`.
2. Copy `config/reactive_gap_follower.yaml` into
   `src/giu_f1t_system/f1tenth_stack/config/`.
3. Add `/reactive_gap_follower/drive` as a route in
   `control_gateway_params.yaml`.
4. Optionally extend the FSM (`StateTraits.REACTIVE_GAP_FOLLOWER`) so
   `fsm_node` can swap to it dynamically.

## Caveats

- **No memory.** Each tick is independent. If a cone briefly drops out of
  the scan, the planner may twitch. Run `lidar_filter` upstream and point
  `scan_topic` at `/scan/filtered` if you need smoothing.
- **Assumes `/scan` is in the laser frame.** No tf lookups.
- **Inflation radius is a worst-case disk model.** Treats the car as round.
- **No backup / 3-point-turn behavior.** If every forward ray is blocked,
  the node publishes `safe_stop_speed` and stays still. Recovery is the
  FSM's job.

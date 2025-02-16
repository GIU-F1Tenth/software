#!/usr/bin/env python3

import math
import rclpy
import rclpy.time
import numpy as np
from rclpy.node import Node, Publisher
from path_planner_node.path_planner import PathPlanner
import rclpy.subscription
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Path, Odometry, GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3, Pose, Quaternion
from tf2_ros import TransformListener, Buffer

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class PurePursuit(Node):
    def __init__(self):
        """
        Class constructor
        """
        super().__init__("pure_pursuit")

        # Set if in debug mode
        self.is_in_debug_mode = (
            self.has_parameter("~debug") and self.get_parameter("~debug") == "true"
        )

        # Publishers
        self.cmd_vel: Publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.lookahead_pub: Publisher = self.create_publisher(
            PointStamped, self.get_parameter('pure_pursuit_lookahed_publisher'), 10
        )

        if self.is_in_debug_mode:
            self.fov_cells_pub: Publisher = self.create_publisher(
                GridCells, self.get_parameter('pure_pursuit_fov_cells_publisher'), 100
            )
            self.close_wall_cells_pub: Publisher = self.create_publisher(
                GridCells, self.get_parameter('pure_pursuit_close_wall_cells_publisher'), 100
            )

        # Subscribers
        self.create_subscription(Odometry, "/odom", self.update_odometry, 10)
        self.create_subscription(OccupancyGrid, "/map", self.update_map, 10)
        self.create_subscription(Path, self.get_parameter('pure_pursuit_path_subscriber'), self.update_path, 10)
        self.create_subscription(Bool, self.get_parameter('pure_pursuit_enabled'), self.update_enabled, 10)

        # Pure pursuit parameters
        self.LOOKAHEAD_DISTANCE = self.get_parameter('pure_pursuit_lookahead_distance').get_parameter_value().double_value  # m
        self.WHEEL_BASE = self.get_parameter('pure_pursuit_wheel_base').get_parameter_value().double_value  # m
        self.MAX_DRIVE_SPEED = self.get_parameter('pure_pursuit_max_drive_speed').get_parameter_value().double_value  # m/s
        self.MAX_TURN_SPEED = self.get_parameter('pure_pursuit_max_turn_speed').get_parameter_value().double_value  # rad/s
        self.TURN_SPEED_KP = self.get_parameter('pure_pursuit_turn_speed_kp').get_parameter_value().double_value
        self.DISTANCE_TOLERANCE = self.get_parameter('pure_pursuit_distance_tolerance').get_parameter_value().double_value  # m

        # Obstacle avoidance parameters
        self.OBSTACLE_AVOIDANCE_GAIN = self.get_parameter('pure_pursuit_obstacle_avoidance_gain').get_parameter_value().double_value
        self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE = self.get_parameter('pure_pursuit_obstacle_avoidance_max_slow_down_distance').get_parameter_value().double_value  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE = self.get_parameter('pure_pursuit_obstacle_avoidance_min_slow_down_distance').get_parameter_value().double_value  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR = self.get_parameter('pure_pursuit_obstacle_avoidance_min_slow_down_factor').get_parameter_value().double_value
        self.FOV = self.get_parameter('pure_pursuit_fov').get_parameter_value().double_value  # degrees
        self.FOV_DISTANCE = self.get_parameter('pure_pursuit_fov_distance').get_parameter_value().integer_value  # Number of grid cells
        self.FOV_DEADZONE = self.get_parameter('pure_pursuit_fov_deadzone').get_parameter_value().double_value  # degrees
        self.SMALL_FOV = self.get_parameter('pure_pursuit_small_fov').get_parameter_value().double_value  # degrees
        self.SMALL_FOV_DISTANCE = self.get_parameter('pure_pursuit_small_fov_distance').get_parameter_value().integer_value  # Number of grid cells

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose = None
        self.map = None
        self.path = Path()
        self.alpha = 0
        self.enabled = True
        self.reversed = False
        self.closest_distance = float("inf")
        self.timer = self.create_timer(0.01, self.run)
        self.get_logger().info("Pure pursuit node started")

    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        """
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", now
            )
            trans = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
            rot = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)
        except:
            return

        self.pose = Pose(
            position=Point(x=trans[0], y=trans[1]),
            orientation=Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3]),
        )

    def update_map(self, msg: OccupancyGrid):
        """
        Updates the current map.
        This method is a callback bound to a Subscriber.
        :param msg [OccupancyGrid] The current map information.
        """
        self.map = msg

    def update_path(self, msg: Path):
        self.path = msg

    def update_enabled(self, msg: Bool):
        self.enabled = msg.data

    def calculate_steering_adjustment(self) -> float:
        if self.pose is None or self.map is None:
            return 0

        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        yaw = float(np.rad2deg(yaw))

        # Get the grid cell of the robot
        robot_cell = PathPlanner.world_to_grid(self.map, self.pose.position)

        weighted_sum_of_angles = 0
        total_weight = 0
        self.closest_distance = float("inf")

        # Get all wall cells near the robot within the distance
        fov_cells = []
        wall_cells = []
        wall_cell_count = 0
        for dx in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
            for dy in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
                cell = (robot_cell[0] + dx, robot_cell[1] + dy)
                distance = PathPlanner.euclidean_distance(robot_cell, cell)

                # If the cell is out of bounds, ignore it
                if not PathPlanner.is_cell_in_bounds(self.map, cell):
                    continue

                is_wall = not PathPlanner.is_cell_walkable(self.map, cell)
                if is_wall and distance < self.closest_distance:
                    self.closest_distance = distance

                # Calculate the angle of the cell relative to the robot
                angle = float(np.rad2deg(np.arctan2(dy, dx))) - yaw

                # If reversed, add 180 to the angle
                if self.reversed:
                    angle += 180

                # Keep angle in the range of -180 to 180
                if angle < -180:
                    angle += 360
                elif angle > 180:
                    angle -= 360

                # Ignore scans that are outside the field of view
                is_in_fov = (
                    distance <= self.FOV_DISTANCE
                    and angle >= -self.FOV / 2
                    and angle <= self.FOV / 2
                    and not abs(angle) < self.FOV_DEADZONE / 2
                )
                is_in_small_fov = (
                    distance <= self.SMALL_FOV_DISTANCE
                    and angle >= -self.SMALL_FOV / 2
                    and angle <= self.SMALL_FOV / 2
                )
                if not is_in_fov and not is_in_small_fov:
                    continue

                # If in debug mode, add the cell to the field of view
                if self.is_in_debug_mode:
                    fov_cells.append(cell)

                # If cell is not a wall, ignore it
                if not is_wall:
                    continue

                weight = 1 / (distance**2) if distance != 0 else 0

                weighted_sum_of_angles += weight * angle
                total_weight += weight

                wall_cell_count += 1

                if self.is_in_debug_mode:
                    wall_cells.append(cell)

        # If in debug mode, publish the wall cells
        if self.is_in_debug_mode:
            self.fov_cells_pub.publish(PathPlanner.get_grid_cells(self.map, fov_cells))
            self.close_wall_cells_pub.publish(
                PathPlanner.get_grid_cells(self.map, wall_cells)
            )

        if total_weight == 0:
            return 0

        # Calculate the average angle (weighted sum of angles divided by total weight)
        average_angle = weighted_sum_of_angles / total_weight

        # Calculate the steering adjustment based on the average angle
        steering_adjustment = (
            -self.OBSTACLE_AVOIDANCE_GAIN * average_angle / wall_cell_count
        )
        return steering_adjustment

    @staticmethod
    def distance(x0, y0, x1, y1) -> float:
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def get_distance_to_waypoint_index(self, i: int) -> float:
        if self.pose is None or self.path.poses is None:
            return -1

        position = self.pose.position
        waypoint = self.path.poses[i].pose.position
        return PurePursuit.distance(position.x, position.y, waypoint.x, waypoint.y)

    def find_nearest_waypoint_index(self) -> int:
        nearest_waypoint_index = -1
        if self.path.poses is None:
            return nearest_waypoint_index

        closest_distance = float("inf")
        for i in range(len(self.path.poses) - 1):
            distance = self.get_distance_to_waypoint_index(i)
            if distance and distance < closest_distance:
                closest_distance = distance
                nearest_waypoint_index = i
        return nearest_waypoint_index

    def find_lookahead(self, nearest_waypoint_index, lookahead_distance) -> Point:
        if self.path.poses is None:
            return Point()

        i = nearest_waypoint_index
        while (
            i < len(self.path.poses)
            and self.get_distance_to_waypoint_index(i) < lookahead_distance
        ):
            i += 1
        return self.path.poses[i - 1].pose.position

    def get_goal(self) -> Point:
        if self.path.poses is None:
            return Point()

        poses = self.path.poses
        return poses[len(poses) - 1].pose.position

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        twist = Twist(linear=Vector3(x=linear_speed), angular=Vector3(z=angular_speed))
        self.cmd_vel.publish(twist)

    def stop(self):
        self.send_speed(0.0, 0.0)

    def run(self):
        
        # If no pose, stop
        if self.pose is None:
            return

        # If not enabled, do nothing
        if not self.enabled:
            return

        # If no path, stop
        if self.path is None or not self.path.poses:
            self.stop()
            return

        goal = self.get_goal()

        nearest_waypoint_index = self.find_nearest_waypoint_index()
        lookahead = self.find_lookahead(
            nearest_waypoint_index, self.LOOKAHEAD_DISTANCE
        )

        self.lookahead_pub.publish(
            PointStamped(header=Header(frame_id="map"), point=lookahead)
        )

        # Calculate alpha (angle between target and current position)
        position = self.pose.position
        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        x = position.x
        y = position.y
        dx = lookahead.x - x
        dy = lookahead.y - y
        self.alpha = float(np.arctan2(dy, dx) - yaw)
        if self.alpha > np.pi:
            self.alpha -= 2 * np.pi
        elif self.alpha < -np.pi:
            self.alpha += 2 * np.pi

        # If the lookahead is behind the robot, follow the path backwards
        self.reversed = abs(self.alpha) > np.pi / 2

        # Calculate the lookahead distance and center of curvature
        lookahead_distance = PurePursuit.distance(x, y, lookahead.x, lookahead.y)
        radius_of_curvature = float(lookahead_distance / (2 * np.sin(self.alpha)))

        # Calculate drive speed
        drive_speed = (-1 if self.reversed else 1) * self.MAX_DRIVE_SPEED

        # Stop if at goal
        distance_to_goal = PurePursuit.distance(x, y, goal.x, goal.y)
        if distance_to_goal < self.DISTANCE_TOLERANCE:
            self.stop()
            return

        # Calculate turn speed
        turn_speed = self.TURN_SPEED_KP * drive_speed / radius_of_curvature

        # Obstacle avoicance
        turn_speed += self.calculate_steering_adjustment()

        # Clamp turn speed
        turn_speed = max(-self.MAX_TURN_SPEED, min(self.MAX_TURN_SPEED, turn_speed))

        # Slow down if close to obstacle
        if self.closest_distance < self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE:
            drive_speed *= float(
                np.interp(
                    self.closest_distance,
                    [
                        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE,
                        self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE,
                    ],
                    [self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR, 1],
                )
            )

        # Send speed
        self.send_speed(drive_speed, turn_speed)

def main(): 
    rclpy.init()
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    PurePursuit().run()
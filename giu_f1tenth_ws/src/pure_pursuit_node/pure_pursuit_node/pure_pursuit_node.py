# !/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Parameters
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('look_ahead_distance', 1.0)
        self.declare_parameter('max_speed', 6.65)
        self.declare_parameter('max_steering_angle_deg', 35.0)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = np.deg2rad(self.get_parameter('max_steering_angle_deg').value)

        # Waypoints from uploaded file
        self.waypoints = [
            (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0),
            (5.0, 0.0), (6.0, 0.0), (7.0, -0.2), (8.0, -0.50), (9.0, -0.50),
            (9.5, 0.25), (9.6, 0.5), (9.7, 0.75), (9.75, 1.0), (9.75, 1.5),
            (9.75, 3.0), (9.75, 4.0), (9.85, 5.0), (10.0, 6.0), (10.0, 7.0),
            (10.0, 7.5), (9.5, 8.0), (9.25, 8.25), (9.0, 8.5), (8.5, 8.5),
            (7.5, 8.75), (6.0, 8.75), (5.0, 8.75), (4.0, 8.75), (3.0, 8.75),
            (2.0, 8.75), (1.0, 8.75), (0.0, 8.75), (-1.0, 8.75), (-2.0, 8.75),
            (-3.0, 8.75), (-4.0, 8.75), (-5.0, 8.75), (-6.0, 8.75), (-7.0, 8.75),
            (-8.0, 8.75), (-9.0, 8.75), (-10.0, 8.75), (-11.0, 9.0), (-12.0,9.0),
            (-12.5, 8.75), (-13.0, 8.5), (-13.25, 8.2), (-13.5, 8.0), (-13.5, 7.8),
            (-13.5, 7.7), (-13.5, 7.6), (-13.5, 7.4), (-13.5, 6.75),(-13.5, 6.5),(-13.5, 6.25),(-13.5, 6.0),(-13.5, 5.75), (-13.5, 5.5),
            (-13.5, 5.0), (-13.5, 4.5), (-13.5, 4.0),(-13.5, 3.5),(-13.5, 2.5),(-13.5, 1.0), (-13.25, 0.5),(-13.0, 0.25), (-12.75, 0.15),
            (-12.6, 0.0), (-12.5, 0.0), (-12.25, 0.0), (-12.0, 0.0), (-11.0, 0.0),
            (-10.0, 0.0), (-9.0, 0.0), (-8.0, 0.0), (-7.0, 0.0), (-6.0, 0.0),
            (-5.0, 0.0), (-4.0, 0.0), (-3.0, 0.0), (-2.0, 0.0), (-1.0, 0.0),
            (0.0, 0.0)
        ]

        self.current_waypoint_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Subscribers and Publishers
        self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)

        self.get_logger().info("Pure Pursuit Node Initialized.")

    def pose_callback(self, pose_msg):
        # Extract position and orientation
        self.current_x = pose_msg.pose.pose.position.x
        self.current_y = pose_msg.pose.pose.position.y
        orientation = pose_msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(orientation)

        # Publish path for visualization
        self.publish_path()

        # Get lookahead point
        lookahead_point = self.get_lookahead_point()
        if lookahead_point:
            steering_angle = self.compute_steering_angle(lookahead_point)
            self.publish_drive_command(steering_angle, self.max_speed)
        else:
            self.get_logger().warn("No valid lookahead point found!")

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for (x, y) in self.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        self.path_publisher.publish(path_msg)

    def get_lookahead_point(self):
        for i in range(len(self.waypoints)):
            idx = (self.current_waypoint_idx + i) % len(self.waypoints)
            wpx, wpy = self.waypoints[idx]
            dist = math.sqrt((wpx - self.current_x)**2 + (wpy - self.current_y)**2)
            if dist >= self.look_ahead_distance:
                self.current_waypoint_idx = idx
                return (wpx, wpy)
        return None

    def compute_steering_angle(self, target_point):
        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y

        # Transform to vehicle coordinates
        local_x = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
        local_y = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)

        if local_x <= 0.0:
            # If point is behind, return max steering
            return math.copysign(self.max_steering_angle, local_y)

        alpha = math.atan2(local_y, local_x)
        L = self.wheelbase
        Ld = math.sqrt(local_x**2 + local_y**2)
        steering_angle = math.atan2(2.0 * L * math.sin(alpha), Ld)
        return max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))

    def publish_drive_command(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_publisher.publish(drive_msg)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

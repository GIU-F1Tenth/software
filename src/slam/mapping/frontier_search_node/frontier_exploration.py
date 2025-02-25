#!/usr/bin/env python3

import os
import rospkg
import rclpy
import threading
import subprocess
import numpy as np
from typing import Union
from rclpy.node import Node
import rclpy.time
from path_planner_node.path_planner import PathPlanner
from .frontier_search import FrontierSearch
from nav_msgs.msg import OccupancyGrid, Path, GridCells, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from giu_f1t_interfaces.msg import FrontierList
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


class FrontierExploration(Node):
    def __init__(self):
        super().__init__(node_name="frontier_exploration")

        self.declare_parameter('~debug', 'false')
        self.declare_parameter('frontier_cells_publisher',
                               '/frontier_exploration/frontier_cells')
        self.declare_parameter('frontier_start_publisher',
                               '/frontier_exploration/start')
        self.declare_parameter('frontier_goal_publisher',
                               "/frontier_exploration/goal")
        self.declare_parameter('cspace_publisher', '/cspace')
        self.declare_parameter('cost_map_publisher', '/cost_map')
        self.declare_parameter('num_explore_fails_before_finish', 5)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('a_star_cost_weight', 10.0)
        self.declare_parameter('frontier_size_cost_weight', 1.0)
        self.declare_parameter('max_num_frontiers_to_check', 8)

        # Set if in debug mode
        self.is_in_debug_mode = (
            self.has_parameter("~debug") and self.get_parameter(
                "~debug") == "true"
        )

        # Publishers
        self.pure_pursuit_pub = self.create_publisher(
            Path, "/pure_pursuit/path", 10
        )

        if self.is_in_debug_mode:
            self.frontier_cells_pub = self.create_publisher(
                GridCells, self.get_parameter('frontier_cells_publisher'), 10
            )
            self.start_pub = self.create_publisher(
                GridCells, self.get_parameter('frontier_start_publisher'), 10
            )
            self.goal_pub = self.create_publisher(
                GridCells, self.get_parameter('frontier_goal_publisher'), 10
            )
            self.cspace_pub = self.create_publisher(
                GridCells, self.get_parameter('cspace_publisher'), 10)
            self.cost_map_pub = self.create_publisher(
                OccupancyGrid, self.get_parameter('cost_map_publisher'), 10
            )

        # Subscribers
        self.create_subscription(
            Odometry, "/ego_racecar/odom", self.update_odometry, 10)
        self.create_subscription(OccupancyGrid, "/map", self.update_map, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.pose = None
        self.map = None

        self.NUM_EXPLORE_FAILS_BEFORE_FINISH = self.get_parameter(
            'num_explore_fails_before_finish').value or 5
        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False
        self.rate = self.get_parameter("rate").value
        self.timer = self.create_timer(1/self.rate, self.run)
        self.get_logger().info("Frontier exploration node started")

    def update_odometry(self, msg: "Union[Odometry, None]" = None):
        """
        Updates the current pose of the robot.
        """
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                "map", "base_footprint", now
            ).transform.translation
            rot = self.tf_buffer.lookup_transform(
                "map", "base_footprint", now
            ).transform.rotation
        except:
            return

        self.pose = Pose(
            position=Point(x=trans.x, y=trans.y),
            orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
        )

    def update_map(self, msg: OccupancyGrid):
        """
        Updates the current map.
        This method is a callback bound to a Subscriber.
        :param msg [OccupancyGrid] The current map information.
        """
        self.map = msg

    def save_map(self):
        # Get the path of the current package
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("lab4")

        # Construct the path to the map
        map_path = os.path.join(package_path, "map/map")
        if not os.path.exists(os.path.dirname(map_path)):
            os.makedirs(os.path.dirname(map_path))

        # Run map_saver
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", map_path])

        self.update_odometry()

        if self.pose is None:
            self.get_logger().error("Failed to get pose")
            return

        # Save the robot's position and orientation
        position = self.pose.position
        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        with open(os.path.join(package_path, "map/pose.txt"), "w") as f:
            f.write(
                f"{position.x} {position.y} {position.z} {yaw} {pitch} {roll}\n")

    @staticmethod
    def get_top_frontiers(frontiers, n):
        # Sort the frontiers by size in descending order
        sorted_frontiers = sorted(
            frontiers, key=lambda frontier: frontier.size, reverse=True
        )

        # Return the top n frontiers
        return sorted_frontiers[:n]

    def publish_cost_map(self, mapdata: OccupancyGrid, cost_map: np.ndarray):
        # Create an OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = mapdata.info.resolution
        grid.info.width = cost_map.shape[1]
        grid.info.height = cost_map.shape[0]
        grid.info.origin = mapdata.info.origin

        # Normalize the cost map to the range [0, 100] and convert it to integers
        cost_map_normalized = (
            cost_map / np.max(cost_map) * 100).astype(np.int8)

        # Flatten the cost map and convert it to a list
        grid.data = cost_map_normalized.flatten().tolist()

        # Publish the OccupancyGrid message
        self.cost_map_pub.publish(grid)

    def check_if_finished_exploring(self):
        # Publish empty path to stop the robot
        self.pure_pursuit_pub.publish(Path())

        # If no frontiers or paths are found for a certain number of times, finish exploring
        if (
            self.no_frontiers_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
            or self.no_path_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
        ):
            self.get_logger().info("Done exploring!")
            self.save_map()
            self.get_logger().info("Saved map")
            self.is_finished_exploring = True

    def explore_frontier(self, frontier_list: FrontierList):
        # If finished exploring, no pose, no map, or no frontier list, return
        if self.is_finished_exploring or self.pose is None or self.map is None:
            return

        frontiers = frontier_list.frontiers

        # If no frontiers are found, check if finished exploring
        if not frontiers:
            self.get_logger().info("No frontiers")
            self.no_frontiers_found_counter += 1
            self.check_if_finished_exploring()
            return
        else:
            self.no_frontiers_found_counter = 0

        A_STAR_COST_WEIGHT = self.get_parameter(
            "a_star_cost_weight").value or 1.0
        FRONTIER_SIZE_COST_WEIGHT = self.get_parameter(
            "frontier_size_cost_weight").value or 1.0

        # Calculate the C-space
        cspace, cspace_cells = PathPlanner.calc_cspace(
            self.map, self.is_in_debug_mode)
        # if cspace_cells is not None:
        #     self.cspace_pub.publish(cspace_cells)

        # Calculate the cost map
        cost_map = PathPlanner.calc_cost_map(self.map)
        if self.is_in_debug_mode:
            self.publish_cost_map(self.map, cost_map)

        # Get the start
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Execute A* for every frontier
        lowest_cost = float("inf")
        best_path = None

        # Check only the top frontiers in terms of size
        MAX_NUM_FRONTIERS_TO_CHECK = self.get_parameter(
            "max_num_frontiers_to_check").value or 5
        top_frontiers = FrontierExploration.get_top_frontiers(
            frontiers, MAX_NUM_FRONTIERS_TO_CHECK
        )

        starts = []
        goals = []

        # Log how many frontiers are being explored
        self.get_logger().info(f"Exploring {len(top_frontiers)} frontiers")

        for frontier in top_frontiers:
            # Get goal
            goal = PathPlanner.world_to_grid(self.map, frontier.centroid)

            # Execute A*
            path, a_star_cost, start, goal = PathPlanner.a_star(
                cspace, cost_map, start, goal
            )

            # If in debug mode, append start and goal
            if self.is_in_debug_mode:
                starts.append(start)
                goals.append(goal)

            if path is None or a_star_cost is None:
                continue

            # Calculate cost
            cost = (A_STAR_COST_WEIGHT * a_star_cost) + (
                FRONTIER_SIZE_COST_WEIGHT / frontier.size
            )

            # Update best path
            if cost < lowest_cost:
                lowest_cost = cost
                best_path = path

        # If in debug mode, publish the start and goal
        if self.is_in_debug_mode:
            self.start_pub.publish(
                PathPlanner.get_grid_cells(self.map, starts))
            self.goal_pub.publish(PathPlanner.get_grid_cells(self.map, goals))

        # If a path was found, publish it
        if best_path:
            self.get_logger().info(f"Found best path with cost {lowest_cost}")
            start = best_path[0]
            path = PathPlanner.path_to_message(self.map, best_path)
            self.pure_pursuit_pub.publish(path)
            self.no_path_found_counter = 0
        # If no path was found, check if finished exploring
        else:
            self.get_logger().info("No paths found")
            self.no_path_found_counter += 1
            self.check_if_finished_exploring()

    def run(self):
        if self.pose is None or self.map is None:
            return

        # Get the start position of the robot
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Get frontiers
        frontier_list, frontier_cells = FrontierSearch.search(
            self.map, start, self.is_in_debug_mode
        )

        if frontier_list is None:
            return

        # Publish frontier cells if in debug mode
        if self.is_in_debug_mode:
            self.frontier_cells_pub.publish(
                PathPlanner.get_grid_cells(self.map, frontier_cells)
            )

        self.explore_frontier(frontier_list)


def main():
    rclpy.init()
    frontier_exploration = FrontierExploration()
    rclpy.spin(frontier_exploration)
    frontier_exploration.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

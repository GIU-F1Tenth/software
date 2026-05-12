import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

from lidar_filter.common import run_pipeline
from lidar_filter.mask_filter import MaskFilter
from lidar_filter.range_filter import RangeFilter


FILTERED_RANGE = float("inf")


class LidarFilterNode(Node):
    def __init__(self):
        super().__init__("lidar_filter")

        self.declare_parameter("scan_input_topic", "/scan")
        self.declare_parameter("scan_output_topic", "/scan_filtered")
        self.declare_parameter("odom_topic", "/car_state/odom")
        self.declare_parameter("mask_path", "")
        self.declare_parameter("max_range", -1.0)
        self.declare_parameter("queue_depth", 10)

        scan_input_topic = self.get_parameter("scan_input_topic").value
        scan_output_topic = self.get_parameter("scan_output_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        mask_path = self.get_parameter("mask_path").value
        max_range = float(self.get_parameter("max_range").value)
        queue_depth = int(self.get_parameter("queue_depth").value)

        if not mask_path:
            raise RuntimeError("mask_path parameter must be set to a mask JSON file")

        mask_filter = MaskFilter.from_json(mask_path)
        self.get_logger().info(
            f"loaded mask {mask_path} shape={mask_filter.mask.shape} "
            f"resolution={mask_filter.resolution}"
        )

        self.filters = [mask_filter]
        if max_range > 0:
            self.filters.append(RangeFilter(max_range))
            self.get_logger().info(f"range filter enabled with max_range={max_range}")

        self.latest_pose = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_depth,
        )

        self.create_subscription(Odometry, odom_topic, self._odom_callback, qos)
        self.create_subscription(LaserScan, scan_input_topic, self._scan_callback, qos)
        self.scan_publisher = self.create_publisher(LaserScan, scan_output_topic, qos)

    def _odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
        self.latest_pose = (position.x, position.y, float(yaw))

    def _scan_callback(self, msg: LaserScan):
        if self.latest_pose is None:
            self.get_logger().warn(
                "no odometry received yet; dropping scan",
                throttle_duration_sec=1.0,
            )
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        pose_x, pose_y, pose_yaw = self.latest_pose

        keep = run_pipeline(self.filters, ranges, angles, pose_x, pose_y, pose_yaw)
        filtered = np.where(keep, ranges, FILTERED_RANGE).astype(np.float32)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered.tolist()
        out.intensities = msg.intensities
        self.scan_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

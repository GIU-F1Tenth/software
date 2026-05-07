import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer

import numpy as np
from scipy.spatial.transform import Rotation
import math


class StatePublisherNode(Node):
    def __init__(self):
        super().__init__("state_publisher")

        self.declare_parameter("odom_input_topic", "/odom")
        self.declare_parameter("odom_output_topic", "/fused_odometry")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("tf_timeout", 1.0)
        self.declare_parameter("publish_rate", 50.0)

        self.odom_input_topic = self.get_parameter("odom_input_topic").value
        self.odom_output_topic = self.get_parameter("odom_output_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.tf_timeout = self.get_parameter("tf_timeout").value
        publish_rate = self.get_parameter("publish_rate").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_input_topic, self.odom_callback, qos_profile
        )

        self.odom_publisher = self.create_publisher(
            Odometry, self.odom_output_topic, qos_profile
        )

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.last_odom = None

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def timer_callback(self):
        if self.last_odom is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Transform lookup failed: {e}", throttle_duration_sec=1.0
            )
            return

        fused_odom = Odometry()
        fused_odom.header.stamp = self.get_clock().now().to_msg()
        fused_odom.header.frame_id = self.map_frame
        fused_odom.child_frame_id = self.base_link_frame

        map_T_odom = self.transform_to_matrix(transform)
        odom_pose = self.last_odom.pose.pose
        odom_T_base = self.pose_to_matrix(odom_pose)

        map_T_base = map_T_odom @ odom_T_base

        position = map_T_base[:3, 3]
        rotation = Rotation.from_matrix(map_T_base[:3, :3])
        quaternion = rotation.as_quat()

        fused_odom.pose.pose.position.x = position[0]
        fused_odom.pose.pose.position.y = position[1]
        fused_odom.pose.pose.position.z = position[2]
        fused_odom.pose.pose.orientation.x = quaternion[0]
        fused_odom.pose.pose.orientation.y = quaternion[1]
        fused_odom.pose.pose.orientation.z = quaternion[2]
        fused_odom.pose.pose.orientation.w = quaternion[3]

        fused_odom.pose.covariance = self.last_odom.pose.covariance
        fused_odom.twist = self.last_odom.twist

        self.odom_publisher.publish(fused_odom)

    def transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        position = np.array([translation.x, translation.y, translation.z])

        rot = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        rotation_matrix = rot.as_matrix()

        matrix = np.eye(4)
        matrix[:3, :3] = rotation_matrix
        matrix[:3, 3] = position

        return matrix

    def pose_to_matrix(self, pose) -> np.ndarray:
        position = np.array([pose.position.x, pose.position.y, pose.position.z])

        rot = Rotation.from_quat(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        rotation_matrix = rot.as_matrix()

        matrix = np.eye(4)
        matrix[:3, :3] = rotation_matrix
        matrix[:3, 3] = position

        return matrix


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

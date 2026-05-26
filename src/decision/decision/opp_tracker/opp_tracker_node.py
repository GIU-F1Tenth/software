#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np


class OvertakePredictorNode(Node):
    def __init__(self):
        super().__init__("opp_tracker")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("velocity_topic", "/object_velocities"),
                ("odom_topic", "/car_state/odom"),
                ("is_overtake_topic", "/is_overtake"),
                ("prediction_horizon", 1.5),
                ("prediction_dt", 0.1),
                ("collision_radius", 0.5),
                ("arrow_scale_seconds", 0.5),
                ("min_opponent_speed", 0.05),
                ("prediction_horizons_topic", "/prediction_horizons"),
                ("map_frame", "map"),
            ],
        )

        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.is_overtake_topic = self.get_parameter("is_overtake_topic").value
        self.prediction_horizon = self.get_parameter("prediction_horizon").value
        self.prediction_dt = self.get_parameter("prediction_dt").value
        self.collision_radius = self.get_parameter("collision_radius").value
        self.arrow_scale = self.get_parameter("arrow_scale_seconds").value
        self.min_opponent_speed = self.get_parameter("min_opponent_speed").value
        self.prediction_horizons_topic = self.get_parameter(
            "prediction_horizons_topic"
        ).value
        self.map_frame = self.get_parameter("map_frame").value

        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_vx = 0.0
        self.ego_vy = 0.0
        self.ego_received = False

        self.velocities_sub = self.create_subscription(
            MarkerArray, self.velocity_topic, self.velocities_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.is_overtake_pub = self.create_publisher(
            Bool, self.is_overtake_topic, 10
        )
        self.horizons_pub = self.create_publisher(
            MarkerArray, self.prediction_horizons_topic, 10
        )

        self.get_logger().info(
            f"Overtake predictor initialized: horizon={self.prediction_horizon}s, "
            f"dt={self.prediction_dt}s, collision_radius={self.collision_radius}m"
        )

    def odom_callback(self, msg: Odometry):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        vx_local = msg.twist.twist.linear.x
        vy_local = msg.twist.twist.linear.y
        c, s = np.cos(yaw), np.sin(yaw)
        self.ego_vx = c * vx_local - s * vy_local
        self.ego_vy = s * vx_local + c * vy_local
        self.ego_received = True

    def velocities_callback(self, msg: MarkerArray):
        if not self.ego_received:
            return

        opponents = self._extract_opponents(msg)
        is_overtake = self._is_overtake_safe(opponents)

        out = Bool()
        out.data = is_overtake
        self.is_overtake_pub.publish(out)

        self._publish_horizons(opponents, is_overtake)

    def _extract_opponents(self, msg: MarkerArray):
        opponents = []
        for marker in msg.markers:
            if marker.type != Marker.ARROW or marker.action != Marker.ADD:
                continue
            if len(marker.points) < 2:
                continue

            start = marker.points[0]
            end = marker.points[1]

            vx = (end.x - start.x) / self.arrow_scale
            vy = (end.y - start.y) / self.arrow_scale

            speed = np.sqrt(vx * vx + vy * vy)
            if speed < self.min_opponent_speed:
                continue

            opponents.append((start.x, start.y, vx, vy))
        return opponents

    def _is_overtake_safe(self, opponents):
        if not opponents:
            return True

        n_steps = max(1, int(self.prediction_horizon / self.prediction_dt))
        radius_sq = self.collision_radius ** 2

        for k in range(n_steps + 1):
            t = k * self.prediction_dt

            ego_x_t = self.ego_x + self.ego_vx * t
            ego_y_t = self.ego_y + self.ego_vy * t

            for opp_x, opp_y, opp_vx, opp_vy in opponents:
                opp_x_t = opp_x + opp_vx * t
                opp_y_t = opp_y + opp_vy * t

                dx = opp_x_t - ego_x_t
                dy = opp_y_t - ego_y_t
                if dx * dx + dy * dy < radius_sq:
                    return False

        return True

    def _publish_horizons(self, opponents, is_overtake):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))

        n_steps = max(1, int(self.prediction_horizon / self.prediction_dt))
        stamp = self.get_clock().now().to_msg()

        ego_marker = Marker()
        ego_marker.header.frame_id = self.map_frame
        ego_marker.header.stamp = stamp
        ego_marker.ns = "ego_horizon"
        ego_marker.id = 0
        ego_marker.type = Marker.LINE_STRIP
        ego_marker.action = Marker.ADD
        ego_marker.scale.x = 0.06
        if is_overtake:
            ego_marker.color.r = 0.0
            ego_marker.color.g = 1.0
            ego_marker.color.b = 0.0
        else:
            ego_marker.color.r = 1.0
            ego_marker.color.g = 0.0
            ego_marker.color.b = 0.0
        ego_marker.color.a = 1.0
        for k in range(n_steps + 1):
            t = k * self.prediction_dt
            p = Point()
            p.x = float(self.ego_x + self.ego_vx * t)
            p.y = float(self.ego_y + self.ego_vy * t)
            p.z = 0.05
            ego_marker.points.append(p)
        arr.markers.append(ego_marker)

        for i, (ox, oy, ovx, ovy) in enumerate(opponents):
            opp_marker = Marker()
            opp_marker.header.frame_id = self.map_frame
            opp_marker.header.stamp = stamp
            opp_marker.ns = "opponent_horizon"
            opp_marker.id = i
            opp_marker.type = Marker.LINE_STRIP
            opp_marker.action = Marker.ADD
            opp_marker.scale.x = 0.06
            opp_marker.color.r = 0.2
            opp_marker.color.g = 0.6
            opp_marker.color.b = 1.0
            opp_marker.color.a = 1.0
            for k in range(n_steps + 1):
                t = k * self.prediction_dt
                p = Point()
                p.x = float(ox + ovx * t)
                p.y = float(oy + ovy * t)
                p.z = 0.05
                opp_marker.points.append(p)
            arr.markers.append(opp_marker)

        self.horizons_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = OvertakePredictorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

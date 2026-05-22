#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from opp_tracker.opponent_tracker import (
    OpponentObservation,
    OpponentCenterTracker,
    centroid,
)


class OppTrackerNode(Node):
    def __init__(self):
        super().__init__("opp_tracker")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("bbox_topic", "/object_bboxes"),
                ("opponent_centers_topic", "/opponent_centers"),
                ("max_history_size", 500),
                ("marker_namespace", "opponent_centers"),
                ("marker_scale", 0.12),
            ],
        )

        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.opponent_centers_topic = self.get_parameter(
            "opponent_centers_topic"
        ).value
        self.marker_namespace = self.get_parameter("marker_namespace").value
        self.marker_scale = self.get_parameter("marker_scale").value

        self.tracker = OpponentCenterTracker(
            max_history_size=self.get_parameter("max_history_size").value
        )

        self.bbox_sub = self.create_subscription(
            MarkerArray, self.bbox_topic, self.bbox_callback, 10
        )
        self.centers_pub = self.create_publisher(
            Marker, self.opponent_centers_topic, 10
        )

        self.get_logger().info("Opponent Tracker Node initialized")

    def bbox_callback(self, msg: MarkerArray):
        centers = [
            (marker.header, centroid(self._corner_points(marker)))
            for marker in msg.markers
            if self._is_bounding_box(marker)
        ]
        if not centers:
            return

        header, center = min(centers, key=self._distance_to_ego)

        self.tracker.add_observation(
            OpponentObservation(
                x=center[0],
                y=center[1],
                z=center[2],
                stamp=self._stamp_to_seconds(header.stamp),
                frame_id=header.frame_id,
            )
        )

        self.publish_centers()

    @staticmethod
    def _is_bounding_box(marker):
        return (
            marker.type == Marker.LINE_STRIP
            and marker.action == Marker.ADD
            and len(marker.points) > 0
        )

    @staticmethod
    def _corner_points(marker):
        points = [(p.x, p.y, p.z) for p in marker.points]
        if len(points) > 1 and points[0] == points[-1]:
            points = points[:-1]
        return points

    @staticmethod
    def _distance_to_ego(header_and_center):
        _, center = header_and_center
        return center[0] ** 2 + center[1] ** 2

    @staticmethod
    def _stamp_to_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def publish_centers(self):
        history = self.tracker.history
        if not history:
            return

        marker = Marker()
        marker.header.frame_id = history[-1].frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_namespace
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            Point(x=obs.x, y=obs.y, z=obs.z) for obs in history
        ]

        self.centers_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = OppTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn.cluster import DBSCAN

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to the LiDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Change to '/echoes' if using multi-echo
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Extract LiDAR data
        ranges = np.array(msg.ranges)  # List of distances
        angle_min = msg.angle_min      # Minimum angle (radians)
        angle_increment = msg.angle_increment  # Angle between measurements (radians)

        # Step 1: Filter by the LiDAR's 270° FOV
        angles = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)
        fov_min = -np.deg2rad(135)  # -135° (leftmost angle)
        fov_max = np.deg2rad(135)   # +135° (rightmost angle)
        fov_filtered_indices = np.where((angles >= fov_min) & (angles <= fov_max))[0]
        fov_filtered_ranges = ranges[fov_filtered_indices]
        fov_filtered_angles = angles[fov_filtered_indices]

        # Step 2: Convert filtered polar coordinates to Cartesian coordinates
        x = fov_filtered_ranges * np.cos(fov_filtered_angles)  # Forward/backward (x-axis)
        y = fov_filtered_ranges * np.sin(fov_filtered_angles)  # Left/right (y-axis)
        points = np.column_stack((x, y))

        # Step 3: Filter by distance (e.g., within 5 meters along x and y axes)
        distance_threshold = 5.0  # Adjust as needed
        distance_filtered_indices = np.where(
            (np.abs(x) <= distance_threshold) & (np.abs(y) <= distance_threshold)
        )[0]
        distance_filtered_points = points[distance_filtered_indices]

        # Step 4: Cluster points to detect objects
        clustering = DBSCAN(eps=0.2, min_samples=3).fit(distance_filtered_points)
        labels = clustering.labels_

        # Step 5: Filter by size (e.g., ignore small clusters)
        large_cluster_indices = [i for i, label in enumerate(labels) if label != -1 and np.sum(labels == label) > 10]
        large_cluster_points = distance_filtered_points[large_cluster_indices]

        # Step 6: Separate objects on the sides from those in front
        side_threshold = 1.0  # Objects with |x| < 1.0 m are on the side
        front_threshold = 1.0  # Objects with |y| < 1.0 m are in front

        objects_in_front = []
        objects_on_side = []

        for point in large_cluster_points:
            x, y = point
            if abs(y) < front_threshold:
                objects_in_front.append(point)  # Object is in front
            elif abs(x) < side_threshold:
                objects_on_side.append(point)  # Object is on the side

        # Log the detected objects
        for i, point in enumerate(objects_in_front):
            x, y = point
            self.get_logger().info(
                f"Object {i + 1} in front at: (x={x:.2f}m, y={y:.2f}m)"
            )

        for i, point in enumerate(objects_on_side):
            x, y = point
            self.get_logger().info(
                f"Object {i + 1} on the side at: (x={x:.2f}m, y={y:.2f}m)"
            )

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
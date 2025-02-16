#!/usr/bin/env python3

import rclpy # ROS 2 Python library
from rclpy.node import Node # Base class for creating a ROS 2 node
from sensor_msgs.msg import LaserScan # ROS 2 message type used for LiDAR data
import serial # Library for communicating with the LiDAR sensor over a serial connection
import numpy as np # Library for numerical operations

# Simulate LiDAR scan data
num_points = 1080  # Number of points in the scan
angle_min = -3 * np.pi / 4  # -135° (start angle)
angle_max = 3 * np.pi / 4   # +135° (end angle)
angle_increment = (angle_max - angle_min) / num_points  # Angular resolution

# Generate mock ranges (distance measurements)
ranges = np.random.uniform(low=0.06, high=30.0, size=num_points)  # Random distances


class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher') # Initializes the node with the name "lidar_publisher"

        # Create a publisher
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 20) # The queue size is 20 (stores up to 20 messages before dropping older ones)
        
        # Set up the serial connection to the LiDAR (update port if needed)
        # Opens a serial connection to the Hokuyo LiDAR sensor
                                    #USB PORT
        self.lidar = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1.0)

        # Create a timer to publish LiDAR data
        # self.timer = self.create_timer(0.1, self.publish_scan) # Runs publish_scan() every 0.1 seconds (10 Hz)
        # Change this line in LidarPublisher __init__:
        self.timer = self.create_timer(0.1, self.publish_scan_mock)  # Use mock data

    def publish_scan_mock(self):
        try:
            # Generate mock LiDAR data
            num_points = 1080
            angle_min = -3 * np.pi / 4  # -135°
            angle_max = 3 * np.pi / 4    # +135°
            angle_increment = (angle_max - angle_min) / num_points
            ranges = np.random.uniform(low=0.06, high=30.0, size=num_points)

            # Create and populate the LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'lidar_frame'
            scan_msg.angle_min = angle_min
            scan_msg.angle_max = angle_max
            scan_msg.angle_increment = angle_increment
            scan_msg.time_increment = 0.0  # Simulated data, no time increment
            scan_msg.scan_time = 0.1       # Simulated scan time (10 Hz)
            scan_msg.range_min = 0.06     # Minimum range (60 mm)
            scan_msg.range_max = 30.0     # Maximum range (30 meters)
            scan_msg.ranges = ranges.tolist()

            # Publish the message
            self.publisher_.publish(scan_msg)
            self.get_logger().info('Publishing mock LiDAR scan data')

        except Exception as e:
            self.get_logger().error(f"Error generating mock LiDAR data: {e}")


    def publish_scan(self):
        try:
            # Reads a single line of data from the LiDAR, Decodes it from bytes to a string, then removes extra spaces
            line = self.lidar.readline().decode('utf-8').strip()
            if not line:
                return # If no data is received, it returns (skips this loop cycle)

            # Process the LiDAR data (convert to an array of distances) :  LiDAR data is a CSV string → "1.2, 2.5, 3.0, 0.8", Converts it into a NumPy array of float values
            ranges = np.array([float(x) for x in line.split(',')])

            # Create and fill the LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg() #  Adds a timestamp to the message
            scan_msg.header.frame_id = "lidar_frame" # you're telling the system that the data it will receive (like the angles and distances) are based on the LiDAR sensor's position and orientation on the car.

            # The Hokuyo UST-10LX has a 270° field of view (FOV) by default. avoids the "back" 90° of a full circle (360°), as the sensor is designed for forward-facing scans. To match its actual specs:
            scan_msg.angle_min = -3 * np.pi / 4  # -135° (start angle) 
            scan_msg.angle_max = 3 * np.pi / 4   # +135° (end angle)  
            # from datasheet:
            scan_msg.angle_increment = 0.25 * np.pi / 180  # 0.25° in radians (~0.00436 rad)  
            scan_msg.range_min = 0.06  # 60 mm (minimum measurable distance)  
            scan_msg.range_max = 30.0   # 30 meters (maximum measurable distance)  
            scan_msg.ranges = ranges.tolist() # Assigns the LiDAR range measurements to the ranges field of the LaserScan message.
            # ranges is typically a NumPy array or list containing the distance measurements (in meters) for each angle in the scan, .tolist() converts a NumPy array into a Python list, which is required by the LaserScan message type in ROS 2.

            # Publish the message
            self.publisher_.publish(scan_msg) # Publishes the LaserScan message to the ROS 2 topic.
            self.get_logger().info('Publishing LiDAR scan data') # Logs a message to the console,  Logs a message to the ROS 2 logger indicating that the LiDAR scan data has been published.

        except Exception as e:
            self.get_logger().error(f"Error reading LiDAR: {e}")

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the LidarPublisher node
    node = LidarPublisher()

    # Keep the node running and processing callbacks
    rclpy.spin(node)

    # Clean up the node when it's no longer needed
    node.destroy_node()

    # Shut down the ROS 2 Python client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class RemapperNode(Node):
    def __init__(self):
        super().__init__('remapper_node')
        
        self.get_logger().info('Remapper Node Started')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)

    def odom_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info('Published Odometry Message')

def main(args=None):
    rclpy.init(args=args)
    node = RemapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
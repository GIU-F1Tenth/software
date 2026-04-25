import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from decision.fsm import FSM, StateType


class FSMNode(Node):
    def __init__(self):
        super().__init__("fsm_node")

        self.declare_parameter("states", ["idle"])
        self.declare_parameter("initial_state", "idle")
        self.declare_parameter("objects_topic", "objects")
        self.declare_parameter("output_topic", "fsm_state")

        states_list = self.get_parameter("states").value
        initial_state = self.get_parameter("initial_state").value
        objects_topic = self.get_parameter("objects_topic").value
        output_topic = self.get_parameter("output_topic").value

        state_types = [StateType(s) for s in states_list]
        initial = StateType(initial_state)

        self.fsm = FSM(state_types, initial)

        self.publisher = self.create_publisher(String, output_topic, 10)

        self.subscription = self.create_subscription(
            MarkerArray, objects_topic, self.objects_callback, 10
        )

    def objects_callback(self, msg):
        self.get_logger().debug(f"Received objects: {msg}")
        state_str = self.fsm.run_once(objects=msg.markers)
        self.get_logger().info(f"Current FSM state: {state_str}")
        output_msg = String()
        output_msg.data = state_str
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

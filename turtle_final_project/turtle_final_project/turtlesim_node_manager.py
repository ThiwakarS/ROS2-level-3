import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class TurtleSimNodeManager(Node):
    def __init__(self):
        super().__init__("turtlesim_node_manager")
        self.declare_parameter("managed_node_name",
                               rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter("managed_node_name").value

        service_change_state_name = "/" + node_name + "/change_state"

        self.client = self.create_client(
            ChangeState, service_change_state_name)

    def change_state(self, transition: Transition):
        self.client.wait_for_service()

        request = ChangeState.Request()

        request.transition = transition

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

    def configure_sequence(self):

        # Unconfigured to Inactive
        self.get_logger().info("Trying to switch to configuring")

        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)

        self.get_logger().info("Configuring OK, now inactive")

        # sleep just for the example
        time.sleep(4)

    def activate_sequence(self):
        # Inactive to Active
        self.get_logger().info("Trying to switch to Activate")

        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)

        self.get_logger().info("Activating OK, now Active")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSimNodeManager()
    node.configure_sequence()
    node.activate_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_node_name = "casper_robot_"

    n = 4

    for i in range(n):
        number_node = LifecycleNode(
            package="lifecycle_py",
            executable="casper_lifecycle_server",
            name=robot_node_name+str(i),
            parameters = [{"node_name" : robot_node_name+str(i)}],
            namespace=""
        )

        lifecycle_node_manager = Node(
            package="lifecycle_py",
            executable="lifecycle_node_manager",
            parameters=[
                {"managed_node_name": robot_node_name+str(i)}
            ]
        )

        ld.add_action(number_node)
        ld.add_action(lifecycle_node_manager)

    return ld
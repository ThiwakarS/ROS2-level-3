from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_node_name = "turtle_node"

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )


    turtlesim_node = LifecycleNode(
        package="turtle_final_project",
        executable="turtlesim_action_server",
        name=robot_node_name,
        # parameters = [{"node_name" : robot_node_name+str(i)}],
        namespace=""
    )

    turtlesim_node_manager = Node(
        package="turtle_final_project",
        executable="turtlesim_node_manager",
        parameters=[
            {"managed_node_name": robot_node_name}
        ]
    )

    ld.add_action(turtlesim)
    ld.add_action(turtlesim_node)
    ld.add_action(turtlesim_node_manager)

    return ld
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    servo_node = Node(
        package="servo_node",
        executable="servo_node",
        parameters=["config/servo_node.yaml"],
        output="screen",
    )

    return launch.LaunchDescription([servo_node])
    # return launch.LaunchDescription([container, robot_node, debug_node])
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    servo_node = Node(
        package="servo_node",
        executable="servo_node",
        parameters=["config/servo_node.yaml"],
        output="screen",
    )
    audio_node = Node(
        package="audio_node",
        executable="audio_node",
        output="screen",
    )

    return LaunchDescription([servo_node, audio_node])
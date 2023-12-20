from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="icm20948_ros",
                namespace="icm20948",
                executable="node",
                name="icm20948",
            ),
        ]
    )

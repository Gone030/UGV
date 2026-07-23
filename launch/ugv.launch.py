from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("ugv")) / "config" / "ugv.yaml"

    return LaunchDescription([
        Node(
            package="ugv",
            executable="ugv_control_node",
            name="ugv_control_node",
            output="screen",
            parameters=[str(config)],
        )
    ])

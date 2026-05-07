"""Launch file for contact_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('contact')
    config_file = os.path.join(package_share, 'config', 'contact_node.yaml')

    contact_node = Node(
        package='contact',
        executable='contact_node',
        name='contact_node',
        parameters=[
            config_file,
            {'launched_properly': True}  # Marker to prevent ros2 run usage
        ],
        output='screen',
    )

    return LaunchDescription([contact_node])

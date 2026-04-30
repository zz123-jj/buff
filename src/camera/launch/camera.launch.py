"""ROS 2 camera launch entry point. ROS 2 相机节点的启动入口。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the camera launch description. 构建相机节点的启动描述。

    Returns:
        LaunchDescription: Launch description wiring parameters and respawn behavior. 聚合参数与重启行为的启动描述。
    """

    package_name = 'camera'
    package_share = get_package_share_directory(package_name)
    default_params_file = os.path.join(package_share, 'config', 'param.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to a YAML file with camera parameters.'
    )

    respawn_arg = DeclareLaunchArgument(
        'respawn',
        default_value='true',
        description='Automatically restart the camera node if it exits.'
    )

    respawn_delay_arg = DeclareLaunchArgument(
        'respawn_delay',
        default_value='2.0',
        description='Seconds to wait before respawning the node.'
    )

    camera_node = Node(
        package=package_name,
        executable='camera_node',
        name='camera_publisher',
        output='screen',
        respawn=LaunchConfiguration('respawn'),
        respawn_delay=LaunchConfiguration('respawn_delay'),
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        respawn_arg,
        respawn_delay_arg,
        camera_node,
    ])

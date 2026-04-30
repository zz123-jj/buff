import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    buff_param = os.path.join(
        get_package_share_directory('rm_vision_bringup'),
        'config',
        'buff_param.yaml',
    )

    buff_detector = Node(
        package='buff_detector',
        executable='buff_detector_node',
        name='buff_detector_node',
        parameters=[buff_param],
        output='screen',
    )

    buff_estimator = Node(
        package='buff_estimator',
        executable='buff_estimator_node',
        name='buff_estimator_node',
        parameters=[buff_param],
        output='screen',
    )

    buff_aimer = Node(
        package='buff_aimer',
        executable='buff_aimer_node',
        name='buff_aimer_node',
        parameters=[buff_param],
        output='screen',
    )

    return LaunchDescription([
        buff_detector,
        buff_estimator,
        buff_aimer,
    ])

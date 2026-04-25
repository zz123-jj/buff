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

    buff_predictor = Node(
        package='buff_predictor',
        executable='buff_predictor_node',
        name='buff_predictor_node',
        parameters=[buff_param],
        output='screen',
    )

    buff_solver = Node(
        package='buff_solver',
        executable='buff_solver_node',
        name='buff_solver_node',
        parameters=[buff_param],
        output='screen',
    )

    return LaunchDescription([
        buff_detector,
        buff_predictor,
        buff_solver,
    ])

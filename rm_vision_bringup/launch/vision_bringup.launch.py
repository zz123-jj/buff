import os
import yaml
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():

    # Load parameters
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))
    
    # node_params = os.path.join(
    #     get_package_share_directory('rm_vision_bringup'), 'config', 'rm_auto_aim_param.yaml')
    
    camera_param = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'camera_param.yaml')
    
    # solver_param = os.path.join(
    #     get_package_share_directory('rm_vision_bringup'), 'config', 'auto_aim_solver_param.yaml')
    
    buff_param = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'buff_param.yaml')
    
# 组件容器：将 camera、detector、tracker、solver 放在同一进程

    vision_container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera',
                plugin='CameraPublisher',
                name='camera_publisher',
                parameters=[camera_param]
            ),
            # ComposableNode(
            #     package='armor_detector',
            #     plugin='rm_auto_aim::ArmorDetectorNode',
            #     name='armor_detector_node',
            #     parameters=[node_params],
            #     extra_arguments=[{'use_intra_process_comms': True}]
            # ),
            # ComposableNode(
            #     package='armor_tracker',
            #     plugin='rm_auto_aim::ArmorTrackerNode',
            #     name='armor_tracker_node',
            #     parameters=[node_params],
            #     extra_arguments=[{'use_intra_process_comms': True}]
            # ),
            # ComposableNode(
            #     package='auto_aim_solver',
            #     plugin='AutoAimSolver',
            #     name='auto_aim_solver_node',
            #     parameters=[solver_param],
            #     extra_arguments=[{'use_intra_process_comms': True}]
            # ),
            # ---- buff 能量机关自瞄组件 ----
            ComposableNode(
                package='buff_detector',
                plugin='BuffDetectorNode',
                name='buff_detector_node',
                parameters=[buff_param],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='buff_predictor',
                plugin='BuffPredictorNode',
                name='buff_predictor_node',
                parameters=[buff_param],
                extra_arguments=[{'use_intra_process_comms': True}]
),
            ComposableNode(
                package='buff_solver',
                plugin='BuffSolver',
                name='buff_solver_node',
                parameters=[buff_param],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', f'armor_detector:={launch_params["armor_detector_log_level"]}',
            '--log-level', f'armor_tracker:={launch_params["armor_tracker_log_level"]}',
        ]
    )
    
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    joint_tf_publisher = Node( package='joint_tf_publisher', executable='joint_tf_publisher')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    return LaunchDescription([
        vision_container,
        joint_tf_publisher,
        robot_state_publisher,
    ])

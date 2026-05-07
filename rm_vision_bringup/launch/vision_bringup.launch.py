import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def get_pose(params, key, fallback=None):
    if key in params:
        return params[key]
    if fallback and fallback in params:
        return params[fallback]
    return {'xyz': '"0.0 0.0 0.0"', 'rpy': '"0.0 0.0 0.0"'}

def vector3(value):
    return [float(item) for item in str(value).replace('"', '').split()]

def axis_name(value):
    text = str(value).replace('"', '').strip().lower()
    sign = ''
    if text.startswith(('-', '+')):
        sign = text[0]
        text = text[1:]
    aliases = {
        'x': 'roll',
        'y': 'pitch',
        'z': 'yaw',
    }
    return sign + aliases.get(text, text)

def generate_launch_description():

    # Load parameters
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

    odom2yaw = get_pose(launch_params, 'odom2yaw', 'base2yaw')
    yaw2roll = get_pose(launch_params, 'yaw2roll')
    roll2pitch = get_pose(launch_params, 'roll2pitch', 'yaw2pitch')
    gimbal2camera = get_pose(launch_params, 'gimbal2camera', 'odom2camera')
    gimbal2muzzle = get_pose(launch_params, 'gimbal2muzzle')
    quaternion_to_gimbal = launch_params.get('quaternion_to_gimbal', {})
    gimbal_tf_source = str(launch_params.get('gimbal_tf_source', 'quaternion')).lower()
    gimbal_tf_executables = {
        'quaternion': 'quaternion_tf_publisher',
        'joint': 'joint_tf_publisher',
    }
    if gimbal_tf_source not in gimbal_tf_executables:
        raise ValueError(
            f"Unsupported gimbal_tf_source '{gimbal_tf_source}', use 'quaternion' or 'joint'")
    
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
        output='screen',
        composable_node_descriptions=[
                # ComposableNode(
                #     package='camera',
                #     plugin='CameraPublisher',
                #     name='camera_publisher',
                #     parameters=[camera_param],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
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
                package='buff_estimator',
                plugin='BuffEstimatorNode',
                name='buff_estimator_node',
                parameters=[buff_param],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='buff_aimer',
                plugin='BuffAimer',
                name='buff_aimer_node',
                parameters=[buff_param],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
       
    )
    
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' camera_xyz:=', gimbal2camera['xyz'],
        ' camera_rpy:=', gimbal2camera['rpy'],
        ' muzzle_xyz:=', gimbal2muzzle['xyz'],
        ' muzzle_rpy:=', gimbal2muzzle['rpy']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 1000.0,
        }]
    )

    gimbal_tf_publisher = Node(
        package='gimbal_tf_publisher',
        executable=gimbal_tf_executables[gimbal_tf_source],
        name=f'{gimbal_tf_source}_tf_publisher',
        output='screen',
        parameters=[{
            'yaw_xyz': vector3(odom2yaw['xyz']),
            'yaw_rpy': vector3(odom2yaw['rpy']),
            'roll_xyz': vector3(yaw2roll['xyz']),
            'roll_rpy': vector3(yaw2roll['rpy']),
            'pitch_xyz': vector3(roll2pitch['xyz']),
            'pitch_rpy': vector3(roll2pitch['rpy']),
            'input_x_axis': axis_name(quaternion_to_gimbal.get('input_x_axis', 'roll')),
            'input_y_axis': axis_name(quaternion_to_gimbal.get('input_y_axis', 'pitch')),
            'input_z_axis': axis_name(quaternion_to_gimbal.get('input_z_axis', 'yaw')),
            'decomposition_order': quaternion_to_gimbal.get('decomposition_order', 'rpy'),
            'roll_source': quaternion_to_gimbal.get('roll_source', 'roll'),
            'yaw_source': quaternion_to_gimbal.get('yaw_source', 'yaw'),
            'pitch_source': quaternion_to_gimbal.get('pitch_source', 'pitch'),
            'roll_sign': float(quaternion_to_gimbal.get('roll_sign', 1.0)),
            'yaw_sign': float(quaternion_to_gimbal.get('yaw_sign', 1.0)),
            'pitch_sign': float(quaternion_to_gimbal.get('pitch_sign', 1.0)),
        }]
    )


    return LaunchDescription([
        vision_container,
        gimbal_tf_publisher,
        robot_state_publisher,
    ])

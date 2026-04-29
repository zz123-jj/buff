from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("rm_vision_bringup"),
        "config",
        "auto_buff.yaml",
    ])
    camera_config = PathJoinSubstitution([
        FindPackageShare("rm_vision_bringup"),
        "config",
        "camera_param.yaml",
    ])
    gimbal_urdf = PathJoinSubstitution([
        FindPackageShare("rm_gimbal_description"),
        "urdf",
        "rm_gimbal.urdf.xacro",
    ])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", gimbal_urdf]),
        }],
        output="screen",
    )
    joint_tf_publisher = Node(
        package="gimbal_tf_publisher",
        executable="joint_tf_publisher",
        name="joint_tf_publisher",
        output="screen",
    )
    container = ComposableNodeContainer(
        name="rm_vision_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="rm_buff_detector",
                plugin="rm_buff_detector::BuffDetectorNode",
                name="buff_detector",
                parameters=[config],
            ),
            ComposableNode(
                package="rm_buff_solver",
                plugin="rm_buff_solver::BuffSolverNode",
                name="buff_solver",
                parameters=[config],
            ),
            ComposableNode(
                package="rm_buff_tracker",
                plugin="rm_buff_tracker::BuffTrackerNode",
                name="buff_tracker",
                parameters=[config],
            ),
            ComposableNode(
                package="rm_buff_aimer",
                plugin="rm_buff_aimer::BuffAimerNode",
                name="buff_aimer",
                parameters=[config],
            ),
            ComposableNode(
                package="camera",
                plugin="CameraPublisher",
                name="camera_publisher",
                parameters=[camera_config],
            ),

        ],
        output="screen",
    )

    return LaunchDescription([robot_state_publisher, joint_tf_publisher, container])

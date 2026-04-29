from launch import LaunchDescription
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
            ComposableNode(
                package="joint_tf_publisher",
                plugin="JointTFPublisher",
                name="joint_tf_publisher",
            ),

        ],
        output="screen",
    )

    return LaunchDescription([container])

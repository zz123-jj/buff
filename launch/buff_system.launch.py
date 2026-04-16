from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    detector_params = PathJoinSubstitution([
        FindPackageShare('buff_detector'),
        'config',
        'buff_detector.yaml'
    ])

    predictor_params = PathJoinSubstitution([
        FindPackageShare('buff_predictor'),
        'config',
        'buff_predictor.yaml'
    ])

    return LaunchDescription([
        # 启动检测节点
        Node(
            package='buff_detector',
            executable='buff_detector',
            name='buff_detector_node',
            output='screen',
            parameters=[detector_params]
        ),
        
        # 启动预测节点
        Node(
            package='buff_predictor',
            executable='buff_predictor_node',
            name='buff_predictor_node',
            output='screen',
            parameters=[predictor_params]
        )
    ])

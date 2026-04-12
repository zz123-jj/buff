from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动检测节点
        Node(
            package='buff_detector',
            executable='buff_detector',
            name='buff_detector_node',
            output='screen'
        ),
        
        # 启动预测节点
        Node(
            package='buff_predictor',
            executable='buff_predictor_node',
            name='buff_predictor_node',
            output='screen',
            parameters=[{
                'delta_t': 0.2,             # 预测时间（秒）
                'camera_fps': 25,            # 相机帧率
                'min_omega': 1.800,          # 最低角速度
                'period_coefficient': 1.0,   # 周期系数
                'target_frame': 'camera_optical_frame',
                'camera_frame': 'camera_optical_frame',
                'enable_angle_log': True,
                'angle_log_path': 'log/angle_time.csv'
            }]
        )
    ])

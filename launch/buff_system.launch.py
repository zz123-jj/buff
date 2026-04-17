from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
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
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=[
                '/home/caleb/onnxruntime/lib:'
                '/home/caleb/.local/lib/python3.10/site-packages/nvidia/cuda_runtime/lib:'
                '/home/caleb/.local/lib/python3.10/site-packages/nvidia/cuda_nvrtc/lib:'
                '/home/caleb/.local/lib/python3.10/site-packages/nvidia/cublas/lib:'
                '/home/caleb/.local/lib/python3.10/site-packages/nvidia/cudnn/lib:'
                '/home/caleb/.local/lib/python3.10/site-packages/nvidia/curand/lib:'
                '/home/caleb/.local/lib/python3.10/site-packages/nvidia/cufft/lib:'
                '/usr/local/cuda/lib64:',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value='')
            ]
        ),
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

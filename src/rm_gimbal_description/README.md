# rm_gimbal_description
RoboMaster 视觉自瞄系统所需的 URDF

该项目为 [rm_vision](https://github.com/chenjunnn/rm_vision) 的子模块

## 坐标系定义

单位和方向请参考 https://www.ros.org/reps/rep-0103.html

odom: 以云台中心为原点的惯性系，由 quaternion_tf_publisher 发布

base_link: 表述 roll 轴后的底盘/云台基座坐标系，由 quaternion_tf_publisher 发布

gimbal_yaw_link: 表述云台 yaw 轴坐标系，由 quaternion_tf_publisher 发布

gimbal_pitch_link: 表述云台 pitch 轴坐标系，是 URDF 固定链路的根

camera_joint: 表述相机到 gimbal_pitch_link 的固定变换关系

camera_optical_joint: 表述以 z 轴为前方的相机坐标系转换为 x 轴为前方的相机坐标系的旋转关系

bullet_muzzle_joint: 表述子弹出膛位置到 gimbal_pitch_link 的固定变换关系

## 使用方法

修改 [urdf/rm_gimbal.urdf.xacro](urdf/rm_gimbal.urdf.xacro) 中的相机和枪口固定变换参数

xyz 与 rpy 对应机器人云台上相机/枪口到 pitch 轴坐标系的平移与旋转关系，可以由机械图纸测量得到，或在机器人上直接测量。base/yaw/pitch 的安装位置在 `rm_vision_bringup/config/launch_params.yaml` 中配置，并由 quaternion_tf_publisher 发布动态 TF。
# rm_gimbal_description

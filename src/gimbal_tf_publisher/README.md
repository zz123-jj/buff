# tf树生成模块

### 订阅 /quaternion
- 消息类型：geometry_msgs/msg/Quaternion
- 默认发布链路：odom -> gimbal_yaw_link -> gimbal_roll_link -> gimbal_pitch_link
- `joint_tf_publisher` 仍可作为 `/joint_states` 备用输入，默认 bringup 不启动

### 发布 /tf
- 用于其他模块构建坐标系

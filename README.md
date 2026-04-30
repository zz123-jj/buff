# 能量机关自瞄模块

## 数据流

```text
/image_raw
  -> buff_detector      识别目标扇叶/R 标，发布 2D 观测
  -> buff_estimator     估计 3D 状态，拟合大符速度参数
  -> buff_aimer         预测命中点，解弹道，发布云台控制量
  -> /autoaim/target
```

## buff_detector

`buff_detector_node` 订阅 `/image_raw`，第一帧使用 OpenVINO YOLOPose 初始化目标扇叶和 R 标，后续用传统视觉跟踪。

发布 `/buff/target`，内容包括：

1. 图像时间戳
2. 是否正在跟踪
3. 能量机关模式
4. 旋转方向
5. 目标扇叶中心像素坐标
6. R 标中心像素坐标
7. 像素半径

模型默认路径为 `model/yolo11_buff_int8.xml`，默认 OpenVINO 设备为 `GPU`。

## buff_estimator

原 `buff_predictor` 已重命名为 `buff_estimator`，因为这个节点的主要职责不是直接输出云台预测量，而是把 detector 的 2D 观测估计成后续瞄准需要的 3D 状态。

`buff_estimator_node` 订阅：

1. `/buff/target`
2. `/camera_info`
3. TF：`odom <- camera_optical_frame`

发布 `/buff/aiming_data`，内容包括：

1. R 标在相机系和 odom 系下的 3D 坐标
2. 目标扇叶在 odom 系下的 3D 坐标
3. 当前半径、角度、旋转方向
4. 大符速度函数参数 `v(t)=A*sin(omega*t+phi)+b`
5. 拟合窗口时间和样本数量

小符模式下主要做坐标估计；大符模式下还会收集角速度样本并用 Ceres 拟合速度曲线。

## buff_aimer

原 `buff_solver` 已重命名为 `buff_aimer`，因为这个节点最终负责把估计状态转换成自瞄控制输出。

`buff_aimer_node` 订阅：

1. `/buff/aiming_data`

发布 `/autoaim/target`，内容包括 yaw/pitch 绝对瞄准角、目标距离和发射命令。

工作流程：

1. 读取当前目标 3D 状态和大符速度拟合参数。
2. 根据消息延迟、子弹飞行时间和射击延迟预测命中时刻的目标位置。
3. 迭代求解弹道飞行时间。
4. 与 buff2 一样，直接输出 `frames.target` 坐标系下的绝对 yaw/pitch，不再做云台相对角、限幅或限速处理。

## Debug

查看主要输出：

```bash
ros2 topic hz /buff/target
ros2 topic hz /buff/aiming_data
ros2 topic hz /autoaim/target
```

Foxglove：

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

# Open DaHeng Camera

## 该分支基于dev_old添加了yaml参数的功能，原分支名：kanade

## INTRO

- 基于 ROS 2 Jazzy 的大恒工业相机驱动节点，提供图片采集、话题发布与基础调试工具。
- 通过 YAML 配置集中管理相机参数，支持快速调整分辨率、曝光、话题命名与标定数据。
- 配套提供图像采集与棋盘标定脚本，可在离线环境下完成整套相机标定流程。

## FEATURES

1. 现在代码内部支持了相机启动失败的时候失败重试(5 times)
2. 支持yaml设置参数了，包括topic name，校准，计时器时长，显示imshow...
3. 在yaml中写show_cam=true就可以弹出imshow窗口了。
4. 现在可以在imshow的时候按下q/Q退出imshow。
5. 在imshow的时候按下d可以保存当前帧到capture_SN目录下。
6. 在imshow的时候按下r可以开始或者停止录制视频，视频会保存到record_SN.avi
7. 现在在启动的时候可以输出具体参数（要在yaml里打开）
8. MER-131 相机疑似不支持fps控制等等(现在300fps左右)，在老版本里会因为强制设置这些参数导致启动失败,现在以及修复了
9. **支持以ROS2组件化的方式启动相机节点（详见启动方法2），且通过unique_ptr消息实现ROS2的零拷贝消息传输**
10. 移除了ciallo！

## Build & Run

1. 安装 ROS 2 Jazzy（或兼容发行版），并确保 `colcon`、`rosdep` 环境可用。
2. 在工作空间根目录执行依赖安装与编译：

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select camera --symlink-install
   # the --symlink-install is RECOMMANDED! it will make debugging easier.
   ```

3. 载入 ROS 环境与工作空间：

   ```bash
   source /opt/ros/jazzy/setup.zsh  # 或对应发行版的 setup 脚本
   source install/setup.zsh
   ```

4. 启动相机节点（支持 respawn）：
    - 方法1：单功能包启动
   ```bash
   ros2 launch camera camera.launch.py
   ```
   此方法将读取 `config/param.yaml` 作为参数文件启动相机节点。

    - 方法2：通过 rm_vision 工作空间启动（推荐）
   ```bash
   ros2 launch rm_vision_bringup vision_bringup.launch.py
   ```
   此方法将读取 `rm_vision_bringup/config/camera_param.yaml` 作为参数文件启动相机节点。
   **图像消息数据量较大，若要订阅图像消息，请确保订阅者和相机节点在同一进程中运行。**
   **rm_vision工作空间中启动会启用ROS2组件化功能，可以使相机节点与订阅者在同一进程中运行，避免消息阻塞。**

## Arguments

| 键/段落                                         | 说明                                   |
|------------------------------------------------|----------------------------------------|
| `config/param.yaml`                            | 主要配置文件，启动时通过 `params_file` 传入。 |
| `device.serial`                                | 相机序列号。                           |
| `sensor`、`roi`、`binning`、`exposure`、`gain` | 分辨率与采集参数。                     |
| `camera_topic` / `camera_info_topic` / `frame_id` | ROS 2 话题与坐标系命名。            |
| `timers`、`*_qos`                              | 图像与 CameraInfo 发布频率及 QoS 配置。|
| `calibration`                                  | 相机内参、畸变参数，支持标定后直接粘贴。|
| `show_cam`                                     | 启用窗口预览。                         |
| `show_params`                                  | 启动时打印当前全部参数。               |
| `is_rotate`、`rotate_type`                     | 开启图像旋转与角度选择。               |

## Camera Calibration

- 默认使用 12×9 内角点棋盘格（列数 12、行数 9）进行标定，脚本会自动扫描最佳尺寸；如尺寸固定，可通过 `--known-size` 指定，例如 `11x8`。
- 标定脚本位于仓库根目录的 `calibrate.py`，支持多目录输入、并行处理与可视化导出。

**步骤 1：启用取像窗口并采集样图**

- 编辑 `config/param.yaml`，确保：

  ```yaml
  show_cam: true
  ```

- 启动节点后会弹出原始图像窗口：
  - 按 `d` 保存当前帧到 `capture_{SN}` 目录（`SN` 为相机序列号，保存目录位于启动时的工作目录）。
  - 按 `q` 或 `Q` 退出窗口。
- 建议采集至少 12 张图片，覆盖不同距离、方向和棋盘位置，避免全部样张过于接近或模糊。

    | 拍摄类型                                   | 目的           | 说明           |
    |--------------------------------------------|----------------|----------------|
    | 近距离、大棋盘（占满屏幕 80~90%）          | 估计焦距、主点 | 2~3 张         |
    | 中等距离、棋盘偏中心                        | 综合视角       | 5~8 张         |
    | 远距离、小棋盘（占画面 30~50%）            | 捕捉畸变       | 3~5 张         |
    | 角落位置（棋盘在四角）                      | 估计畸变对称性 | 每个角至少 1 张 |
    | 俯斜姿态（绕 X/Y/Z 轴旋转）                 | 提供不同透视   | 若干张         |

    | 误区                       | 后果                             |
    |----------------------------|----------------------------------|
    | 每张都让棋盘格居中并占满画面 | 畸变估计不准，重投影误差大       |
    | 棋盘格太小或太远           | 像素级角点分辨率差，精度下降     |
    | 拍太近导致模糊             | 棋盘检测失败或误差大             |
    | 光照太亮导致白格过曝       | 棋盘角点检测不出                 |

**步骤 2：运行标定脚本**

- 使用采集到的目录执行脚本，可直接采用默认参数：

  ```bash
  python3 calibrate.py <capture_directory> \
    --sizes 6-14,6-14 --sample 5 --min-hits 2 \
    --detect-scale 0.5 --lite-pre --jobs 8 --sb-first --log debug
  ```

- 主要参数含义：
  - `--sizes`：扫描内角点范围；
  - `--sample` / `--min-hits`：快速抽样判定棋盘尺寸；
  - `--detect-scale`：缩放后检测提升鲁棒性；
  - `--jobs`：并行线程数量；
  - `--sb-first`：优先使用 OpenCV `findChessboardCornersSB`；
  - `--lite-pre`：扫描阶段采用轻量预处理；
  - `--log`：日志等级（`debug` 便于观察命中情况）。

**步骤 3：写回标定结果**

- 成功标定后会输出示例：

  ```bash
  [USE] FGV22100004_20251011_234943.png: OK (SB); total_used=60
  [CALIB] ret=0.28726341656176163 time=2830.8 ms
  [RMSE] 0.2873px over 5368 points

  ===== YAML =====
  calibration:
      k: [...]
      d: [...]
      model: "plumb_bob"
      compute_rectified_p: true
  ```

- 复制 `YAML` 段落到 `config/param.yaml` 的 `calibration` 节，重新启动节点即可应用新内参。

## 历史问题记录

- 由于大恒 API 截取图像函数的缺陷，首次插入相机时可能出现返回错误 “-11 用户写入值越界”。目前通过 `ros2 launch` 的 `respawn` 功能自动重启节点规避，可在 `camera.launch.py` 中调整重启延迟（默认无额外延迟）。

``` bash
[camera-1] device: SN:FGV22100004 NAME:MER2-135-150U3C-L(FGV22100004) TYPE:GX_DEVICE_CLASS_U3V
[camera-1] DHCamera Sensor: 1280 X 1024
[camera-1] WARNING: Logging before InitGoogleLogging() is written to STDERR
[camera-1] I20251012 01:23:42.088785 193014 CamWrapper.cpp:30] ROI_X set failed!
[camera-1] I20251012 01:23:42.091938 193014 CamWrapper.cpp:30] ROI_Y set failed!
[camera-1] E20251012 01:23:42.191282 193014 CamWrapper.cpp:330] failed to set some parameters!
[camera-1] [WARN] [1760203422.191323074] [camera_publisher]: Camera not ready, retry 1/5 ...
[camera-1] [WARN] [1760203422.691618291] [camera_publisher]: Camera not ready, retry 2/5 ...
[camera-1] [WARN] [1760203423.191777322] [camera_publisher]: Camera not ready, retry 3/5 ...
[camera-1] [WARN] [1760203423.691931805] [camera_publisher]: Camera not ready, retry 4/5 ...
[camera-1] [WARN] [1760203424.192104112] [camera_publisher]: Camera not ready, retry 5/5 ...
[camera-1] [ERROR] [1760203424.692299672] [camera_publisher]: Failed to start camera, exiting.
[ERROR] [camera-1]: process has died [pid 193014, exit code 1, cmd '/home/inubashiri/04_Cam/install/camera/lib/camera/camera --ros-args -r __node:=camera_publisher --params-file /home/inubashiri/04_Cam/install/camera/share/camera/
config/param.yaml'].
Task exception was never retrieved
future: <Task finished name='Task-3' coro=<ExecuteLocal.__execute_process() done, defined at /opt/ros/jazzy/lib/python3.12/site-packages/launch/actions/execute_local.py:544> exception=TypeError("'>' not supported between instances
 of 'LaunchConfiguration' and 'float'")>
Traceback (most recent call last):
  File "/opt/ros/jazzy/lib/python3.12/site-packages/launch/actions/execute_local.py", line 611, in __execute_process
    if self.__respawn_delay is not None and self.__respawn_delay > 0.0:
                                            ^^^^^^^^^^^^^^^^^^^^^^^^^^
TypeError: '>' not supported between instances of 'LaunchConfiguration' and 'float'
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
```

## WARNING

- 有些老相机型号（如 MER-131）可能不支持部分参数设置（如 FPS 控制），可以在CamWrapper.cpp的init函数中添加相应的容错处理以避免启动失败。
- 示范已经在CameraWrapper 中添加

## TODO

- [ ] 支持双相机拓展。
- [ ] 修复大恒 API 首次插入相机必然失败的问题。

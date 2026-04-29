# RM Vision ROS2 Workspace

This workspace carries the buff pipeline context as ROS2 packages, messages, parameters, and launch files instead of relying on the original non-ROS `tasks/auto_buff` code.

## Packages

- `rm_vision_interfaces`: shared messages.
- `rm_buff_common`: shared math and trajectory helpers.
- `rm_buff_detector`: `/image_raw` -> `/buff/detection`.
- `rm_buff_solver`: `/buff/detection` + TF `world <- camera` -> `/buff/pose`.
- `rm_buff_tracker`: `/buff/pose` -> `/buff/target`, with SmallTarget/BigTarget EKF tracking and big-rune online sine speed fitting.
- `rm_buff_aimer`: `/buff/target` + TF `world <- gimbal` -> `/gimbal_cmd`.
- `rm_vision_bringup`: launch and YAML config.

## Pipeline

```text
/image_raw
  -> rm_buff_detector/BuffDetectorNode
  -> /buff/detection
  -> rm_buff_solver/BuffSolverNode
  -> /buff/pose
  -> rm_buff_tracker/BuffTrackerNode
  -> /buff/target
  -> rm_buff_aimer/BuffAimerNode
  -> /gimbal_cmd
```

`rm_buff_tracker` owns the target state model. Small mode tracks the fixed-speed model; big mode
tracks `[R_yaw, R_v_yaw, R_pitch, R_dis, yaw, roll, speed, A, omega, phi]` and also fits
`speed(t)=A*sin(omega*t+phi)+C` online from EKF speed estimates. `rm_buff_aimer` only applies the
tracker state to the requested future time, solves the trajectory, and converts the absolute aim
angle into gimbal-relative offsets.

## Build

On this arm64 machine, Intel's Ubuntu apt repository exposes the OpenVINO meta packages, but the
matching `libopenvino-dev-*` architecture package is not installable. The detector package can build
against the Python wheel instead:

```bash
python3 -m pip install --user openvino-dev==2024.6.0
```

The detector CMake file auto-detects the wheel's `OpenVINOConfig.cmake` and embeds an install RPATH
to the wheel's `openvino/libs` directory, so no extra `LD_LIBRARY_PATH` export is needed after build.

Source ROS2 first, then:

```bash
source /opt/ros/humble/setup.zsh
cd /home/calebevans/RM/sp_vision_25/ros_ws
colcon build --packages-up-to rm_vision_bringup
```

Use `--packages-select <package>` when dependencies are already built or installed.

## Run

```bash
source /opt/ros/humble/setup.zsh
source /home/calebevans/RM/sp_vision_25/ros_ws/install/setup.zsh
ros2 launch rm_vision_bringup auto_buff.launch.py
```

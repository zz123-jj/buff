# buff_aimer

`buff_aimer` 接收 `/buff/aiming_data`，预测命中时刻目标位置，解弹道，并发布最终云台控制量。

输入：

- `/buff/aiming_data`
- `/joint_states`，可选

输出：

- `/autoaim/target`
- `/autoaim/debug`

仿真没有 `/joint_states` 时默认使用 yaw/pitch 为 0 继续发布；真车需要强制等待关节角时，将 `gimbal.require_joint_states` 设为 `true`。

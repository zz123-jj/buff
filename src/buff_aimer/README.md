# buff_aimer

`buff_aimer` 接收 `/buff/world_model`，用内部 Rodrigues 旋转器预测命中时刻目标位置，解弹道，并发布最终绝对瞄准角。

输入：

- `/buff/world_model`

输出：

- `/autoaim/target`
- `/autoaim/debug`

`/autoaim/target` 中的 yaw/pitch 与 buff2 保持一致，直接使用 `frames.target` 坐标系下的绝对角；节点不再订阅 `/joint_states`，也不再做云台相对角、限幅或限速处理。

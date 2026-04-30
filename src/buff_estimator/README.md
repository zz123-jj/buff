# buff_estimator

`buff_estimator` 接收 `buff_detector` 的 2D 观测，结合相机内参和 TF，把目标估计到 3D 空间，并输出给自瞄节点使用的 `/buff/aiming_data`。

它同时负责大符速度曲线拟合，所以内部仍保留 `Big_Buff_Predictor` 这类预测/拟合实现；包名改为 estimator 是为了更准确表达节点的接口职责。

输入：

- `/buff/target`
- `/camera_info`
- TF：`odom <- camera_optical_frame`

输出：

- `/buff/aiming_data`

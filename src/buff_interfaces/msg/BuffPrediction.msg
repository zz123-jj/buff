# 预测节点发布的预测信息

std_msgs/Header header
float32 angle_delta         # 弧度差（预测角度 - 当前角度）
bool is_fitted              # 是否已完成拟合
int32 frame_number          # 预测时的帧号

geometry_msgs/Point r_center_odom

float32 current_angle

# 速度正弦函数参数 dy = a·sin(ω·x + φ) + b
float32 sin_a               # 振幅
float32 sin_omega           # 角频率 (rad/frame)
float32 sin_phi             # 相位 (rad)
float32 sin_b               # 线性系数 (rad/frame)


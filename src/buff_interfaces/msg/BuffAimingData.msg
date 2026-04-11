std_msgs/Header header

# 1. 状态位
bool    is_tracking       # 是否拟合成功
int32   frame_number      # 帧号

# 2. 3D 坐标 (明确这是 R 标圆心)
float64 r_center_x_3d        # odom系 R标 X
float64 r_center_y_3d        # odom系 R标 Y
float64 r_center_z_3d        # odom系 R标 Z

# 3. 原始识别信息
float64 pixel_r_center_x  
float64 pixel_r_center_y  
float64 pixel_radius

# 4. 速度正弦函数参数 dy = a·sin(ω·x + φ) + b
float32 sin_a
float32 sin_omega
float32 sin_phi
float32 sin_b


float32 target_x_3d            #odom系 目标 X
float32 target_y_3d            #odom系 目标 Y
float32 target_z_3d            #odom系 目标 Z

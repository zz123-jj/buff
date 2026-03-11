std_msgs/Header header

# 1. 状态位
bool    is_tracking       # 是否拟合成功
int32   frame_number      # 帧号

# 2. 3D 坐标 (明确这是 R 标圆心)
float64 r_center_x_3d        # odom系 R标 X
float64 r_center_y_3d        # odom系 R标 Y
float64 r_center_z_3d        # odom系 R标 Z

# 3. 预测量
float64 angle_delta       # 预测的角度增量

# 4. 原始识别信息
float64 pixel_r_center_x  
float64 pixel_r_center_y  
float64 pixel_radius


float32 target_x_3d
float32 target_y_3d
float32 target_z_3d

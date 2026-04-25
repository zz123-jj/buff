std_msgs/Header header

bool is_tracking       # 是否拟合成功

int8 is_bigbuff          #-1=small buff 1=bigbuff

int8 spin_direction        # 旋转方向: 0=unknown, -1=anticlockwise(人眼逆时针), 1=clockwise(顺时针)

float32 r_x_3d
float32 r_y_3d
float32 r_z_3d


float32 r_cam_x_3d         #camera系 R标 X
float32 r_cam_y_3d         #camera系 R标 Y
float32 r_cam_z_3d         #camera系 R标 Z

float32 depth

float32 target_x_3d            # 目标 X
float32 target_y_3d            # 目标 Y
float32 target_z_3d            # 目标 Z

#速度正弦函数参数 dy = a·sin(ω·x + φ) + b
float32 sin_a
float32 sin_omega
float32 sin_phi
float32 sin_b
float64 fit_start_time_sec      # 速度函数计算的开始时间
#用于调试
float32 fit_buffer_duration_sec # 触发拟合时累计的数据时长（秒）
int32   fit_data_point_count    # 触发拟合时使用的数据点数量

float32 filter_radius
float32 angle
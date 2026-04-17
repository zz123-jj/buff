# 检测节点发布的目标信息

std_msgs/Header header
float32 radius              # 旋转半径
bool buff_type               #0=small buff 1=bigbuff
bool is_tracking            # 是否在跟踪
uint8 spin_direction        # 旋转方向: 0=unknown, 1=anticlockwise(逆时针), 2=clockwise(顺时针)

float32 target_center_x     # 扇叶框中心x坐标
float32 target_center_y     # 扇叶框中心y坐标

float32 r_center_x          # R框中心x坐标
float32 r_center_y          # R框中心y坐标

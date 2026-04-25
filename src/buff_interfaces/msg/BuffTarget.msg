# 检测节点发布的目标信息
std_msgs/Header header
bool is_tracking            # 是否在跟踪

int8 is_bigbuff          #0=small buff 1=bigbuff

int8 spin_direction        # 旋转方向: 0=unknown, -1=anticlockwise(人眼逆时针), 1=clockwise(顺时针)

float32 target_center_x     # 扇叶框中心x坐标
float32 target_center_y     # 扇叶框中心y坐标

float32 r_center_x          # R框中心x坐标
float32 r_center_y          # R框中心y坐标

float32 radius              # 旋转半径

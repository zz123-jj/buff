# 检测节点发布的目标信息
std_msgs/Header header
bool is_tracking            # 是否在跟踪

int8 is_bigbuff            # 0=unknown, -1=small buff, 1=big buff

bool has_target_keypoints   # 是否包含 YOLOPose 输出的目标四点
float32[8] target_keypoints # x0 y0 x1 y1 x2 y2 x3 y3
float32 pose_confidence     # YOLOPose 置信度
int32 yolo_target_count     # 当前帧 YOLOPose/NMS 后候选目标数量

#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <cstdint>

struct BuffDetectorConfig
{
    std::string model_path = "src/buff_detector/model/Fan.onnx";
    bool use_cuda = false;
    float confidence_threshold = 0.5f;
    float iou_threshold = 0.5f;
    bool debug_mode = true;
    float inside_shade_rate = 0.7f;
    float outside_shade_rate = 1.39f;
    cv::Scalar lower_hsv = cv::Scalar(0, 40, 220);
    cv::Scalar upper_hsv = cv::Scalar(70, 255, 255);
    int dilate_kernel_size = 7;
    int max_lost_frame = 5;
};

float euclidean_distance(const cv::Point2f& p1, const cv::Point2f& p2); // 计算两点间的欧氏距离
cv::Point2f rotate_point(
    const cv::Point2f& point, const cv::Point2f& center, float angle_rad
); // 一点绕中心点旋转指定弧度

enum class IoU_Type
{
    IoU,
    GIoU,
    DIoU,
    CIoU
};

enum class FanBladeState
{
    target,    // 目标扇叶
    unlighted, // 未点亮扇叶
    shotted,   // 已击打扇叶
    unknow      // 无状态
};

enum class BuffType
{
    small_buff, // 小能量机关 
    big_buff    // 大能量机关
};

class RotationRectangle
{
private:
    std::vector<cv::Point2f> points_; // 四个顶点

public:
    RotationRectangle(const cv::Mat& points, const cv::Point2f& R_Box_center);          // 构造函数
    static cv::Point2f get_line_center(const cv::Point2f& p1, const cv::Point2f& p2);  // 计算线段中心点
    cv::Point2f get_center_2f() const;                                                  // 计算旋转矩形中心点
    cv::Point2i get_center_2i() const;                   // 计算旋转矩形中心点（整数类型）
    float get_width() const;                             // 计算旋转矩形宽度
    float get_height() const;                            // 计算旋转矩形高度
    float get_area() const;                              // 计算旋转矩形面积
    float get_dist_top(const cv::Point2f& point) const;    // 获取顶边中点到一点的距离
    float get_dist_bottom(const cv::Point2f& point) const; // 获取底边中点到一点的距离
    const std::vector<cv::Point2f>& get_points() const { return points_; } // 获取四个顶点
    void move_by(float x, float y);                                        // 根据x，y值进行移动
};

class BBox
{
private:
    cv::Point2f point_min_; // 左上角顶点
    cv::Point2f point_max_; // 右下角顶点

public:
    BBox(float x_min, float y_min, float x_max, float y_max) : point_min_(x_min, y_min), point_max_(x_max, y_max) {}
    BBox(const std::vector<BBox>& boxes);                   // 用多个BBox构造一个BBox（包围盒）
    BBox() : point_min_(0, 0), point_max_(0, 0) {}          // 默认构造函数定点坐标都设为0
    float operator&(const BBox& other) const;               // 计算两个BBox的重合部分面积(交集)
    float operator|(const BBox& other) const;               // 计算两个BBox的总面积(并集)
    float operator^(const BBox& other) const;               // 计算IoU(重合部分面积与总面积的比值)
    BBox boundof(const BBox& other) const;                  // 计算box和other的最小外接矩形
    cv::Point2f get_center_2f() const;                      // 计算BBox中心点
    cv::Point2i get_center_2i() const;                      // 计算BBox中心点（整数类型）
    cv::Point2f get_point_min() const;                      // 获取左上角顶点
    cv::Point2f get_point_max() const;                      // 获取右下角顶点
    float get_area() const;                                 // 计算BBox面积
    float get_width() const;                                // 计算BBox宽度
    float get_height() const;                               // 计算BBox高度
    BBox copy_bbox_by_center(cv::Point2f new_center) const; // 通过中心点复制BBox
    void move_by(float x, float y);                         // 根据x，y值进行移动
};

float IoU(const BBox& box1, const BBox& box2);
float GIoU(const BBox& box1, const BBox& box2);
float DIoU(const BBox& box1, const BBox& box2);
float CIoU(const BBox& box1, const BBox& box2);
std::vector<std::pair<BBox, float>> compare_by_IoU(
    const BBox& last_box, const std::vector<BBox>& boxes,
    const IoU_Type& iou_type = IoU_Type::IoU
); // 根据IoU值比较并排序目标列表

struct FanBlade
{
    BBox box;                                  // 扇叶框
    FanBladeState state = FanBladeState::unknow; // 扇叶状态
    int id = -1;                               // 扇叶ID
};

class BuffDetector
{
private:
    BBox current_R_box_;                                    // 当前R标框
    BBox last_R_box_;                                       // 上一帧R标框
    std::vector<FanBlade> target_blades_;                   // 目标扇叶列表
    std::vector<FanBlade> blade_list_;                      // 所有扇叶框
    float buff_radius_ = 0.0f;                              // 能量机关半径（R标到扇叶中心的距离）
    int lighted_blade_num_ = 0;                             // 亮起的扇叶数量
                            // 记录亮起过的最大扇叶数量，用于判断是否结束
    int lost_frame_count_ = 0;                              // 目标丢失帧数
    uint8_t spin_direction_ = 0;                            // 旋转方向: 0=unknown, -1=人眼anticlockwise, 1=人眼clockwise
    uint8_t pending_spin_direction_ = 0;                    // 候选方向
    int pending_direction_count_ = 0;                       // 候选方向连续计数
    int direction_confirm_frames_ = 3;                      // 连续确认帧数（稳定优先）
    float last_target_angle_ = 0.0f;                        // 上一帧目标角度
    bool has_last_target_angle_ = false;                    // 是否已初始化上一帧目标角度
    float min_valid_angle_step_ = 0.003f;                   // 有效角度变化下限（抑制抖动）
                                    // 调试帧图像
    BuffType buff_type_ = BuffType::small_buff;             // 能量机关类型
    BuffDetectorConfig config_;

    bool detect_by_yolo(cv::Mat& frame);                            // 使用yolo模型获取扇叶和R标初始位置
    bool preprocess_image(cv::Mat& frame);                          // 获取预处理后的二值图
    bool update_R_box(cv::Mat& image, bool is_init = false);        // 获取R标框
    bool update_fan_blades(cv::Mat& image);                         // 检测扇叶位置
    void reset_spin_direction_state();                               // 重置方向状态机
    void update_spin_direction_by_target_id();                       // 根据目标扇叶ID更新方向
    

public:
    BuffDetector() = default;
    void set_config(const BuffDetectorConfig& config) { config_ = config; }

    bool init(cv::Mat& frame);                                               // 初始化检测器
    bool update(cv::Mat& frame);                                             // 更新检测结果
    BBox get_first_target_blade_bbox() const                                  // 获取首个目标扇叶框
    {return target_blades_.empty() ? BBox() : target_blades_.front().box;}
    const std::vector<FanBlade>& get_target_fan_blades() const { return target_blades_; }
    BBox get_current_R_box() const { return current_R_box_; }                // 获取当前R标框
    float get_radius() const { return buff_radius_; }                             // 获取能量机关半径
    uint8_t get_spin_direction() const { return spin_direction_; }
    cv::Mat debug_frame_;             // 获取调试图像
    int max_lighted_blade_num_ = 0; 
    BuffType get_buff_type() const { return buff_type_; } // 获取当前buff模式
};

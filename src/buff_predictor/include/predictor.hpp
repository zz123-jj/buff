#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/FFT>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>
#include <stdexcept>
#include <deque>
#include "dataProcessor.hpp"
using namespace Eigen;
//拟合开始检测器
//根据用户设定的时间阈值和时间戳来判断是否可以开始拟合
class FitStartDetect {
private:
    double required_time_;      // 用户设定的时间阈值（秒）
    double first_timestamp_;    // 第一次tracking的时间戳（秒）
    bool has_first_stamp_;      // 是否已记录第一次时间戳
public:
    // required_time: 用户设定的时间（秒），默认1.5秒
    FitStartDetect(double required_time = 1.5);
    // 更新时间戳，判断是否可以开始拟合
    // current_timestamp: 当前时间戳（秒）
    // is_tracking: 是否处于tracking状态
    bool update(double current_timestamp, bool is_tracking);
    // 重置检测器
    void reset();
};


class Big_Buff_Predictor {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
    MovAvg Avg_filter_;                     // 平滑滤波器
    std::vector<float> smoothed_angles_;    // 平滑后的数据
    std::vector<double> time_stamps_;       // 时间戳（秒，基于首帧对齐）
    double start_time_sec_ = -1.0;          // 首帧时间戳（秒）
    double last_time_sec_ = 0.0;            // 最近一帧时间戳（秒）
    int frame_counter_;                     // 当前帧计数
    FitStartDetect start_detector_;         // 拟合开始检测器
    bool canStart_;                          // 是否开始拟合
    bool is_get_para_;                      // 是否已有拟合参数
    Eigen::VectorXf sin_para_;              // 速度函数a sin ( ωt + φ ) + b
    Eigen::VectorXf cos_para_;              // 积分后的位移函数A cos(ωt + φ) + Bx + C
    Eigen::VectorXf diff_datas_;            // 原始差分数据
    Eigen::VectorXf diff_smoothed_datas_;   // 滤波后的差分数据
    public:
     Big_Buff_Predictor();
     Eigen::VectorXf sinFit(const Eigen::VectorXf& x, const Eigen::VectorXf& y);
    std::pair<bool, float> update(float data, double stamp_sec);  // 时间戳驱动的更新
     bool is_completed() const { return is_get_para_; }                                 // 是否已完成拟合
     Eigen::VectorXf get_diff_datas() const { return diff_datas_; }                     // 获取原始差分数据
     Eigen::VectorXf get_diff_smoothed_datas() const { return diff_smoothed_datas_; }   // 获取滤波后的差分数据
     Eigen::VectorXf get_sin_para() const { return sin_para_; }                         // 获取微分拟合参数 (a, ω, φ, b) dy = a * sin(ω * x + φ) + b
     Eigen::VectorXf get_cos_para() const { return cos_para_; }                         // 获取拟合参数 (A, ω, φ, B, C) y = A * cos(ω * x + φ) + B * x + C
     // 设置内部状态
     void set_parameters(const Eigen::VectorXf& params) { sin_para_ = params; is_get_para_ = true; }
     void set_smoothed_datas(const std::vector<float>& data) { smoothed_angles_ = data; }
     void set_frame_counter(int counter) { frame_counter_ = counter; }
    };


class smallPredictor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    float window_time_;              // 窗口时间（秒）
    float angular_velocity_;         // 角速度（rad/s）
    std::vector<float> angles_;      // 角度序列
    std::vector<double> timestamps_; // 时间戳序列（秒）
public:
    smallPredictor();
    void reset();
    std::pair<bool, float> update(float angle, double timestamp);  // 基于时间戳更新
};

float euclidean_distance(cv::Point2f p1, cv::Point2f p2);
float trans(float x, float y);
enum sizeMode{
    big,
    small
};
enum clockMode{
    unknown,
    anticlockwise,
    clockwise
};
/**
 * 相机内参结构体
 */
struct CameraParams {
    float fx, fy;       // 焦距（像素）
    float cx, cy;       // 主点（像素）
    float k1, k2, p1, p2;  // 畸变系数（可选）
    CameraParams() : fx(1000.0f), fy(1000.0f), cx(640.0f), cy(360.0f),
                     k1(0), k2(0), p1(0), p2(0) {}
    CameraParams(float _fx, float _fy, float _cx, float _cy)
        : fx(_fx), fy(_fy), cx(_cx), cy(_cy), k1(0), k2(0), p1(0), p2(0) {}
};
/**
 * 三维坐标点结构体（相机坐标系）
 * X轴：指向图像右侧
 * Y轴：指向图像下方
 * Z轴：指向相机前方（深度方向）
 */
struct Point3D {
    float x, y, z;
    Point3D() : x(0), y(0), z(0) {}
    Point3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};
//3D预测器类：将2D预测结果转换为3D坐标
//已知能量机关半径为1.4米

class Predictor3D {
private:
    CameraParams cam_;          // 相机内参
    float radius_world_;        // 目标实际半径（米），默认1.4m
public:
    
    Predictor3D(const CameraParams& cam, float radius_world = 1.4f);
    
    void set_camera_params(const CameraParams& cam) { cam_ = cam; }
    
    void set_radius(float radius) { radius_world_ = radius; }
    
    CameraParams get_camera_params() const { return cam_; }
    /**
     * 通过已知半径计算深度
     * @param radius_pixel 目标在图像中的像素半径
     * @return 深度值（米）
     */
    float compute_depth(float radius_pixel) const;
    /**
     * 像素坐标 + 深度 → 相机坐标系3D点
     * @param u 像素坐标u（列）
     * @param v 像素坐标v（行）
     * @param depth 深度值（米）
     * @return 相机坐标系下的3D点
     */
    Point3D pixel_to_3d(float u, float v, float depth) const;
    /**
     * 预测未来3D位置
     * @param current_angle 当前角度（弧度）
     * @param angle_increment 角度增量（弧度），从Big_Buff_Predictor获得
     * @param center_u 旋转中心像素坐标u
     * @param center_v 旋转中心像素坐标v
     * @param radius_pixel 目标旋转半径（像素）
     * @return 预测的相机坐标系3D位置
     */
    Point3D predict_3d_position(float current_angle, float angle_increment,
                                float center_u, float center_v,
                                float radius_pixel) const;
};
/**
 * 角度观测器：跟踪旋转目标的角度变化
 * 处理角度连续性问题和装甲板跳变检测（如能量机关的5个装甲板）
 */
class angleObserver{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
    clockMode mode_;
    int direction_detect_count_ = 0;       // 检测次数
    float total_angle_diff_ = 0.0f;        // 累计的角度变化量
    cv::Point2f last_point_;
    float last_angle_;
    int blade_jump_count_;
    bool is_first_angle_;
    public:
    angleObserver(clockMode mode);
    Vector2f Rotation(float theta, Vector2f vector);
    float AngleTransformer(float x, float y);
    float update(float target_x, float target_y, float radius);
};
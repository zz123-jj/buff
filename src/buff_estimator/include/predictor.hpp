#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <deque>
#include "dataProcessor.hpp"
using namespace Eigen;

//拟合开始检测器
//根据用户设定的时间阈值和时间戳来判断是否可以开始拟合
class Big_Buff_Predictor {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::vector<std::pair<double,float>> time_w_pairs_;  //时间角速度对 
    private:
    bool fit_attempted_ = false;//是否已经尝试过拟合
    bool fit_ready_ = false;//是否拟合完成并准备好预测结果
    Vector4f velocity_fit_params_ = Vector4f::Zero();  // [A, omega, phi, b]
    double fit_offset_sum_ = 2.090;
    double fit_window_sec_ = 1.5;
    int fit_min_samples_ = 10;
    double a_lower_ = 0.780;
    double a_upper_ = 1.045;
    double omega_lower_ = 1.884;
    double omega_upper_ = 2.000;
    double phi_lower_ = -M_PI;
    double phi_upper_ = M_PI;

    public:
    Big_Buff_Predictor();
    void reset();
    bool try_fit_once_at_1p5s();
    bool is_completed() const { return fit_ready_; }
    bool has_fit_attempted() const { return fit_attempted_; }
    void set_fit_config(
        double fit_offset_sum,
        double fit_window_sec,
        int fit_min_samples,
        double a_lower,
        double a_upper,
        double omega_lower,
        double omega_upper,
        double phi_lower,
        double phi_upper)
    {
        fit_offset_sum_ = fit_offset_sum;
        fit_window_sec_ = fit_window_sec;
        fit_min_samples_ = fit_min_samples;
        a_lower_ = a_lower;
        a_upper_ = a_upper;
        omega_lower_ = omega_lower;
        omega_upper_ = omega_upper;
        phi_lower_ = phi_lower;
        phi_upper_ = phi_upper;
    }
    Vector4f get_velocity_fit_params() const { return velocity_fit_params_; }
    float get_fit_buffer_duration_sec() const
    {
        return time_w_pairs_.empty() ? 0.0f : static_cast<float>(time_w_pairs_.back().first);
    }
    int get_fit_data_point_count() const { return static_cast<int>(time_w_pairs_.size()); }
    double get_fit_window_sec() const { return fit_window_sec_; }
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

 //角度观测器：跟踪旋转目标的角度变化
 //处理角度连续性问题和装甲板跳变检测（如能量机关的5个装甲板）
 class angleObserver{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
    clockMode mode_;
    cv::Point2f last_point_;
    int blade_jump_count_;
    bool is_first_angle_;
    public:
        float last_angle_;

    angleObserver(clockMode mode);
    Vector2f Rotation(float theta, Vector2f vector);
    float AngleTransformer(float x, float y);
    clockMode getClockMode() const;
    void setClockMode(clockMode mode);
    void reset();
    float update(float target_x, float target_y, float radius);
};
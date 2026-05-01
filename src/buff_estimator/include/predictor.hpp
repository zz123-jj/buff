#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <utility>
#include <vector>

class Big_Buff_Predictor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<std::pair<double, float>> time_w_pairs_;

    Big_Buff_Predictor() = default;

    void reset();
    bool fit_velocity_curve();
    void trim_before(double min_time);
    bool is_completed() const { return fit_ready_; }

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

    Eigen::Vector4f get_velocity_fit_params() const { return velocity_fit_params_; }
    float get_fit_buffer_duration_sec() const
    {
        return time_w_pairs_.empty() ? 0.0f : static_cast<float>(time_w_pairs_.back().first);
    }
    int get_fit_data_point_count() const { return static_cast<int>(time_w_pairs_.size()); }

private:
    bool fit_ready_ = false;
    Eigen::Vector4f velocity_fit_params_ = Eigen::Vector4f::Zero();
    double fit_offset_sum_ = 2.090;
    double fit_window_sec_ = 1.5;
    int fit_min_samples_ = 10;
    double a_lower_ = 0.780;
    double a_upper_ = 1.045;
    double omega_lower_ = 1.884;
    double omega_upper_ = 2.000;
    double phi_lower_ = -M_PI;
    double phi_upper_ = M_PI;
};

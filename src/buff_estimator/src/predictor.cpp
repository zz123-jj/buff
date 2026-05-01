#include "predictor.hpp"

#include <algorithm>
#include <cmath>

#include <ceres/ceres.h>

namespace
{
struct VelocitySineResidual
{
    VelocitySineResidual(double t, double y, double fit_offset_sum)
        : t_(t), y_(y), fit_offset_sum_(fit_offset_sum)
    {
    }

    template <typename T>
    bool operator()(const T* const A, const T* const omega, const T* const phi, T* residual) const
    {
        const T prediction =
            A[0] * ceres::sin(omega[0] * T(t_) + phi[0]) + (T(fit_offset_sum_) - A[0]);
        residual[0] = prediction - T(y_);
        return true;
    }

private:
    double t_;
    double y_;
    double fit_offset_sum_;
};
}  // namespace

void Big_Buff_Predictor::reset()
{
    time_w_pairs_.clear();
    fit_ready_ = false;
    velocity_fit_params_.setZero();
}

bool Big_Buff_Predictor::fit_velocity_curve()
{
    if (time_w_pairs_.size() < static_cast<std::size_t>(fit_min_samples_)) {
        return false;
    }

    const double duration = time_w_pairs_.back().first - time_w_pairs_.front().first;
    if (duration < fit_window_sec_ - 0.3) {
        return false;
    }

    double A = velocity_fit_params_[0] > 0.0f
        ? static_cast<double>(velocity_fit_params_[0])
        : 0.5 * (a_lower_ + a_upper_);
    double omega = velocity_fit_params_[1] > 0.0f
        ? static_cast<double>(velocity_fit_params_[1])
        : 0.5 * (omega_lower_ + omega_upper_);
    double phi = static_cast<double>(velocity_fit_params_[2]);

    ceres::Problem problem;
    for (const auto& sample : time_w_pairs_) {
        ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<VelocitySineResidual, 1, 1, 1, 1>(
                new VelocitySineResidual(
                    sample.first,
                    static_cast<double>(sample.second),
                    fit_offset_sum_));
        problem.AddResidualBlock(cost, nullptr, &A, &omega, &phi);
    }

    problem.SetParameterLowerBound(&A, 0, a_lower_);
    problem.SetParameterUpperBound(&A, 0, a_upper_);
    problem.SetParameterLowerBound(&omega, 0, omega_lower_);
    problem.SetParameterUpperBound(&omega, 0, omega_upper_);
    problem.SetParameterLowerBound(&phi, 0, phi_lower_);
    problem.SetParameterUpperBound(&phi, 0, phi_upper_);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 80;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    const bool success = summary.IsSolutionUsable() &&
                         std::isfinite(A) &&
                         std::isfinite(omega) &&
                         std::isfinite(phi);
    if (!success) {
        return false;
    }

    velocity_fit_params_[0] = static_cast<float>(A);
    velocity_fit_params_[1] = static_cast<float>(omega);
    velocity_fit_params_[2] = static_cast<float>(phi);
    velocity_fit_params_[3] = static_cast<float>(fit_offset_sum_ - A);
    fit_ready_ = true;
    return true;
}

void Big_Buff_Predictor::trim_before(double min_time)
{
    time_w_pairs_.erase(
        std::remove_if(
            time_w_pairs_.begin(),
            time_w_pairs_.end(),
            [min_time](const auto& sample) { return sample.first < min_time; }),
        time_w_pairs_.end());
}

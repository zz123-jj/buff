#include "rm_buff_common/ransac_sine_fitter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace rm_buff_common
{

RansacSineFitter::RansacSineFitter(
  int max_iterations, double threshold, double min_omega, double max_omega)
{
  configure(max_iterations, threshold, min_omega, max_omega);
}

void RansacSineFitter::configure(
  int max_iterations, double threshold, double min_omega, double max_omega)
{
  max_iterations_ = max_iterations;
  threshold_ = threshold;
  min_omega_ = min_omega;
  max_omega_ = max_omega;
}

void RansacSineFitter::add_data(double t, double v)
{
  if (!fit_data_.empty() && t - fit_data_.back().first > 5.0) reset();
  fit_data_.emplace_back(t, v);
}

void RansacSineFitter::fit()
{
  if (fit_data_.size() < 3) return;

  std::uniform_real_distribution<double> omega_dist(min_omega_, max_omega_);
  std::vector<size_t> indices(fit_data_.size());
  std::iota(indices.begin(), indices.end(), 0);

  for (int iter = 0; iter < max_iterations_; ++iter) {
    std::shuffle(indices.begin(), indices.end(), gen_);

    std::vector<std::pair<double, double>> sample;
    for (int i = 0; i < 3; ++i) sample.push_back(fit_data_[indices[i]]);

    Eigen::Vector3d params;
    const auto omega = omega_dist(gen_);
    if (!fit_partial_model(sample, omega, params)) continue;

    const auto A1 = params.x();
    const auto A2 = params.y();
    const auto C = params.z();
    const auto A = std::hypot(A1, A2);
    const auto phi = std::atan2(A2, A1);
    const auto inliers = evaluate_inliers(A, omega, phi, C);

    if (inliers > best_result_.inliers) {
      best_result_ = {A, omega, phi, C, inliers};
    }
  }

  if (fit_data_.size() > 150) fit_data_.pop_front();
}

void RansacSineFitter::reset()
{
  fit_data_.clear();
  best_result_ = {};
}

double RansacSineFitter::sine_function(double t) const
{
  return sine_function(t, best_result_.A, best_result_.omega, best_result_.phi, best_result_.C);
}

double RansacSineFitter::sine_function(double t, double A, double omega, double phi, double C)
{
  return A * std::sin(omega * t + phi) + C;
}

bool RansacSineFitter::fit_partial_model(
  const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params)
{
  Eigen::MatrixXd X(sample.size(), 3);
  Eigen::VectorXd Y(sample.size());

  for (size_t i = 0; i < sample.size(); ++i) {
    X(i, 0) = std::sin(omega * sample[i].first);
    X(i, 1) = std::cos(omega * sample[i].first);
    X(i, 2) = 1.0;
    Y(i) = sample[i].second;
  }

  params = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
  return params.allFinite();
}

int RansacSineFitter::evaluate_inliers(double A, double omega, double phi, double C) const
{
  int count = 0;
  for (const auto & [t, y] : fit_data_) {
    if (std::abs(y - sine_function(t, A, omega, phi, C)) < threshold_) ++count;
  }
  return count;
}

}  // namespace rm_buff_common

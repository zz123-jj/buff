#ifndef RM_BUFF_COMMON__RANSAC_SINE_FITTER_HPP_
#define RM_BUFF_COMMON__RANSAC_SINE_FITTER_HPP_

#include <Eigen/Dense>

#include <deque>
#include <random>
#include <utility>
#include <vector>

namespace rm_buff_common
{

class RansacSineFitter
{
public:
  struct Result
  {
    double A = 0.0;
    double omega = 0.0;
    double phi = 0.0;
    double C = 0.0;
    int inliers = 0;
  };

  RansacSineFitter() = default;
  RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega);

  void configure(int max_iterations, double threshold, double min_omega, double max_omega);
  void add_data(double t, double v);
  void fit();
  void reset();

  Result best_result() const { return best_result_; }
  double sine_function(double t) const;
  static double sine_function(double t, double A, double omega, double phi, double C);

private:
  bool fit_partial_model(
    const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params);
  int evaluate_inliers(double A, double omega, double phi, double C) const;

  int max_iterations_ = 100;
  double threshold_ = 0.5;
  double min_omega_ = 1.884;
  double max_omega_ = 2.000;
  Result best_result_;
  std::mt19937 gen_{std::random_device{}()};
  std::deque<std::pair<double, double>> fit_data_;
};

}  // namespace rm_buff_common

#endif  // RM_BUFF_COMMON__RANSAC_SINE_FITTER_HPP_

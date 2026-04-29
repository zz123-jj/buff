#include "rm_buff_common/extended_kalman_filter.hpp"

#include <utility>

namespace rm_buff_common
{

ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I_(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add_(std::move(x_add))
{
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & state) { return F * state; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & state) { return H * state; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const Eigen::MatrixXd S = H * P * H.transpose() + R;
  const Eigen::MatrixXd K = P * H.transpose() * S.inverse();
  P = (I_ - K * H) * P * (I_ - K * H).transpose() + K * R * K.transpose();
  x = x_add_(x, K * z_subtract(z, h(x)));
  return x;
}

}  // namespace rm_buff_common

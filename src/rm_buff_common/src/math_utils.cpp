#include "rm_buff_common/math_utils.hpp"

#include <cmath>

namespace rm_buff_common
{

double limit_rad(double angle)
{
  while (angle > kPi) angle -= kTwoPi;
  while (angle <= -kPi) angle += kTwoPi;
  return angle;
}

Eigen::Matrix3d rotation_matrix(double yaw, double pitch, double roll)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Vector3d xyz_to_ypd(const Eigen::Vector3d & xyz)
{
  return {
    std::atan2(xyz.y(), xyz.x()),
    std::atan2(xyz.z(), std::hypot(xyz.x(), xyz.y())),
    xyz.norm()};
}

Eigen::Vector3d ypd_to_xyz(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd.x();
  auto pitch = ypd.y();
  auto distance = ypd.z();
  return {
    distance * std::cos(pitch) * std::cos(yaw),
    distance * std::cos(pitch) * std::sin(yaw),
    distance * std::sin(pitch)};
}

Eigen::Matrix3d xyz_to_ypd_jacobian(const Eigen::Vector3d & xyz)
{
  const auto x = xyz.x();
  const auto y = xyz.y();
  const auto z = xyz.z();
  const auto xy2 = std::max(x * x + y * y, 1e-9);
  const auto xy = std::sqrt(xy2);
  const auto r2 = std::max(xy2 + z * z, 1e-9);
  const auto r = std::sqrt(r2);

  Eigen::Matrix3d j;
  j << -y / xy2, x / xy2, 0.0,
    -x * z / (xy * r2), -y * z / (xy * r2), xy / r2,
    x / r, y / r, z / r;
  return j;
}

Eigen::Matrix3d ypd_to_xyz_jacobian(const Eigen::Vector3d & ypd)
{
  const auto yaw = ypd.x();
  const auto pitch = ypd.y();
  const auto distance = ypd.z();
  const auto cos_yaw = std::cos(yaw);
  const auto sin_yaw = std::sin(yaw);
  const auto cos_pitch = std::cos(pitch);
  const auto sin_pitch = std::sin(pitch);

  Eigen::Matrix3d j;
  j << -distance * cos_pitch * sin_yaw, -distance * sin_pitch * cos_yaw,
    cos_pitch * cos_yaw,
    distance * cos_pitch * cos_yaw, -distance * sin_pitch * sin_yaw,
    cos_pitch * sin_yaw,
    0.0, distance * cos_pitch, sin_pitch;
  return j;
}

Eigen::Vector3d eulers_zyx(const Eigen::Matrix3d & rotation)
{
  auto yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  auto pitch = std::atan2(-rotation(2, 0), std::hypot(rotation(2, 1), rotation(2, 2)));
  auto roll = std::atan2(rotation(2, 1), rotation(2, 2));
  return {yaw, pitch, roll};
}

Eigen::Isometry3d transform_to_eigen(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q = transform.transform.rotation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  Eigen::Isometry3d eigen = Eigen::Isometry3d::Identity();
  eigen.linear() = rotation.normalized().toRotationMatrix();
  eigen.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  return eigen;
}

double seconds(const rclcpp::Time & time) { return static_cast<double>(time.nanoseconds()) * 1e-9; }

}  // namespace rm_buff_common

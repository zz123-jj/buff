#ifndef RM_BUFF_COMMON__MATH_UTILS_HPP_
#define RM_BUFF_COMMON__MATH_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace rm_buff_common
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDeg2Rad = kPi / 180.0;

double limit_rad(double angle);
Eigen::Matrix3d rotation_matrix(double yaw, double pitch, double roll);
Eigen::Vector3d xyz_to_ypd(const Eigen::Vector3d & xyz);
Eigen::Vector3d ypd_to_xyz(const Eigen::Vector3d & ypd);
Eigen::Matrix3d xyz_to_ypd_jacobian(const Eigen::Vector3d & xyz);
Eigen::Matrix3d ypd_to_xyz_jacobian(const Eigen::Vector3d & ypd);
Eigen::Vector3d eulers_zyx(const Eigen::Matrix3d & rotation);
Eigen::Isometry3d transform_to_eigen(const geometry_msgs::msg::TransformStamped & transform);
double seconds(const rclcpp::Time & time);

}  // namespace rm_buff_common

#endif  // RM_BUFF_COMMON__MATH_UTILS_HPP_

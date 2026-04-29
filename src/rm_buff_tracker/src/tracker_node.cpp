#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rm_buff_common/extended_kalman_filter.hpp"
#include "rm_buff_common/math_utils.hpp"
#include "rm_buff_common/ransac_sine_fitter.hpp"
#include "rm_vision_interfaces/msg/buff_pose.hpp"
#include "rm_vision_interfaces/msg/buff_target.hpp"

namespace rm_buff_tracker
{
using namespace rm_buff_common;

namespace
{
Eigen::VectorXd diagonal_vector(
  const std::vector<double> & values, size_t expected, const std::vector<double> & defaults)
{
  const auto & source = values.size() == expected ? values : defaults;
  Eigen::VectorXd result(expected);
  for (size_t i = 0; i < expected; ++i) result[static_cast<Eigen::Index>(i)] = source[i];
  return result;
}

Eigen::MatrixXd diagonal_matrix(
  const std::vector<double> & values, size_t expected, const std::vector<double> & defaults)
{
  return diagonal_vector(values, expected, defaults).asDiagonal();
}

Eigen::Vector3d vector3(const geometry_msgs::msg::Vector3 & msg)
{
  return {msg.x, msg.y, msg.z};
}
}  // namespace

class BuffTrackerNode : public rclcpp::Node
{
public:
  explicit BuffTrackerNode(const rclcpp::NodeOptions & options) : Node("buff_tracker", options)
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/buff/pose");
    target_topic_ = declare_parameter<std::string>("target_topic", "/buff/target");
    buff_mode_ = declare_parameter<std::string>("buff_mode", "small");
    lost_threshold_ = declare_parameter<int>("lost_threshold", 6);

    small_initial_p_ = diagonal_matrix(
      declare_parameter<std::vector<double>>("small_initial_p", std::vector<double>{}),
      7, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1e-2});
    big_initial_p_ = diagonal_matrix(
      declare_parameter<std::vector<double>>("big_initial_p", std::vector<double>{}),
      10, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 100.0, 10.0, 10.0, 400.0});

    center_r_ = diagonal_matrix(
      declare_parameter<std::vector<double>>("center_r", std::vector<double>{}), 4,
      {0.01, 0.01, 0.5, 0.1});
    blade_r_ =
      diagonal_matrix(
        declare_parameter<std::vector<double>>("blade_r", std::vector<double>{}), 3,
        {0.01, 0.01, 0.5});

    small_speed_ = declare_parameter<double>("small_speed", kPi / 3.0);
    small_q_yaw_acc_ = declare_parameter<double>("small_q_yaw_acc", 0.001);

    big_speed_bias_ = declare_parameter<double>("big_speed_bias", 2.09);
    big_initial_a_ = declare_parameter<double>("big_initial_a", 0.9125);
    big_initial_w_ = declare_parameter<double>("big_initial_w", 1.942);
    big_initial_phi_ = declare_parameter<double>("big_initial_phi", 0.0);
    big_q_yaw_acc_ = declare_parameter<double>("big_q_yaw_acc", 0.9);
    big_q_roll_ = declare_parameter<double>("big_q_roll", 0.09);
    big_q_speed_ = declare_parameter<double>("big_q_speed", 0.5);
    big_q_phi_ = declare_parameter<double>("big_q_phi", 1.0);
    fitter_min_speed_ = declare_parameter<double>("fitter_min_speed", 0.0);
    fitter_max_speed_ = declare_parameter<double>("fitter_max_speed", 2.1);
    fitter_min_inliers_ = declare_parameter<int>("fitter_min_inliers", 3);
    speed_fitter_.configure(
      declare_parameter<int>("fitter_max_iterations", 100),
      declare_parameter<double>("fitter_threshold", 0.5),
      declare_parameter<double>("fitter_min_omega", 1.884),
      declare_parameter<double>("fitter_max_omega", 2.000));

    if (buff_mode_ != "small" && buff_mode_ != "big") {
      throw std::runtime_error("buff_mode must be 'small' or 'big'");
    }

    publisher_ = create_publisher<rm_vision_interfaces::msg::BuffTarget>(target_topic_, 10);
    subscription_ = create_subscription<rm_vision_interfaces::msg::BuffPose>(
      pose_topic_, 10, [this](const rm_vision_interfaces::msg::BuffPose::ConstSharedPtr msg) {
        pose_callback(msg);
      });

    RCLCPP_INFO(
      get_logger(), "buff tracker started: pose=%s target=%s mode=%s", pose_topic_.c_str(),
      target_topic_.c_str(), buff_mode_.c_str());
  }

private:
  void pose_callback(const rm_vision_interfaces::msg::BuffPose::ConstSharedPtr msg)
  {
    if (!msg->solved) {
      ++lost_count_;
      if (lost_count_ > lost_threshold_) reset();
      publish_safe(msg->header);
      return;
    }

    lost_count_ = 0;
    const auto stamp_sec = seconds(msg->header.stamp);
    if (!initialized_) {
      time_origin_sec_ = stamp_sec;
      init(*msg, 0.0);
    }

    const auto model_time = stamp_sec - time_origin_sec_;
    update(*msg, model_time);
    publish_target(msg->header, model_time);
  }

  void reset()
  {
    initialized_ = false;
    lost_count_ = 0;
    direction_vote_ = 0;
    speed_fitter_.reset();
  }

  void init(const rm_vision_interfaces::msg::BuffPose & msg, double model_time)
  {
    last_time_ = model_time;
    const auto center = vector3(msg.center_ypd);
    const auto ypr = vector3(msg.buff_ypr);

    const auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd c = a + b;
      c[0] = limit_rad(c[0]);
      c[2] = limit_rad(c[2]);
      c[4] = limit_rad(c[4]);
      c[5] = limit_rad(c[5]);
      if (c.rows() > 9) c[9] = limit_rad(c[9]);
      return c;
    };

    if (buff_mode_ == "small") {
      Eigen::VectorXd x0(7);
      x0 << center.x(), 0.0, center.y(), center.z(), ypr.x(), ypr.z(), small_speed_;
      ekf_ = ExtendedKalmanFilter(x0, small_initial_p_, x_add);
    } else {
      Eigen::VectorXd x0(10);
      x0 << center.x(), 0.0, center.y(), center.z(), ypr.x(), ypr.z(),
        big_speed_bias_ - big_initial_a_, big_initial_a_, big_initial_w_, big_initial_phi_;
      ekf_ = ExtendedKalmanFilter(x0, big_initial_p_, x_add);
    }

    initialized_ = true;
  }

  void update(const rm_vision_interfaces::msg::BuffPose & msg, double model_time)
  {
    const auto center = vector3(msg.center_ypd);
    const auto ypr = vector3(msg.buff_ypr);
    const auto blade = vector3(msg.blade_ypd);

    unwrap_roll(ypr.z());
    vote_direction(ekf_.x[5], ypr.z());
    if (buff_mode_ == "small" && direction() * ekf_.x[6] < 0.0) ekf_.x[6] *= -1.0;

    const auto angle_before_predict = ekf_.x[5];
    predict(model_time - last_time_);

    Eigen::MatrixXd H1(4, ekf_.x.rows());
    H1.setZero();
    H1(0, 0) = 1.0;
    H1(1, 2) = 1.0;
    H1(2, 3) = 1.0;
    H1(3, 5) = 1.0;

    auto z_subtract1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd c = a - b;
      c[0] = limit_rad(c[0]);
      c[1] = limit_rad(c[1]);
      c[3] = limit_rad(c[3]);
      return c;
    };
    Eigen::VectorXd z1(4);
    z1 << center.x(), center.y(), center.z(), ypr.z();
    ekf_.update(z1, H1, center_r_, z_subtract1);

    const auto H2 = h_jacobian();
    const auto h2 = [this](const Eigen::VectorXd & x) { return xyz_to_ypd(point_buff_to_world(x, {0.0, 0.0, 0.7})); };
    auto z_subtract2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd c = a - b;
      c[0] = limit_rad(c[0]);
      c[1] = limit_rad(c[1]);
      return c;
    };
    Eigen::VectorXd z2(3);
    z2 << blade.x(), blade.y(), blade.z();
    ekf_.update(z2, H2, blade_r_, h2, z_subtract2);

    if (buff_mode_ == "big") {
      const auto speed = ekf_.x[6];
      if (speed >= fitter_min_speed_ && speed < fitter_max_speed_) speed_fitter_.add_data(model_time, speed);
      speed_fitter_.fit();
      debug_speed_ = direction() * (ekf_.x[5] - angle_before_predict) / std::max(model_time - last_time_, 1e-6);
    }

    last_time_ = model_time;
  }

  void predict(double dt)
  {
    if (dt < 0.0) dt = 0.0;
    if (buff_mode_ == "small") {
      predict_small(dt);
    } else {
      predict_big(dt);
    }
  }

  void predict_small(double dt)
  {
    Eigen::MatrixXd F(7, 7);
    F << 1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7, 7);
    const auto a = dt * dt * dt * dt / 4.0;
    const auto b = dt * dt * dt / 2.0;
    const auto c = dt * dt;
    Q(0, 0) = a * small_q_yaw_acc_;
    Q(0, 1) = b * small_q_yaw_acc_;
    Q(1, 0) = b * small_q_yaw_acc_;
    Q(1, 1) = c * small_q_yaw_acc_;

    ekf_.predict(F, Q, [this, dt](const Eigen::VectorXd & x) {
      Eigen::VectorXd prior = x;
      prior[0] = limit_rad(x[0] + dt * x[1]);
      prior[2] = limit_rad(x[2]);
      prior[4] = limit_rad(x[4]);
      prior[5] = limit_rad(x[5] + dt * x[6]);
      return prior;
    });
  }

  void predict_big(double dt)
  {
    const auto a = ekf_.x[7];
    const auto w = ekf_.x[8];
    const auto phi = ekf_.x[9];
    const auto t = last_time_ + dt;

    Eigen::MatrixXd F(10, 10);
    F << 1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, direction() * dt, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, std::sin(w * t + phi) - 1.0,
      t * a * std::cos(w * t + phi), a * std::cos(w * t + phi),
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(10, 10);
    const auto q0 = dt * dt * dt * dt / 4.0;
    const auto q1 = dt * dt * dt / 2.0;
    const auto q2 = dt * dt;
    Q(0, 0) = q0 * big_q_yaw_acc_;
    Q(0, 1) = q1 * big_q_yaw_acc_;
    Q(1, 0) = q1 * big_q_yaw_acc_;
    Q(1, 1) = q2 * big_q_yaw_acc_;
    Q(5, 5) = big_q_roll_;
    Q(6, 6) = big_q_speed_;
    Q(9, 9) = big_q_phi_;

    ekf_.predict(F, Q, [this, dt, a, w, phi, t](const Eigen::VectorXd & x) {
      Eigen::VectorXd prior = x;
      prior[0] = limit_rad(x[0] + dt * x[1]);
      prior[2] = limit_rad(x[2]);
      prior[4] = limit_rad(x[4]);
      prior[5] = limit_rad(
        x[5] + direction() *
                 (-a / w * std::cos(w * t + phi) + a / w * std::cos(w * last_time_ + phi) +
                   (big_speed_bias_ - a) * dt));
      prior[6] = a * std::sin(w * t + phi) + big_speed_bias_ - a;
      return prior;
    });
  }

  void unwrap_roll(double roll)
  {
    if (std::abs(roll - ekf_.x[5]) <= kPi / 12.0) return;
    for (int i = -5; i <= 5; ++i) {
      const auto candidate = ekf_.x[5] + i * kTwoPi / 5.0;
      if (std::abs(candidate - roll) < kPi / 5.0) {
        ekf_.x[5] += i * kTwoPi / 5.0;
        break;
      }
    }
  }

  void vote_direction(double last_roll, double roll)
  {
    if (std::abs(direction_vote_) > 50) return;
    direction_vote_ += last_roll > roll ? -1 : 1;
  }

  double direction() const { return direction_vote_ >= 0 ? 1.0 : -1.0; }

  Eigen::Vector3d point_buff_to_world(
    const Eigen::VectorXd & state, const Eigen::Vector3d & point_in_buff) const
  {
    const auto R_buff2world = rotation_matrix(state[4], 0.0, state[5]);
    return R_buff2world * point_in_buff + ypd_to_xyz({state[0], state[2], state[3]});
  }

  Eigen::MatrixXd h_jacobian() const
  {
    const auto n = ekf_.x.rows();
    Eigen::MatrixXd H0 = Eigen::MatrixXd::Zero(5, n);
    H0(0, 0) = 1.0;
    H0(1, 2) = 1.0;
    H0(2, 3) = 1.0;
    H0(3, 4) = 1.0;
    H0(4, 5) = 1.0;

    const Eigen::Vector3d center_ypd{ekf_.x[0], ekf_.x[2], ekf_.x[3]};
    const auto H_ypd2xyz = ypd_to_xyz_jacobian(center_ypd);
    Eigen::MatrixXd H1 = Eigen::MatrixXd::Zero(5, 5);
    H1.block<3, 3>(0, 0) = H_ypd2xyz;
    H1(3, 3) = 1.0;
    H1(4, 4) = 1.0;

    const auto yaw = ekf_.x[4];
    const auto roll = ekf_.x[5];
    const auto cos_yaw = std::cos(yaw);
    const auto sin_yaw = std::sin(yaw);
    const auto cos_roll = std::cos(roll);
    const auto sin_roll = std::sin(roll);
    Eigen::Matrix<double, 3, 5> H2;
    H2 << 1.0, 0.0, 0.0, 0.7 * cos_yaw * sin_roll, 0.7 * sin_yaw * cos_roll,
      0.0, 1.0, 0.0, 0.7 * sin_yaw * sin_roll, -0.7 * cos_yaw * cos_roll,
      0.0, 0.0, 1.0, 0.0, -0.7 * sin_roll;

    const auto blade_world = point_buff_to_world(ekf_.x, {0.0, 0.0, 0.7});
    return xyz_to_ypd_jacobian(blade_world) * H2 * H1 * H0;
  }

  void publish_safe(const std_msgs::msg::Header & header)
  {
    rm_vision_interfaces::msg::BuffTarget target;
    target.header = header;
    target.tracked = false;
    target.mode = buff_mode_;
    publisher_->publish(target);
  }

  void publish_target(const std_msgs::msg::Header & header, double model_time)
  {
    rm_vision_interfaces::msg::BuffTarget target;
    target.header = header;
    target.tracked = initialized_;
    target.mode = buff_mode_;
    target.model_time = model_time;
    target.direction = static_cast<int8_t>(direction() >= 0.0 ? 1 : -1);
    target.state_dimension = static_cast<uint8_t>(ekf_.x.rows());

    target.center_ypd.x = ekf_.x[0];
    target.center_ypd.y = ekf_.x[2];
    target.center_ypd.z = ekf_.x[3];
    target.buff_ypr.x = ekf_.x[4];
    target.buff_ypr.y = 0.0;
    target.buff_ypr.z = ekf_.x[5];
    target.angular_speed = ekf_.x[6];

    if (buff_mode_ == "big") {
      auto fit = speed_fitter_.best_result();
      if (fit.inliers >= fitter_min_inliers_) {
        target.speed_a = fit.A;
        target.speed_w = fit.omega;
        target.speed_phi = fit.phi;
        target.speed_c = fit.C;
      } else {
        target.speed_a = ekf_.x[7];
        target.speed_w = ekf_.x[8];
        target.speed_phi = ekf_.x[9];
        target.speed_c = big_speed_bias_ - ekf_.x[7];
      }
    } else {
      target.speed_a = 0.0;
      target.speed_w = 0.0;
      target.speed_phi = 0.0;
      target.speed_c = std::abs(ekf_.x[6]);
    }

    const auto aim_point = point_buff_to_world(ekf_.x, {0.0, 0.0, 0.7});
    target.aim_point_world.x = aim_point.x();
    target.aim_point_world.y = aim_point.y();
    target.aim_point_world.z = aim_point.z();

    target.state.fill(0.0);
    for (Eigen::Index i = 0; i < ekf_.x.rows() && i < 10; ++i) {
      target.state[static_cast<size_t>(i)] = ekf_.x[i];
    }
    publisher_->publish(target);
  }

  std::string pose_topic_;
  std::string target_topic_;
  std::string buff_mode_;
  int lost_threshold_;
  int lost_count_ = 0;
  int direction_vote_ = 0;
  bool initialized_ = false;
  double time_origin_sec_ = 0.0;
  double last_time_ = 0.0;
  double debug_speed_ = 0.0;

  Eigen::MatrixXd small_initial_p_;
  Eigen::MatrixXd big_initial_p_;
  Eigen::MatrixXd center_r_;
  Eigen::MatrixXd blade_r_;
  double small_speed_;
  double small_q_yaw_acc_;
  double big_speed_bias_;
  double big_initial_a_;
  double big_initial_w_;
  double big_initial_phi_;
  double big_q_yaw_acc_;
  double big_q_roll_;
  double big_q_speed_;
  double big_q_phi_;
  double fitter_min_speed_;
  double fitter_max_speed_;
  int fitter_min_inliers_;

  ExtendedKalmanFilter ekf_;
  RansacSineFitter speed_fitter_;
  rclcpp::Subscription<rm_vision_interfaces::msg::BuffPose>::SharedPtr subscription_;
  rclcpp::Publisher<rm_vision_interfaces::msg::BuffTarget>::SharedPtr publisher_;
};

}  // namespace rm_buff_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff_tracker::BuffTrackerNode)

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "gary_msgs/msg/auto_aim.hpp"
#include "rm_buff_common/math_utils.hpp"
#include "rm_buff_common/trajectory.hpp"
#include "rm_vision_interfaces/msg/buff_target.hpp"

namespace rm_buff_aimer
{
using namespace rm_buff_common;

class BuffAimerNode : public rclcpp::Node
{
public:
  explicit BuffAimerNode(const rclcpp::NodeOptions & options) : Node("buff_aimer", options)
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/buff/target");
    command_topic_ = declare_parameter<std::string>("command_topic", "/autoaim/target");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal");
    bullet_speed_ = declare_parameter<double>("bullet_speed", 24.0);
    predict_time_ = declare_parameter<double>("predict_time", 0.10);
    fire_gap_time_ = declare_parameter<double>("fire_gap_time", 0.52);
    yaw_offset_ = declare_parameter<double>("yaw_offset_deg", 0.0) * kDeg2Rad;
    pitch_offset_ = declare_parameter<double>("pitch_offset_deg", 0.0) * kDeg2Rad;
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.02);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<gary_msgs::msg::AutoAIM>(command_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<rm_vision_interfaces::msg::BuffTarget>( 
      target_topic_, rclcpp::SensorDataQoS(), [this](const rm_vision_interfaces::msg::BuffTarget::ConstSharedPtr msg) {
        target_callback(msg);
      });

    RCLCPP_INFO(
      get_logger(), "buff aimer started: target=%s command=%s", target_topic_.c_str(),
      command_topic_.c_str());
  }

private:
  struct AimResult
  {
    bool control = false;
    bool fire = false;
    double yaw = 0.0;
    double pitch = 0.0;
    double target_distance = 0.0;
  };

  void target_callback(const rm_vision_interfaces::msg::BuffTarget::ConstSharedPtr msg)
  {
    if (!msg->tracked) {
      //publish_safe(msg->header);
      return;
    }

    Eigen::Isometry3d T_world_gimbal;
    try {
      auto transform = tf_buffer_->lookupTransform(
        world_frame_, gimbal_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(tf_timeout_sec_));
      T_world_gimbal = transform_to_eigen(transform);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed: %s", e.what());
      //publish_safe(msg->header);
      return;
    }

    auto result = aim(*msg, seconds(now()), T_world_gimbal.translation());
    if (!result.control) {
      //publish_safe(msg->header);
      return;
    }

    publish_command(
      msg->header, true, result.fire, result.yaw, result.pitch, result.target_distance, msg->mode);
  }

  AimResult aim(
    const rm_vision_interfaces::msg::BuffTarget & target, double now_sec,
    const Eigen::Vector3d & shooter_position_world)
  {
    AimResult result;
    if (bullet_speed_ < 10.0) bullet_speed_ = 24.0;
    if (!std::isfinite(last_fire_sec_)) last_fire_sec_ = now_sec;

    auto future = std::max(0.0, now_sec - seconds(target.header.stamp)) + predict_time_;
    auto aim_point_world = predict_aim_point(target, future);
    Eigen::Vector3d aim_vector_world = aim_point_world - shooter_position_world;
    auto trajectory0 = trajectory_for(aim_vector_world);
    if (!trajectory0.solvable) return result;

    aim_point_world = predict_aim_point(target, future + trajectory0.fly_time);
    aim_vector_world = aim_point_world - shooter_position_world;
    auto trajectory1 = trajectory_for(aim_vector_world);
    if (!trajectory1.solvable) return result;
    if (std::abs(trajectory1.fly_time - trajectory0.fly_time) > 0.01) return result;

    result.yaw = limit_rad(std::atan2(aim_vector_world.y(), aim_vector_world.x()) + yaw_offset_);
    result.pitch = -(trajectory1.pitch + pitch_offset_);
    result.target_distance = aim_vector_world.norm();
    result.control = stable(result.yaw, result.pitch);
    result.fire = result.control && now_sec - last_fire_sec_ > fire_gap_time_;
    if (!result.control || result.fire) last_fire_sec_ = now_sec;
    return result;
  }

  Eigen::Vector3d predict_aim_point(
    const rm_vision_interfaces::msg::BuffTarget & target, double future) const
  {
    Eigen::Vector3d center_ypd(target.state[0], target.state[2], target.state[3]);
    auto center_xyz = ypd_to_xyz(center_ypd);
    auto yaw = target.state[4];
    auto roll = predict_roll(target, future);
    return rotation_matrix(yaw, 0.0, roll) * Eigen::Vector3d(0.0, 0.0, 0.7) + center_xyz;
  }

  double predict_roll(const rm_vision_interfaces::msg::BuffTarget & target, double future) const
  {
    const auto roll = target.state[5];
    if (target.mode == "small" || target.state_dimension <= 7) {
      return limit_rad(roll + target.angular_speed * future);
    }

    const auto direction = target.direction >= 0 ? 1.0 : -1.0;
    const auto A = target.speed_a;
    const auto w = std::abs(target.speed_w) > 1e-6 ? target.speed_w : 1.942;
    const auto phi = target.speed_phi;
    const auto C = target.speed_c;
    const auto t0 = target.model_time;
    const auto t1 = t0 + future;
    const auto delta =
      -A / w * std::cos(w * t1 + phi) + A / w * std::cos(w * t0 + phi) + C * future;
    return limit_rad(roll + direction * delta);
  }

  Trajectory trajectory_for(const Eigen::Vector3d & aim_vector) const
  {
    auto d = std::hypot(aim_vector.x(), aim_vector.y());
    return solve_trajectory(bullet_speed_, d, aim_vector.z());
  }

  bool stable(double yaw, double pitch)
  {
    if (mistake_count_ > 3) {
      mistake_count_ = 0;
      last_yaw_ = yaw;
      last_pitch_ = pitch;
      return true;
    }
    if (std::abs(limit_rad(last_yaw_ - yaw)) > 5.0 * kDeg2Rad ||
      std::abs(limit_rad(last_pitch_ - pitch)) > 5.0 * kDeg2Rad) {
      ++mistake_count_;
      last_yaw_ = yaw;
      last_pitch_ = pitch;
      return false;
    }
    mistake_count_ = 0;
    last_yaw_ = yaw;
    last_pitch_ = pitch;
    return true;
  }

  void publish_safe(const std_msgs::msg::Header & header)
  {
    publish_command(header, false, false, 0.0, 0.0, 0.0, "");
  }

  void publish_command(
    const std_msgs::msg::Header & header, bool control, bool fire, double yaw, double pitch,
    double target_distance, const std::string & mode)
  {
    gary_msgs::msg::AutoAIM command;
    command.header = header;
    command.header.frame_id = world_frame_;
    command.yaw = static_cast<float>(yaw);
    command.pitch = static_cast<float>(pitch);
    command.target_id = control ? gary_msgs::msg::AutoAIM::TARGET_ID6_OUTPOST :
                                  gary_msgs::msg::AutoAIM::TARGET_ID0_NONE;
    command.target_distance = static_cast<float>(target_distance);
    command.vision_mode = mode == "big" ? gary_msgs::msg::AutoAIM::VISION_MODE_BIG :
                                          gary_msgs::msg::AutoAIM::VISION_MODE_SMALL;
    command.shoot_command = fire ? gary_msgs::msg::AutoAIM::ALLOW_SHOOT :
                                   gary_msgs::msg::AutoAIM::CEASE_FIRE;
    publisher_->publish(command);
  }

  std::string target_topic_;
  std::string command_topic_;
  std::string world_frame_;
  std::string gimbal_frame_;
  double bullet_speed_;
  double predict_time_;
  double fire_gap_time_;
  double yaw_offset_;
  double pitch_offset_;
  double tf_timeout_sec_;

  int mistake_count_ = 0;
  double last_yaw_ = 0.0;
  double last_pitch_ = 0.0;
  double last_fire_sec_ = std::numeric_limits<double>::quiet_NaN();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<rm_vision_interfaces::msg::BuffTarget>::SharedPtr subscription_;
  rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr publisher_;
};

}  // namespace rm_buff_aimer

RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff_aimer::BuffAimerNode)

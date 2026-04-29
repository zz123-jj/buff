#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rm_buff_common/math_utils.hpp"
#include "rm_vision_interfaces/msg/buff_detection.hpp"
#include "rm_vision_interfaces/msg/buff_pose.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace rm_buff_solver
{
using namespace rm_buff_common;

class BuffSolverNode : public rclcpp::Node
{
public:
  explicit BuffSolverNode(const rclcpp::NodeOptions & options) : Node("buff_solver", options)
  {
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/buff/detection");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/buff/pose");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.02);
    auto camera_matrix = declare_parameter<std::vector<double>>(
      "camera_matrix", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    auto distort_coeffs = declare_parameter<std::vector<double>>(
      "distort_coeffs", std::vector<double>{0, 0, 0, 0, 0});
    if (camera_matrix.size() != 9 || distort_coeffs.size() != 5) {
      throw std::runtime_error("camera_matrix must have 9 values and distort_coeffs must have 5 values");
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix_eigen(camera_matrix.data());
    Eigen::Matrix<double, 1, 5> distort_coeffs_eigen(distort_coeffs.data());
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      cv::eigen2cv(camera_matrix_eigen, camera_matrix_);
      cv::eigen2cv(distort_coeffs_eigen, distort_coeffs_);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<rm_vision_interfaces::msg::BuffPose>(pose_topic_, 10);
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&BuffSolverNode::camera_info_callback, this, std::placeholders::_1));
    subscription_ = create_subscription<rm_vision_interfaces::msg::BuffDetection>(
      detection_topic_, 10, [this](const rm_vision_interfaces::msg::BuffDetection::ConstSharedPtr msg) {
        detection_callback(msg);
      });

    RCLCPP_INFO(
      get_logger(), "buff solver started: detection=%s camera_info=%s pose=%s",
      detection_topic_.c_str(), camera_info_topic_.c_str(), pose_topic_.c_str());
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    cv::Mat camera_matrix(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_matrix.at<double>(row, col) = msg->k[static_cast<std::size_t>(row * 3 + col)];
      }
    }

    cv::Mat distort_coeffs;
    if (msg->d.empty()) {
      distort_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    } else {
      distort_coeffs = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
      for (std::size_t i = 0; i < msg->d.size(); ++i) {
        distort_coeffs.at<double>(0, static_cast<int>(i)) = msg->d[i];
      }
    }

    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_matrix_ = camera_matrix;
      distort_coeffs_ = distort_coeffs;
      has_camera_info_ = true;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != camera_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CameraInfo frame_id is %s, but solver camera_frame is %s",
        msg->header.frame_id.c_str(), camera_frame_.c_str());
    }
  }

  void detection_callback(const rm_vision_interfaces::msg::BuffDetection::ConstSharedPtr msg)
  {
    rm_vision_interfaces::msg::BuffPose output;
    output.header = msg->header;
    output.header.frame_id = world_frame_;
    if (!msg->detected) {
      publisher_->publish(output);
      return;
    }

    Eigen::Isometry3d T_world_camera;
    try {
      auto transform = tf_buffer_->lookupTransform(
        world_frame_, camera_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(tf_timeout_sec_));
      T_world_camera = transform_to_eigen(transform);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed: %s", e.what());
      publisher_->publish(output);
      return;
    }

    std::vector<cv::Point2f> image_points;
    for (int i = 0; i < 4; ++i) {
      image_points.emplace_back(msg->keypoints[i].x, msg->keypoints[i].y);
    }

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::Mat camera_matrix;
    cv::Mat distort_coeffs;
    bool camera_info_received = false;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_matrix = camera_matrix_.clone();
      distort_coeffs = distort_coeffs_.clone();
      camera_info_received = has_camera_info_;
    }
    if (!camera_info_received) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CameraInfo has not been received on %s; using fallback camera intrinsics",
        camera_info_topic_.c_str());
    }
    if (!cv::solvePnP(object_points_fourth_, image_points, camera_matrix, distort_coeffs, rvec, tvec, false,
          cv::SOLVEPNP_IPPE)) {
      publisher_->publish(output);
      return;
    }

    cv::Mat R_buff2camera_cv;
    cv::Rodrigues(rvec, R_buff2camera_cv);
    Eigen::Matrix3d R_buff2camera;
    cv::cv2eigen(R_buff2camera_cv, R_buff2camera);
    Eigen::Vector3d t_buff2camera(tvec[0], tvec[1], tvec[2]);

    Eigen::Vector3d blade_in_buff(0.0, 0.0, 0.7);
    Eigen::Vector3d blade_in_camera = R_buff2camera * blade_in_buff + t_buff2camera;
    Eigen::Matrix3d R_buff2world = T_world_camera.linear() * R_buff2camera;
    Eigen::Vector3d center_in_world = T_world_camera * t_buff2camera;
    Eigen::Vector3d blade_in_world = T_world_camera * blade_in_camera;
    Eigen::Vector3d center_ypd = xyz_to_ypd(center_in_world);
    Eigen::Vector3d blade_ypd = xyz_to_ypd(blade_in_world);
    Eigen::Vector3d ypr = eulers_zyx(R_buff2world);

    output.solved = true;
    output.center_ypd.x = center_ypd.x();
    output.center_ypd.y = center_ypd.y();
    output.center_ypd.z = center_ypd.z();
    output.buff_ypr.x = ypr.x();
    output.buff_ypr.y = ypr.y();
    output.buff_ypr.z = ypr.z();
    output.blade_ypd.x = blade_ypd.x();
    output.blade_ypd.y = blade_ypd.y();
    output.blade_ypd.z = blade_ypd.z();
    output.blade_world.x = blade_in_world.x();
    output.blade_world.y = blade_in_world.y();
    output.blade_world.z = blade_in_world.z();
    publisher_->publish(output);
  }

  std::string detection_topic_;
  std::string pose_topic_;
  std::string camera_info_topic_;
  std::string world_frame_;
  std::string camera_frame_;
  double tf_timeout_sec_;
  std::mutex camera_info_mutex_;
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  bool has_camera_info_ = false;

  const std::vector<cv::Point3f> object_points_fourth_ = {
    cv::Point3f(0, 0, 827e-3), cv::Point3f(0, 127e-3, 700e-3),
    cv::Point3f(0, 0, 573e-3), cv::Point3f(0, -127e-3, 700e-3)};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<rm_vision_interfaces::msg::BuffDetection>::SharedPtr subscription_;
  rclcpp::Publisher<rm_vision_interfaces::msg::BuffPose>::SharedPtr publisher_;
};

}  // namespace rm_buff_solver

RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff_solver::BuffSolverNode)

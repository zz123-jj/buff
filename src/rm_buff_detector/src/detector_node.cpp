#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <openvino/openvino.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <filesystem>
#include <functional>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <chrono>
#include <vector>

#include "rm_vision_interfaces/msg/buff_detection.hpp"
#include <image_transport/image_transport.hpp>
namespace rm_buff_detector
{
namespace
{
std::string resolve_path(const std::string & repository_root, const std::string & path)
{
  std::filesystem::path fs_path(path);
  if (fs_path.is_absolute()) return fs_path.string();
  return (std::filesystem::path(repository_root) / fs_path).string();
}

geometry_msgs::msg::Point to_point(const cv::Point2f & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = 0.0;
  return msg;
}
}  // namespace

class BuffDetectorNode : public rclcpp::Node
{
public:
  explicit BuffDetectorNode(const rclcpp::NodeOptions & options) : Node("buff_detector", options)
  {
    repository_root_ = declare_parameter<std::string>("repository_root", "");
    if (repository_root_.empty()) {
      repository_root_ = std::filesystem::weakly_canonical(
                           std::filesystem::path(__FILE__).parent_path() / "../../../..")
                           .string();
    }
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/buff/detection");
    debug_ = declare_parameter<bool>("debug", false);
    debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/buff/debug_img");
    image_reliable_ = declare_parameter<bool>("image_reliable", true);
    lock_target_ = declare_parameter<bool>("lock_target", true);
    lock_max_distance_px_ = declare_parameter<double>("lock_max_distance_px", 180.0);
    switch_confidence_margin_ = declare_parameter<double>("switch_confidence_margin", 0.20);
    lock_lost_threshold_ = declare_parameter<int>("lock_lost_threshold", 5);
    model_path_ =
      resolve_path(repository_root_, declare_parameter<std::string>("model_path", "src/rm_buff_detector/model/yolo11_buff_int8.xml"));
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.7);

    model_ = core_.read_model(model_path_);
    compiled_model_ = core_.compile_model(model_, "CPU");
    infer_request_ = compiled_model_.create_infer_request();
    input_tensor_ = infer_request_.get_input_tensor();
    input_tensor_.set_shape({1, 3, input_size_, input_size_});

    publisher_ = create_publisher<rm_vision_interfaces::msg::BuffDetection>(detection_topic_, 10);
    debug_image_publisher_ = image_transport::create_publisher(this, debug_image_topic_);
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    if (image_reliable_) {
      image_qos.reliable();
    } else {
      image_qos.best_effort();
    }
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos,
      std::bind(&BuffDetectorNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "buff detector started: image=%s image_reliable=%s detection=%s debug=%s debug_image=%s lock=%s model=%s",
      image_topic_.c_str(), image_reliable_ ? "true" : "false",
      detection_topic_.c_str(), debug_ ? "true" : "false", debug_image_topic_.c_str(),
      lock_target_ ? "true" : "false", model_path_.c_str());
  }

private:
  struct Detection
  {
    double confidence = 0.0;
    std::vector<cv::Point2f> keypoints;
  };

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    debug_ = get_parameter("debug").as_bool();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "received image: encoding=%s size=%ux%u frame=%s",
      msg->encoding.c_str(), msg->width, msg->height, msg->header.frame_id.c_str());

    rm_vision_interfaces::msg::BuffDetection output;
    output.header = msg->header;

    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cv_bridge failed: %s", e.what());
      publisher_->publish(output);
      return;
    }

    auto detection = detect(cv_image->image);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - callback_start);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "detector callback finished: detected=%s elapsed=%ld ms",
      detection.has_value() ? "true" : "false", elapsed_ms.count());
    if (!detection.has_value()) {
      publisher_->publish(output);
      publish_debug_image(output.header, cv_image->image, std::nullopt, std::nullopt);
      return;
    }

    output.detected = true;
    output.confidence = detection->confidence;
    for (std::size_t i = 0; i < keypoint_count_; ++i) {
      output.keypoints[i] = to_point(detection->keypoints[i]);
    }
    auto r_center = get_r_center(detection->keypoints, cv_image->image);
    output.r_center = to_point(r_center);
    publisher_->publish(output);
    publish_debug_image(output.header, cv_image->image, detection, r_center);
  }

  std::optional<Detection> detect(const cv::Mat & bgr_image)
  {
    if (bgr_image.empty()) return std::nullopt;

    auto factor = fill_tensor(bgr_image);
    infer_request_.infer();

    const ov::Tensor output = infer_request_.get_output_tensor();
    const auto shape = output.get_shape();
    const auto * buffer = output.data<const float>();
    cv::Mat det_output(static_cast<int>(shape[1]), static_cast<int>(shape[2]), CV_32F, const_cast<float *>(buffer));

    std::vector<Detection> detections;
    for (int i = 0; i < det_output.cols; ++i) {
      auto confidence = det_output.at<float>(4, i);
      if (confidence < confidence_threshold_) {
        continue;
      }

      Detection detection;
      detection.confidence = confidence;
      cv::Mat keypoints = det_output.col(i).rowRange(5, 5 + keypoint_count_ * 2);
      for (std::size_t j = 0; j < keypoint_count_; ++j) {
        detection.keypoints.emplace_back(
          keypoints.at<float>(static_cast<int>(j * 2), 0) * factor,
          keypoints.at<float>(static_cast<int>(j * 2 + 1), 0) * factor);
      }
      detections.emplace_back(std::move(detection));
    }

    if (detections.empty()) {
      if (++lock_lost_count_ > lock_lost_threshold_) {
        has_locked_target_ = false;
      }
      return std::nullopt;
    }

    auto detection = select_detection(detections);
    lock_lost_count_ = 0;
    last_target_center_ = detection_center(detection);
    has_locked_target_ = true;
    return detection;
  }

  Detection select_detection(const std::vector<Detection> & detections) const
  {
    auto best_it = std::max_element(
      detections.begin(), detections.end(),
      [](const Detection & lhs, const Detection & rhs) { return lhs.confidence < rhs.confidence; });
    if (!lock_target_ || !has_locked_target_) {
      return *best_it;
    }

    const Detection * nearest = nullptr;
    auto nearest_distance = std::numeric_limits<double>::max();
    for (const auto & detection : detections) {
      const auto distance = cv::norm(detection_center(detection) - last_target_center_);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = &detection;
      }
    }

    if (nearest != nullptr && nearest_distance <= lock_max_distance_px_ &&
      nearest->confidence + switch_confidence_margin_ >= best_it->confidence) {
      return *nearest;
    }
    return *best_it;
  }

  cv::Point2f detection_center(const Detection & detection) const
  {
    cv::Point2f center(0.0F, 0.0F);
    for (const auto & point : detection.keypoints) {
      center += point;
    }
    return center * (1.0F / static_cast<float>(std::max<std::size_t>(detection.keypoints.size(), 1)));
  }

  float fill_tensor(const cv::Mat & bgr_image)
  {
    const float scale =
      std::min(input_size_ / static_cast<float>(bgr_image.rows), input_size_ / static_cast<float>(bgr_image.cols));
    cv::Mat blob_image;
    cv::Matx23f matrix{scale, 0.0F, 0.0F, 0.0F, scale, 0.0F};
    cv::warpAffine(bgr_image, blob_image, matrix, cv::Size(input_size_, input_size_));
    blob_image.convertTo(blob_image, CV_32F, 1.0 / 255.0);
    cv::cvtColor(blob_image, blob_image, cv::COLOR_BGR2RGB);

    auto * data = input_tensor_.data<float>();
    for (std::size_t c = 0; c < 3; ++c) {
      for (std::size_t h = 0; h < input_size_; ++h) {
        for (std::size_t w = 0; w < input_size_; ++w) {
          data[c * input_size_ * input_size_ + h * input_size_ + w] =
            blob_image.at<cv::Vec3f>(static_cast<int>(h), static_cast<int>(w))[c];
        }
      }
    }
    return 1.0F / scale;
  }

  cv::Point2f get_r_center(const std::vector<cv::Point2f> & keypoints, const cv::Mat & bgr_image) const
  {
    auto rough_center = (keypoints[5] - keypoints[4]) * 1.4F + keypoints[4];

    cv::Mat gray;
    cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(binary, binary, kernel);

    auto radius = cv::norm(keypoints[2] - keypoints[4]) * 0.8;
    cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8U);
    cv::circle(mask, rough_center, radius, cv::Scalar(255), -1);
    cv::bitwise_and(binary, mask, binary);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    auto r_center = rough_center;
    double best_score = 1e9;
    for (const auto & contour : contours) {
      if (contour.size() < 5) continue;
      auto rect = cv::minAreaRect(contour);
      auto width = std::max(rect.size.width, 1.0F);
      auto height = std::max(rect.size.height, 1.0F);
      auto ratio = std::max(width / height, height / width);
      auto score = ratio + cv::norm(rect.center - rough_center) / std::max(radius / 3.0, 1.0);
      if (score < best_score) {
        best_score = score;
        r_center = rect.center;
      }
    }
    return r_center;
  }

  void publish_debug_image(
    const std_msgs::msg::Header & header, const cv::Mat & bgr_image,
    const std::optional<Detection> & detection, const std::optional<cv::Point2f> & r_center) const
  {
    if (!debug_ || bgr_image.empty()) return;

    cv::Mat debug_image = bgr_image.clone();
    if (detection.has_value()) {
      static const std::vector<cv::Scalar> colors{
        cv::Scalar(0, 0, 255), cv::Scalar(0, 165, 255), cv::Scalar(0, 255, 255),
        cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 255)};
      for (std::size_t i = 0; i < detection->keypoints.size(); ++i) {
        const auto & point = detection->keypoints[i];
        const auto & color = colors[i % colors.size()];
        cv::circle(debug_image, point, 5, color, -1, cv::LINE_AA);
        cv::circle(debug_image, point, 8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::putText(
          debug_image, std::to_string(i), point + cv::Point2f(8.0F, -8.0F), cv::FONT_HERSHEY_SIMPLEX, 0.6,
          color, 2, cv::LINE_AA);
      }
      if (r_center.has_value()) {
        cv::drawMarker(
          debug_image, *r_center, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
        cv::putText(
          debug_image, "R", *r_center + cv::Point2f(8.0F, -8.0F), cv::FONT_HERSHEY_SIMPLEX, 0.7,
          cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      }
    }

    debug_image_publisher_.publish(
      *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg());
  }

  static constexpr std::size_t keypoint_count_ = 6;
  static constexpr std::size_t input_size_ = 640;

  std::string repository_root_;
  std::string image_topic_;
  std::string detection_topic_;
  std::string debug_image_topic_;
  std::string model_path_;
  double confidence_threshold_;
  bool image_reliable_ = true;
  bool debug_ = false;
  bool lock_target_ = true;
  double lock_max_distance_px_ = 180.0;
  double switch_confidence_margin_ = 0.20;
  int lock_lost_threshold_ = 5;
  int lock_lost_count_ = 0;
  bool has_locked_target_ = false;
  cv::Point2f last_target_center_;

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::Tensor input_tensor_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<rm_vision_interfaces::msg::BuffDetection>::SharedPtr publisher_;
  image_transport::Publisher debug_image_publisher_;
};

}  // namespace rm_buff_detector

RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff_detector::BuffDetectorNode)

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if __has_include("cv_bridge/cv_bridge.hpp")
  #include "cv_bridge/cv_bridge.hpp"
#elif __has_include("cv_bridge/cv_bridge.h")
  #include "cv_bridge/cv_bridge.h"
#else
  #error "cv_bridge headers not found"
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "buff_interfaces/msg/buff_world_model.hpp"

namespace
{
constexpr double kBladeInterval = 2.0 * M_PI / 5.0;

Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::TransformStamped& transform)
{
    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;

    Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
    q.normalize();

    Eigen::Isometry3d eigen = Eigen::Isometry3d::Identity();
    eigen.linear() = q.toRotationMatrix();
    eigen.translation() = Eigen::Vector3d(translation.x, translation.y, translation.z);
    return eigen;
}

Eigen::Vector3d rotateRodrigues(
    const Eigen::Vector3d& vector,
    const Eigen::Vector3d& axis,
    double angle)
{
    const Eigen::Vector3d k = axis.normalized();
    return vector * std::cos(angle)
         + k.cross(vector) * std::sin(angle)
         + k * k.dot(vector) * (1.0 - std::cos(angle));
}

}  // namespace

class BuffDebugVisualizer : public rclcpp::Node
{
public:
    explicit BuffDebugVisualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("buff_debug_visualizer_node", options)
    {
        declare_parameter<std::string>("topics.image", "/image_raw");
        declare_parameter<std::string>("topics.camera_info", "/camera_info");
        declare_parameter<std::string>("topics.world_model", "/buff/world_model");
        declare_parameter<std::string>(
            "topics.output_compressed_image",
            "/buff/world_model_debug_image/compressed");
        declare_parameter<std::string>("frames.camera", "camera_optical_frame");
        declare_parameter<bool>("enabled", true);
        declare_parameter<int>("circle_samples", 144);
        declare_parameter<double>("normal_length_scale", 0.8);
        declare_parameter<double>("stale_timeout_sec", 0.25);
        declare_parameter<double>("max_publish_hz", 0.0);
        declare_parameter<double>("output_scale", 0.5);
        declare_parameter<int>("jpeg_quality", 80);

        image_topic_ = get_parameter("topics.image").as_string();
        camera_info_topic_ = get_parameter("topics.camera_info").as_string();
        world_model_topic_ = get_parameter("topics.world_model").as_string();
        output_topic_ = get_parameter("topics.output_compressed_image").as_string();
        camera_frame_ = get_parameter("frames.camera").as_string();
        enabled_ = get_parameter("enabled").as_bool();
        circle_samples_ = std::max(
            16,
            static_cast<int>(get_parameter("circle_samples").as_int()));
        normal_length_scale_ = get_parameter("normal_length_scale").as_double();
        stale_timeout_sec_ = get_parameter("stale_timeout_sec").as_double();
        max_publish_hz_ = get_parameter("max_publish_hz").as_double();
        output_scale_ = std::clamp(get_parameter("output_scale").as_double(), 0.05, 1.0);
        jpeg_quality_ = std::clamp(
            static_cast<int>(get_parameter("jpeg_quality").as_int()), 1, 100);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

        world_model_sub_ = create_subscription<buff_interfaces::msg::BuffWorldModel>(
            world_model_topic_, rclcpp::SensorDataQoS(),
            [this](const buff_interfaces::msg::BuffWorldModel::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(world_model_mutex_);
                latest_world_model_ = msg;
            });

        image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            output_topic_, rclcpp::SensorDataQoS());

        RCLCPP_INFO(
            get_logger(),
            "BuffDebugVisualizer started. compressed_output=%s scale=%.2f jpeg_quality=%d",
            output_topic_.c_str(), output_scale_, jpeg_quality_);
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);

        if (msg->d.empty()) {
            dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        } else {
            dist_coeffs_ = cv::Mat(msg->d, true).reshape(1, 1);
        }

        has_camera_info_ = true;
        camera_info_sub_.reset();
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!enabled_ || !has_camera_info_) {
            return;
        }

        const rclcpp::Time now = get_clock()->now();
        if (max_publish_hz_ > 0.0 && last_publish_time_.nanoseconds() != 0) {
            const double interval = (now - last_publish_time_).seconds();
            if (interval < 1.0 / max_publish_hz_) {
                return;
            }
        }

        cv::Mat image;
        try {
            image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "cv_bridge failed: %s", error.what());
            return;
        }

        buff_interfaces::msg::BuffWorldModel::SharedPtr world_model;
        {
            std::lock_guard<std::mutex> lock(world_model_mutex_);
            world_model = latest_world_model_;
        }

        if (world_model && world_model->valid) {
            const double age = std::abs((rclcpp::Time(msg->header.stamp) -
                                         rclcpp::Time(world_model->header.stamp)).seconds());
            if (stale_timeout_sec_ <= 0.0 || age <= stale_timeout_sec_) {
                drawWorldModel(image, *world_model, msg->header.stamp);
            } else {
                cv::putText(
                    image, "buff model stale", cv::Point(20, 36),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 180, 255), 2);
            }
        } else {
            cv::putText(
                image, "buff world model invalid", cv::Point(20, 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }

        if (output_scale_ > 0.0 && output_scale_ < 1.0) {
            cv::resize(image, image, cv::Size(), output_scale_, output_scale_, cv::INTER_AREA);
        }

        sensor_msgs::msg::CompressedImage output;
        output.header = msg->header;
        output.format = "bgr8; jpeg compressed bgr8";
        const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        if (!cv::imencode(".jpg", image, output.data, params)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Failed to encode buff world model debug image as JPEG.");
            return;
        }
        image_pub_->publish(output);
        last_publish_time_ = now;
    }

    bool lookupCameraFromModel(
        const std::string& model_frame,
        const builtin_interfaces::msg::Time& stamp,
        Eigen::Isometry3d& camera_from_model)
    {
        try {
            const auto transform = tf_buffer_->lookupTransform(
                camera_frame_, model_frame, stamp, std::chrono::milliseconds(5));
            camera_from_model = transformToEigen(transform);
            return true;
        } catch (const tf2::TransformException& first_error) {
            try {
                const auto transform = tf_buffer_->lookupTransform(
                    camera_frame_, model_frame, tf2::TimePointZero,
                    std::chrono::milliseconds(5));
                camera_from_model = transformToEigen(transform);
                return true;
            } catch (const tf2::TransformException& second_error) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "TF Error(target=%s, source=%s): %s; latest fallback: %s",
                    camera_frame_.c_str(), model_frame.c_str(),
                    first_error.what(), second_error.what());
                return false;
            }
        }
    }

    bool projectWorldPoints(
        const std::vector<Eigen::Vector3d>& world_points,
        const Eigen::Isometry3d& camera_from_model,
        std::vector<cv::Point2f>& image_points,
        std::vector<bool>& visible) const
    {
        std::vector<cv::Point3f> camera_points;
        camera_points.reserve(world_points.size());
        visible.assign(world_points.size(), false);

        for (const auto& world_point : world_points) {
            const Eigen::Vector3d camera_point = camera_from_model * world_point;
            camera_points.emplace_back(
                static_cast<float>(camera_point.x()),
                static_cast<float>(camera_point.y()),
                static_cast<float>(camera_point.z()));
        }

        cv::projectPoints(
            camera_points, cv::Vec3d::zeros(), cv::Vec3d::zeros(),
            camera_matrix_, dist_coeffs_, image_points);

        for (std::size_t i = 0; i < camera_points.size(); ++i) {
            visible[i] = camera_points[i].z > 1e-4 &&
                         std::isfinite(image_points[i].x) &&
                         std::isfinite(image_points[i].y);
        }

        return image_points.size() == world_points.size();
    }

    void drawWorldModel(
        cv::Mat& image,
        const buff_interfaces::msg::BuffWorldModel& world_model,
        const builtin_interfaces::msg::Time& image_stamp)
    {
        const Eigen::Vector3d center(
            world_model.circle_center_world.x,
            world_model.circle_center_world.y,
            world_model.circle_center_world.z);
        Eigen::Vector3d normal(
            world_model.circle_axis_world.x,
            world_model.circle_axis_world.y,
            world_model.circle_axis_world.z);
        const Eigen::Vector3d observed(
            world_model.observed_point_world.x,
            world_model.observed_point_world.y,
            world_model.observed_point_world.z);
        const Eigen::Vector3d fitted(
            world_model.fitted_point_world.x,
            world_model.fitted_point_world.y,
            world_model.fitted_point_world.z);
        const double radius = world_model.circle_radius;

        if (!center.allFinite() || !normal.allFinite() || !observed.allFinite() ||
            !fitted.allFinite() ||
            normal.norm() < 1e-6 || radius <= 1e-5) {
            cv::putText(
                image, "buff world model incomplete", cv::Point(20, 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 180, 255), 2);
            return;
        }

        normal.normalize();
        Eigen::Vector3d current_axis = fitted - center;
        current_axis -= normal * current_axis.dot(normal);
        if (current_axis.norm() < 1e-6) {
            current_axis = normal.unitOrthogonal();
        } else {
            current_axis.normalize();
        }
        const Eigen::Vector3d model_current = center + radius * current_axis;
        Eigen::Vector3d basis_u = current_axis;
        Eigen::Vector3d basis_v = normal.cross(basis_u).normalized();

        Eigen::Isometry3d camera_from_model;
        if (!lookupCameraFromModel(world_model.header.frame_id, image_stamp, camera_from_model)) {
            return;
        }

        std::vector<Eigen::Vector3d> circle_points;
        circle_points.reserve(static_cast<std::size_t>(circle_samples_));
        for (int i = 0; i < circle_samples_; ++i) {
            const double theta =
                2.0 * M_PI * static_cast<double>(i) / static_cast<double>(circle_samples_);
            circle_points.push_back(center + radius * (std::cos(theta) * basis_u +
                                                       std::sin(theta) * basis_v));
        }

        std::vector<cv::Point2f> circle_pixels;
        std::vector<bool> circle_visible;
        projectWorldPoints(circle_points, camera_from_model, circle_pixels, circle_visible);

        for (int i = 0; i < circle_samples_; ++i) {
            const int next = (i + 1) % circle_samples_;
            if (circle_visible[i] && circle_visible[next]) {
                cv::line(
                    image, circle_pixels[i], circle_pixels[next],
                    cv::Scalar(255, 220, 0), 2, cv::LINE_AA);
            }
        }

        std::vector<Eigen::Vector3d> key_points;
        key_points.reserve(9);
        key_points.push_back(center);
        key_points.push_back(center + normal * radius * normal_length_scale_);
        key_points.push_back(observed);
        key_points.push_back(model_current);
        for (int i = 0; i < 5; ++i) {
            key_points.push_back(center + rotateRodrigues(
                model_current - center, normal, static_cast<double>(i) * kBladeInterval));
        }

        std::vector<cv::Point2f> pixels;
        std::vector<bool> visible;
        projectWorldPoints(key_points, camera_from_model, pixels, visible);

        if (visible[0]) {
            cv::drawMarker(
                image, pixels[0], cv::Scalar(255, 255, 255),
                cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
            cv::putText(
                image, "C", pixels[0] + cv::Point2f(6.0f, -6.0f),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2);
        }

        if (visible[0] && visible[1]) {
            cv::arrowedLine(
                image, pixels[0], pixels[1], cv::Scalar(255, 0, 255),
                3, cv::LINE_AA, 0, 0.18);
            cv::putText(
                image, "n", pixels[1] + cv::Point2f(6.0f, -6.0f),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 255), 2);
        }

        if (world_model.has_observation && visible[2]) {
            cv::circle(image, pixels[2], 8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::putText(
                image, "obs", pixels[2] + cv::Point2f(8.0f, 16.0f),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        if (visible[3]) {
            cv::circle(image, pixels[3], 6, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            cv::putText(
                image, "fit", pixels[3] + cv::Point2f(8.0f, -10.0f),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        for (int i = 0; i < 5; ++i) {
            const std::size_t idx = static_cast<std::size_t>(4 + i);
            if (!visible[idx]) {
                continue;
            }

            const cv::Scalar color = i == 0 ? cv::Scalar(0, 255, 0)
                                            : cv::Scalar(0, 180, 255);
            cv::circle(image, pixels[idx], i == 0 ? 7 : 5, color, -1, cv::LINE_AA);
            cv::putText(
                image, std::to_string(i), pixels[idx] + cv::Point2f(7.0f, -7.0f),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
        }

        const std::string mode = world_model.mode == 1 ? "BIG" : "SMALL";
        const std::string text = "world circle " + mode +
            " r=" + std::to_string(radius).substr(0, 5) +
            " angle=" + std::to_string(world_model.current_angle).substr(0, 6) +
            " obs_age=" + std::to_string(world_model.seconds_since_observation).substr(0, 4);
        cv::putText(
            image, text, cv::Point(20, 36),
            cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
    }

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string world_model_topic_;
    std::string output_topic_;
    std::string camera_frame_;

    bool enabled_ = true;
    int circle_samples_ = 144;
    double normal_length_scale_ = 0.8;
    double stale_timeout_sec_ = 0.25;
    double max_publish_hz_ = 0.0;
    double output_scale_ = 0.5;
    int jpeg_quality_ = 80;
    bool has_camera_info_ = false;
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<buff_interfaces::msg::BuffWorldModel>::SharedPtr world_model_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;

    std::mutex world_model_mutex_;
    buff_interfaces::msg::BuffWorldModel::SharedPtr latest_world_model_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffDebugVisualizer)

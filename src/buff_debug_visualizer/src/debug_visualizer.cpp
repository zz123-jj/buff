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
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "buff_interfaces/msg/buff_aiming_data.hpp"

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
        declare_parameter<std::string>("topics.aiming_data", "/buff/aiming_data");
        declare_parameter<std::string>("topics.output_image", "/buff/world_model_debug_image");
        declare_parameter<std::string>("frames.camera", "camera_optical_frame");
        declare_parameter<bool>("enabled", true);
        declare_parameter<int>("circle_samples", 144);
        declare_parameter<double>("normal_length_scale", 0.8);
        declare_parameter<double>("stale_timeout_sec", 0.25);

        image_topic_ = get_parameter("topics.image").as_string();
        camera_info_topic_ = get_parameter("topics.camera_info").as_string();
        aiming_topic_ = get_parameter("topics.aiming_data").as_string();
        output_topic_ = get_parameter("topics.output_image").as_string();
        camera_frame_ = get_parameter("frames.camera").as_string();
        enabled_ = get_parameter("enabled").as_bool();
        circle_samples_ = std::max(
            16,
            static_cast<int>(get_parameter("circle_samples").as_int()));
        normal_length_scale_ = get_parameter("normal_length_scale").as_double();
        stale_timeout_sec_ = get_parameter("stale_timeout_sec").as_double();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

        aiming_sub_ = create_subscription<buff_interfaces::msg::BuffAimingData>(
            aiming_topic_, rclcpp::SensorDataQoS(),
            [this](const buff_interfaces::msg::BuffAimingData::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(aiming_mutex_);
                latest_aiming_ = msg;
            });

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::SensorDataQoS());

        RCLCPP_INFO(
            get_logger(),
            "BuffDebugVisualizer started. output=%s",
            output_topic_.c_str());
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

        cv::Mat image;
        try {
            image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "cv_bridge failed: %s", error.what());
            return;
        }

        buff_interfaces::msg::BuffAimingData::SharedPtr aiming;
        {
            std::lock_guard<std::mutex> lock(aiming_mutex_);
            aiming = latest_aiming_;
        }

        if (aiming && aiming->is_tracking) {
            const double age = std::abs((rclcpp::Time(msg->header.stamp) -
                                         rclcpp::Time(aiming->header.stamp)).seconds());
            if (stale_timeout_sec_ <= 0.0 || age <= stale_timeout_sec_) {
                drawWorldModel(image, *aiming, msg->header.stamp);
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

        auto output = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        output->header.frame_id = msg->header.frame_id;
        image_pub_->publish(*output);
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
        const buff_interfaces::msg::BuffAimingData& aiming,
        const builtin_interfaces::msg::Time& image_stamp)
    {
        const Eigen::Vector3d center(
            aiming.r_x_3d,
            aiming.r_y_3d,
            aiming.r_z_3d);
        Eigen::Vector3d normal(
            aiming.axis_x_3d,
            aiming.axis_y_3d,
            aiming.axis_z_3d);
        const Eigen::Vector3d current(
            aiming.target_x_3d,
            aiming.target_y_3d,
            aiming.target_z_3d);
        const double radius = aiming.filter_radius;

        if (!center.allFinite() || !normal.allFinite() || !current.allFinite() ||
            normal.norm() < 1e-6 || radius <= 1e-5) {
            cv::putText(
                image, "buff world model incomplete", cv::Point(20, 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 180, 255), 2);
            return;
        }

        normal.normalize();
        Eigen::Vector3d basis_u = current - center;
        basis_u -= normal * basis_u.dot(normal);
        if (basis_u.norm() < 1e-6) {
            basis_u = normal.unitOrthogonal() * radius;
        }
        basis_u.normalize();
        Eigen::Vector3d basis_v = normal.cross(basis_u).normalized();

        Eigen::Isometry3d camera_from_model;
        if (!lookupCameraFromModel(aiming.header.frame_id, image_stamp, camera_from_model)) {
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
        key_points.reserve(8);
        key_points.push_back(center);
        key_points.push_back(center + normal * radius * normal_length_scale_);
        key_points.push_back(current);
        for (int i = 0; i < 5; ++i) {
            key_points.push_back(center + rotateRodrigues(
                current - center, normal, static_cast<double>(i) * kBladeInterval));
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

        if (visible[2]) {
            cv::circle(image, pixels[2], 8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::putText(
                image, "obs", pixels[2] + cv::Point2f(8.0f, 16.0f),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        for (int i = 0; i < 5; ++i) {
            const std::size_t idx = static_cast<std::size_t>(3 + i);
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

        const std::string mode = aiming.is_bigbuff == 1 ? "BIG" : "SMALL";
        const std::string text = "world circle " + mode +
            " r=" + std::to_string(radius).substr(0, 5) +
            " angle=" + std::to_string(aiming.angle).substr(0, 6);
        cv::putText(
            image, text, cv::Point(20, 36),
            cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
    }

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string aiming_topic_;
    std::string output_topic_;
    std::string camera_frame_;

    bool enabled_ = true;
    int circle_samples_ = 144;
    double normal_length_scale_ = 0.8;
    double stale_timeout_sec_ = 0.25;
    bool has_camera_info_ = false;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<buff_interfaces::msg::BuffAimingData>::SharedPtr aiming_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    std::mutex aiming_mutex_;
    buff_interfaces::msg::BuffAimingData::SharedPtr latest_aiming_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffDebugVisualizer)

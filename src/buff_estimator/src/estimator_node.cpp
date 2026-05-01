#include "predictor.hpp"
#include "world_model.hpp"

#include "buff_interfaces/msg/buff_target.hpp"
#include "buff_interfaces/msg/buff_world_model.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr double kBladeInterval = 2.0 * M_PI / 5.0;
constexpr double kSmallBuffAngularSpeed = M_PI / 3.0;

Eigen::Quaterniond transformRotationToEigen(const geometry_msgs::msg::TransformStamped& transform)
{
    const auto& rotation = transform.transform.rotation;
    Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
    q.normalize();
    return q;
}

geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& p)
{
    geometry_msgs::msg::Point msg;
    msg.x = p.x();
    msg.y = p.y();
    msg.z = p.z();
    return msg;
}

geometry_msgs::msg::Vector3 toVector3Msg(const Eigen::Vector3d& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

Eigen::Vector3d projectPointToCircle(
    const buff_estimator::CircleModel& model,
    const Eigen::Vector3d& p)
{
    if (!model.valid || model.radius <= 1e-6)
    {
        return p;
    }

    Eigen::Vector3d radial = p - model.center;
    radial -= model.normal * radial.dot(model.normal);
    if (!radial.allFinite() || radial.norm() < 1e-6)
    {
        return model.center + model.radius * model.basis_u;
    }
    return model.center + model.radius * radial.normalized();
}

Eigen::Vector3d pointOnCircleAtAngle(
    const buff_estimator::CircleModel& model,
    double angle)
{
    if (!model.valid || model.radius <= 1e-6)
    {
        return Eigen::Vector3d::Zero();
    }
    return model.center + model.radius *
        (std::cos(angle) * model.basis_u + std::sin(angle) * model.basis_v);
}

}  // namespace

class BuffEstimatorNode : public rclcpp::Node
{
public:
    explicit BuffEstimatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("buff_estimator_node", options),
          point_buffer_(5.0)
    {
        predictor_ = std::make_unique<Big_Buff_Predictor>();

        declareParameters();
        readParameters();

        velocity_median_filter_ =
            std::make_unique<MedianFilter>(std::max(1, velocity_median_window_));

        predictor_->set_fit_config(
            fit_offset_sum_,
            fit_window_sec_,
            fit_min_samples_,
            fit_a_range_[0],
            fit_a_range_[1],
            fit_omega_range_[0],
            fit_omega_range_[1],
            fit_phi_range_[0],
            fit_phi_range_[1]);

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

        target_sub_ = this->create_subscription<buff_interfaces::msg::BuffTarget>(
            "/buff/target", rclcpp::SensorDataQoS(),
            [this](const buff_interfaces::msg::BuffTarget::SharedPtr msg) { targetCallback(msg); });

        world_model_pub_ = this->create_publisher<buff_interfaces::msg::BuffWorldModel>(
            "/buff/world_model", rclcpp::SensorDataQoS());

        if (model_publish_rate_hz_ > 0.0) {
            const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / model_publish_rate_hz_));
            model_publish_timer_ = this->create_wall_timer(
                period, [this]() { publishPredictedWorldModel(this->now(), 0, false); });
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (debug_mode_) {
            debug_csv_.open(debug_csv_path_);
            if (debug_csv_.is_open()) {
                debug_csv_
                    << "t,px,py,pz,model_valid,cx,cy,cz,nx,ny,nz,radius,plane_rms,circle_rms,angle_span,theta,omega\n";
            }

            point_debug_csv_.open(debug_points_csv_path_);
            if (point_debug_csv_.is_open()) {
                point_debug_csv_
                    << "t,used_for_fit,reason,raw_px,raw_py,raw_pz,px,py,pz,nx,ny,nz,pcam_x,pcam_y,pcam_z,"
                    << "reprojection_error,confidence,yolo_target_count,buffer_size\n";
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "BuffEstimator world model enabled. PnP hit point=(%.3f, %.3f, %.3f), buffer %.2fs",
            hit_point_object_.x,
            hit_point_object_.y,
            hit_point_object_.z,
            geometry_buffer_sec_);
    }

    ~BuffEstimatorNode() override
    {
        if (debug_csv_.is_open()) {
            debug_csv_.close();
        }
        if (point_debug_csv_.is_open()) {
            point_debug_csv_.close();
        }
    }

private:
    void declareParameters()
    {
        this->declare_parameter<std::string>("target_frame", "odom");
        this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<std::string>("debug_csv_path", "buff_estimator_debug.csv");
        this->declare_parameter<std::string>("debug_points_csv_path", "buff_estimator_points_debug.csv");
        this->declare_parameter<double>("max_reprojection_error_px", 12.0);
        this->declare_parameter<double>("geometry_buffer_sec", 5.0);
        this->declare_parameter<int>("geometry_min_samples", 20);
        this->declare_parameter<double>("geometry_min_span_sec", 1.0);
        this->declare_parameter<double>("max_model_residual_m", 0.25);
        this->declare_parameter<double>("max_plane_rms_m", 0.08);
        this->declare_parameter<double>("max_circle_rms_m", 0.10);
        this->declare_parameter<double>("min_fit_angle_span_rad", 0.75);
        this->declare_parameter<double>("pnp_normal_weight", 0.85);
        this->declare_parameter<double>("known_circle_radius_m", 0.75);
        this->declare_parameter<double>("model_publish_rate_hz", 60.0);
        this->declare_parameter<bool>("use_plane_constrained_points", true);
        this->declare_parameter<double>("plane_normal_alpha", 0.08);
        this->declare_parameter<double>("plane_offset_alpha", 0.05);
        this->declare_parameter<bool>("continuity_gate_enabled", true);
        this->declare_parameter<double>("max_observation_jump_m", 0.55);
        this->declare_parameter<double>("continuity_reset_sec", 0.35);
        this->declare_parameter<int>("velocity_median_window", 3);
        this->declare_parameter<double>("velocity_sample_window_sec", 4.0);
        this->declare_parameter<double>("max_velocity_sample_abs", 8.0);
        this->declare_parameter<double>("fit_window_sec", 1.5);
        this->declare_parameter<int>("fit_min_samples", 10);
        this->declare_parameter<double>("fit_offset_sum", 2.090);
        this->declare_parameter<std::vector<double>>("fit_a_range", {0.780, 1.045});
        this->declare_parameter<std::vector<double>>("fit_omega_range", {1.884, 2.000});
        this->declare_parameter<std::vector<double>>("fit_phi_range", {-M_PI, M_PI});
    }

    static std::array<double, 2> readRange(
        const std::vector<double>& values,
        const std::array<double, 2>& fallback)
    {
        if (values.size() < 2) {
            return fallback;
        }
        return {values[0], values[1]};
    }

    void readParameters()
    {
        target_frame_ = this->get_parameter("target_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
        debug_csv_path_ = this->get_parameter("debug_csv_path").as_string();
        debug_points_csv_path_ = this->get_parameter("debug_points_csv_path").as_string();
        max_reprojection_error_px_ = this->get_parameter("max_reprojection_error_px").as_double();
        geometry_buffer_sec_ = this->get_parameter("geometry_buffer_sec").as_double();
        geometry_min_samples_ = this->get_parameter("geometry_min_samples").as_int();
        geometry_min_span_sec_ = this->get_parameter("geometry_min_span_sec").as_double();
        max_model_residual_m_ = this->get_parameter("max_model_residual_m").as_double();
        max_plane_rms_m_ = this->get_parameter("max_plane_rms_m").as_double();
        max_circle_rms_m_ = this->get_parameter("max_circle_rms_m").as_double();
        min_fit_angle_span_rad_ = this->get_parameter("min_fit_angle_span_rad").as_double();
        pnp_normal_weight_ = this->get_parameter("pnp_normal_weight").as_double();
        known_circle_radius_m_ = this->get_parameter("known_circle_radius_m").as_double();
        model_publish_rate_hz_ = this->get_parameter("model_publish_rate_hz").as_double();
        use_plane_constrained_points_ =
            this->get_parameter("use_plane_constrained_points").as_bool();
        plane_normal_alpha_ = this->get_parameter("plane_normal_alpha").as_double();
        plane_offset_alpha_ = this->get_parameter("plane_offset_alpha").as_double();
        continuity_gate_enabled_ = this->get_parameter("continuity_gate_enabled").as_bool();
        max_observation_jump_m_ = this->get_parameter("max_observation_jump_m").as_double();
        continuity_reset_sec_ = this->get_parameter("continuity_reset_sec").as_double();
        velocity_median_window_ = this->get_parameter("velocity_median_window").as_int();
        velocity_sample_window_sec_ = this->get_parameter("velocity_sample_window_sec").as_double();
        max_velocity_sample_abs_ = this->get_parameter("max_velocity_sample_abs").as_double();
        fit_window_sec_ = this->get_parameter("fit_window_sec").as_double();
        fit_min_samples_ = this->get_parameter("fit_min_samples").as_int();
        fit_offset_sum_ = this->get_parameter("fit_offset_sum").as_double();

        fit_a_range_ = readRange(this->get_parameter("fit_a_range").as_double_array(), {0.780, 1.045});
        fit_omega_range_ = readRange(this->get_parameter("fit_omega_range").as_double_array(), {1.884, 2.000});
        fit_phi_range_ = readRange(this->get_parameter("fit_phi_range").as_double_array(), {-M_PI, M_PI});

        point_buffer_.setWindow(geometry_buffer_sec_);
    }

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

        has_cam_info_ = true;
        cam_info_sub_.reset();
    }

    bool transformPointToTarget(
        const geometry_msgs::msg::PointStamped& input,
        const builtin_interfaces::msg::Time& stamp,
        geometry_msgs::msg::PointStamped& output)
    {
        if (input.header.frame_id == target_frame_) {
            output = input;
            return true;
        }

        try {
            const auto tf_stamped = tf_buffer_->lookupTransform(
                target_frame_, input.header.frame_id, stamp, std::chrono::milliseconds(10));
            tf2::doTransform(input, output, tf_stamped);
            return true;
        } catch (const tf2::TransformException& first_error) {
            try {
                const auto tf_stamped = tf_buffer_->lookupTransform(
                    target_frame_, input.header.frame_id, tf2::TimePointZero,
                    std::chrono::milliseconds(10));
                tf2::doTransform(input, output, tf_stamped);
                return true;
            } catch (const tf2::TransformException& second_error) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "TF Error(target=%s, source=%s): %s; latest fallback: %s",
                    target_frame_.c_str(), input.header.frame_id.c_str(),
                    first_error.what(), second_error.what());
                return false;
            }
        }
    }

    bool solvePnpHitPoint(
        const buff_interfaces::msg::BuffTarget& msg,
        Eigen::Vector3d& p_camera,
        Eigen::Vector3d& normal_camera,
        double& reprojection_error)
    {
        if (!has_cam_info_ || !msg.has_target_keypoints) {
            return false;
        }

        std::vector<cv::Point2f> image_points;
        image_points.reserve(4);
        for (std::size_t i = 0; i < 4; ++i) {
            image_points.emplace_back(
                msg.target_keypoints[i * 2],
                msg.target_keypoints[i * 2 + 1]);
        }

        cv::Vec3d rvec;
        cv::Vec3d tvec;
        const bool ok = cv::solvePnP(
            object_points_, image_points, camera_matrix_, dist_coeffs_, rvec, tvec,
            false, cv::SOLVEPNP_IPPE);
        if (!ok || !std::isfinite(tvec[2]) || tvec[2] <= 0.0) {
            return false;
        }

        cv::Mat rotation_mat;
        cv::Rodrigues(rvec, rotation_mat);
        const Eigen::Vector3d hit_object(
            hit_point_object_.x,
            hit_point_object_.y,
            hit_point_object_.z);
        p_camera = Eigen::Vector3d(
            rotation_mat.at<double>(0, 0) * hit_object.x() +
                rotation_mat.at<double>(0, 1) * hit_object.y() +
                rotation_mat.at<double>(0, 2) * hit_object.z() + tvec[0],
            rotation_mat.at<double>(1, 0) * hit_object.x() +
                rotation_mat.at<double>(1, 1) * hit_object.y() +
                rotation_mat.at<double>(1, 2) * hit_object.z() + tvec[1],
            rotation_mat.at<double>(2, 0) * hit_object.x() +
                rotation_mat.at<double>(2, 1) * hit_object.y() +
                rotation_mat.at<double>(2, 2) * hit_object.z() + tvec[2]);
        if (!p_camera.allFinite() || p_camera.z() <= 0.0) {
            return false;
        }

        normal_camera = Eigen::Vector3d(
            rotation_mat.at<double>(0, 0),
            rotation_mat.at<double>(1, 0),
            rotation_mat.at<double>(2, 0));
        if (!normal_camera.allFinite() || normal_camera.norm() < 1e-6) {
            return false;
        }
        normal_camera.normalize();

        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points_, rvec, tvec, camera_matrix_, dist_coeffs_, projected);
        double error_sum = 0.0;
        for (std::size_t i = 0; i < projected.size(); ++i) {
            error_sum += cv::norm(projected[i] - image_points[i]);
        }
        reprojection_error = error_sum / static_cast<double>(projected.size());

        return std::isfinite(reprojection_error);
    }

    static cv::Point2f imageCenterFromTarget(const buff_interfaces::msg::BuffTarget& msg)
    {
        cv::Point2f center(0.0f, 0.0f);
        for (std::size_t i = 0; i < 4; ++i) {
            center.x += msg.target_keypoints[i * 2];
            center.y += msg.target_keypoints[i * 2 + 1];
        }
        center.x *= 0.25f;
        center.y *= 0.25f;
        return center;
    }

    bool cameraToWorld(
        const Eigen::Vector3d& p_camera,
        const builtin_interfaces::msg::Time& stamp,
        Eigen::Vector3d& p_world)
    {
        geometry_msgs::msg::PointStamped camera_point;
        camera_point.header.stamp = stamp;
        camera_point.header.frame_id = camera_frame_;
        camera_point.point.x = p_camera.x();
        camera_point.point.y = p_camera.y();
        camera_point.point.z = p_camera.z();

        geometry_msgs::msg::PointStamped world_point;
        if (!transformPointToTarget(camera_point, stamp, world_point)) {
            return false;
        }

        p_world = Eigen::Vector3d(world_point.point.x, world_point.point.y, world_point.point.z);
        return p_world.allFinite();
    }

    bool cameraNormalToWorld(
        const Eigen::Vector3d& normal_camera,
        const builtin_interfaces::msg::Time& stamp,
        Eigen::Vector3d& normal_world)
    {
        try {
            const auto tf_stamped = tf_buffer_->lookupTransform(
                target_frame_, camera_frame_, stamp, std::chrono::milliseconds(10));
            normal_world = transformRotationToEigen(tf_stamped) * normal_camera;
        } catch (const tf2::TransformException& first_error) {
            try {
                const auto tf_stamped = tf_buffer_->lookupTransform(
                    target_frame_, camera_frame_, tf2::TimePointZero, std::chrono::milliseconds(10));
                normal_world = transformRotationToEigen(tf_stamped) * normal_camera;
            } catch (const tf2::TransformException& second_error) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "TF normal Error(target=%s, source=%s): %s; latest fallback: %s",
                    target_frame_.c_str(), camera_frame_.c_str(),
                    first_error.what(), second_error.what());
                return false;
            }
        }

        if (!normal_world.allFinite() || normal_world.norm() < 1e-6) {
            return false;
        }
        normal_world.normalize();
        return true;
    }

    bool cameraRayToWorld(
        const cv::Point2f& image_point,
        const builtin_interfaces::msg::Time& stamp,
        Eigen::Vector3d& ray_origin_world,
        Eigen::Vector3d& ray_direction_world)
    {
        std::vector<cv::Point2f> input{image_point};
        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(input, undistorted, camera_matrix_, dist_coeffs_);
        if (undistorted.empty()) {
            return false;
        }

        const Eigen::Vector3d ray_camera(
            undistorted[0].x,
            undistorted[0].y,
            1.0);

        try {
            const auto tf_stamped = tf_buffer_->lookupTransform(
                target_frame_, camera_frame_, stamp, std::chrono::milliseconds(10));
            const auto rotation = transformRotationToEigen(tf_stamped);
            ray_origin_world = Eigen::Vector3d(
                tf_stamped.transform.translation.x,
                tf_stamped.transform.translation.y,
                tf_stamped.transform.translation.z);
            ray_direction_world = rotation * ray_camera;
        } catch (const tf2::TransformException& first_error) {
            try {
                const auto tf_stamped = tf_buffer_->lookupTransform(
                    target_frame_, camera_frame_, tf2::TimePointZero, std::chrono::milliseconds(10));
                const auto rotation = transformRotationToEigen(tf_stamped);
                ray_origin_world = Eigen::Vector3d(
                    tf_stamped.transform.translation.x,
                    tf_stamped.transform.translation.y,
                    tf_stamped.transform.translation.z);
                ray_direction_world = rotation * ray_camera;
            } catch (const tf2::TransformException& second_error) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "TF ray Error(target=%s, source=%s): %s; latest fallback: %s",
                    target_frame_.c_str(), camera_frame_.c_str(),
                    first_error.what(), second_error.what());
                return false;
            }
        }

        if (!ray_origin_world.allFinite() ||
            !ray_direction_world.allFinite() ||
            ray_direction_world.norm() < 1e-6) {
            return false;
        }
        ray_direction_world.normalize();
        return true;
    }

    void updateObservationPlane(const Eigen::Vector3d& p_world, Eigen::Vector3d normal_world)
    {
        if (!normal_world.allFinite() || normal_world.norm() < 1e-6) {
            return;
        }
        normal_world.normalize();

        if (!has_observation_plane_) {
            observation_plane_normal_ = normal_world;
            observation_plane_offset_ = observation_plane_normal_.dot(p_world);
            has_observation_plane_ = true;
            return;
        }

        if (normal_world.dot(observation_plane_normal_) < 0.0) {
            normal_world = -normal_world;
        }

        const double normal_alpha = std::clamp(plane_normal_alpha_, 0.0, 1.0);
        const double offset_alpha = std::clamp(plane_offset_alpha_, 0.0, 1.0);
        observation_plane_normal_ =
            ((1.0 - normal_alpha) * observation_plane_normal_ + normal_alpha * normal_world)
                .normalized();
        const double measured_offset = observation_plane_normal_.dot(p_world);
        observation_plane_offset_ =
            (1.0 - offset_alpha) * observation_plane_offset_ + offset_alpha * measured_offset;
    }

    bool constrainPointToObservationPlane(
        const cv::Point2f& image_center,
        const builtin_interfaces::msg::Time& stamp,
        Eigen::Vector3d& p_world)
    {
        if (!use_plane_constrained_points_ || !has_observation_plane_) {
            return false;
        }

        Eigen::Vector3d ray_origin_world;
        Eigen::Vector3d ray_direction_world;
        if (!cameraRayToWorld(image_center, stamp, ray_origin_world, ray_direction_world)) {
            return false;
        }

        const double denom = observation_plane_normal_.dot(ray_direction_world);
        if (std::abs(denom) < 1e-6) {
            return false;
        }

        const double ray_t =
            (observation_plane_offset_ - observation_plane_normal_.dot(ray_origin_world)) / denom;
        if (!std::isfinite(ray_t) || ray_t <= 0.0) {
            return false;
        }

        p_world = ray_origin_world + ray_t * ray_direction_world;
        return p_world.allFinite();
    }

    void tryUpdateCircleModel(double now)
    {
        if (point_buffer_.size() < static_cast<std::size_t>(geometry_min_samples_) ||
            point_buffer_.span() < geometry_min_span_sec_) {
            return;
        }

        buff_estimator::CircleModel fitted;
        if (!buff_estimator::WorldCircleFitter::fit(
                point_buffer_.samples(),
                circle_model_,
                pnp_normal_weight_,
                known_circle_radius_m_,
                fitted)) {
            return;
        }

        if (fitted.plane_rms > max_plane_rms_m_ ||
            fitted.circle_rms > max_circle_rms_m_ ||
            fitted.angle_span < min_fit_angle_span_rad_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Reject circle fit: plane_rms=%.3f circle_rms=%.3f angle_span=%.3f samples=%d",
                fitted.plane_rms, fitted.circle_rms, fitted.angle_span, fitted.sample_count);
            return;
        }

        fitted.stamp = now;
        circle_model_ = fitted;
    }

    bool updateAngularState(double now, const Eigen::Vector3d& p_world, int8_t mode)
    {
        if (!circle_model_.valid) {
            return false;
        }

        double theta = buff_estimator::WorldCircleFitter::angleOf(circle_model_, p_world);
        bool blade_jump = false;
        if (has_last_angle_) {
            theta = buff_estimator::unwrapAngle(theta, last_theta_);

            double best_theta = theta;
            double best_abs_diff = std::abs(theta - last_theta_);
            for (int k = -5; k <= 5; ++k) {
                const double candidate = theta + static_cast<double>(k) * kBladeInterval;
                const double abs_diff = std::abs(candidate - last_theta_);
                if (abs_diff < best_abs_diff) {
                    best_abs_diff = abs_diff;
                    best_theta = candidate;
                    blade_jump = k != 0;
                }
            }
            theta = best_theta;

            const double dt = now - last_angle_time_;
            if (dt > 1e-4) {
                const double omega = (theta - last_theta_) / dt;
                if (!blade_jump && std::abs(omega) < max_velocity_sample_abs_) {
                    last_velocity_ = omega;
                    const float filtered_abs_velocity =
                        velocity_median_filter_->update(static_cast<float>(std::abs(omega)));

                    if (std::abs(omega) > 1e-3) {
                        world_spin_direction_ = omega > 0.0 ? 1 : -1;
                    }

                    if (mode == 1) {
                        if (!has_fit_start_time_) {
                            fit_start_time_ = now;
                            has_fit_start_time_ = true;
                        }
                        const double rel_time = now - fit_start_time_;
                        predictor_->time_w_pairs_.push_back({rel_time, filtered_abs_velocity});
                        predictor_->trim_before(std::max(0.0, rel_time - velocity_sample_window_sec_));
                        predictor_->fit_velocity_curve();
                    }
                }
            }
        } else {
            has_last_angle_ = true;
            if (!has_fit_start_time_) {
                fit_start_time_ = now;
                has_fit_start_time_ = true;
            }
        }

        last_theta_ = theta;
        last_angle_time_ = now;
        last_blade_jump_ = blade_jump;
        return true;
    }

    bool acceptContinuousObservation(double now, const Eigen::Vector3d& p_world)
    {
        if (!continuity_gate_enabled_) {
            last_accepted_world_point_ = p_world;
            last_accepted_observation_time_ = now;
            has_last_accepted_observation_ = true;
            return true;
        }

        if (!has_last_accepted_observation_ ||
            now - last_accepted_observation_time_ > continuity_reset_sec_) {
            last_accepted_world_point_ = p_world;
            last_accepted_observation_time_ = now;
            has_last_accepted_observation_ = true;
            return true;
        }

        const double jump = (p_world - last_accepted_world_point_).norm();
        if (jump > max_observation_jump_m_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Reject discontinuous YOLO observation: jump=%.3f m limit=%.3f m",
                jump, max_observation_jump_m_);
            return false;
        }

        last_accepted_world_point_ = p_world;
        last_accepted_observation_time_ = now;
        return true;
    }

    double integrateAbsAngularSpeed(double from_time, double to_time) const
    {
        const double dt = std::max(0.0, to_time - from_time);
        if (dt <= 0.0) {
            return 0.0;
        }

        if (last_mode_ == 1) {
            if (predictor_->is_completed()) {
                const Eigen::Vector4f params = predictor_->get_velocity_fit_params();
                const double a = params[0];
                const double omega = params[1];
                const double phi = params[2];
                const double b = params[3];
                if (std::abs(a) > 1e-6 && std::abs(omega) > 1e-6) {
                    const double t0 = from_time - fit_start_time_;
                    const double t1 = to_time - fit_start_time_;
                    return b * dt - (a / omega) *
                        (std::cos(omega * t1 + phi) - std::cos(omega * t0 + phi));
                }
            }
            return std::abs(last_velocity_) * dt;
        }

        if (last_mode_ == -1) {
            return kSmallBuffAngularSpeed * dt;
        }

        return 0.0;
    }

    double predictedAngleAt(double stamp_sec) const
    {
        if (!has_last_angle_) {
            return 0.0;
        }

        int8_t direction = world_spin_direction_;
        if (direction == 0 && std::abs(last_velocity_) > 1e-3) {
            direction = last_velocity_ > 0.0 ? 1 : -1;
        }
        if (direction == 0) {
            return last_theta_;
        }

        return last_theta_ +
            static_cast<double>(direction) * integrateAbsAngularSpeed(last_angle_time_, stamp_sec);
    }

    void rememberObservation(
        double now,
        const Eigen::Vector3d& p_world,
        const Eigen::Vector3d& p_camera,
        double confidence,
        int yolo_target_count)
    {
        has_last_yolo_observation_ = true;
        last_yolo_observation_time_ = now;
        last_observed_world_point_ = p_world;
        last_observed_camera_point_ = p_camera;
        last_observation_confidence_ = confidence;
        last_yolo_target_count_ = yolo_target_count;
        last_observed_depth_ = p_camera.z();
    }

    buff_interfaces::msg::BuffWorldModel makeWorldModelMessage(
        const rclcpp::Time& stamp,
        int8_t mode_hint,
        bool has_observation,
        const Eigen::Vector3d& observed_world,
        const Eigen::Vector3d& observed_camera,
        double confidence,
        int yolo_target_count)
    {
        buff_interfaces::msg::BuffWorldModel model_msg;
        model_msg.header.stamp = stamp;
        model_msg.header.frame_id = target_frame_;
        model_msg.mode = mode_hint != 0 ? mode_hint : last_mode_;
        model_msg.valid = circle_model_.valid && has_last_angle_;
        model_msg.spin_direction = world_spin_direction_;
        model_msg.has_observation = has_observation;

        const double stamp_sec = stamp.seconds();
        const double observation_age =
            has_last_yolo_observation_ ? std::max(0.0, stamp_sec - last_yolo_observation_time_) : 0.0;
        model_msg.seconds_since_observation = static_cast<float>(observation_age);

        const Eigen::Vector3d observation_world =
            has_observation || !has_last_yolo_observation_
                ? observed_world
                : last_observed_world_point_;
        const Eigen::Vector3d observation_camera =
            has_observation || !has_last_yolo_observation_
                ? observed_camera
                : last_observed_camera_point_;

        const double predicted_angle = model_msg.valid
            ? predictedAngleAt(stamp_sec)
            : (has_last_angle_ ? last_theta_ : 0.0);
        const Eigen::Vector3d fitted_world = model_msg.valid
            ? pointOnCircleAtAngle(circle_model_, predicted_angle)
            : projectPointToCircle(circle_model_, observation_world);

        model_msg.observed_point_world = toPointMsg(observation_world);
        model_msg.fitted_point_world = toPointMsg(fitted_world);
        model_msg.observed_point_camera = toPointMsg(observation_camera);
        model_msg.depth = static_cast<float>(
            has_observation || !has_last_yolo_observation_
                ? observed_camera.z()
                : last_observed_depth_);

        if (circle_model_.valid) {
            model_msg.circle_center_world = toPointMsg(circle_model_.center);
            model_msg.circle_axis_world = toVector3Msg(circle_model_.normal);
            model_msg.circle_radius = static_cast<float>(circle_model_.radius);
            model_msg.plane_rms = static_cast<float>(circle_model_.plane_rms);
            model_msg.circle_rms = static_cast<float>(circle_model_.circle_rms);
            model_msg.angle_span = static_cast<float>(circle_model_.angle_span);
        }

        model_msg.current_angle = static_cast<float>(predicted_angle);
        model_msg.angular_velocity = static_cast<float>(last_velocity_);
        model_msg.observation_confidence = static_cast<float>(
            has_observation ? confidence : last_observation_confidence_);
        model_msg.yolo_target_count =
            has_observation ? yolo_target_count : last_yolo_target_count_;
        fillVelocityFields(model_msg, model_msg.mode);
        return model_msg;
    }

    void publishPredictedWorldModel(
        const rclcpp::Time& stamp,
        int8_t mode_hint,
        bool publish_invalid_if_unready)
    {
        if (!circle_model_.valid || !has_last_angle_) {
            if (!publish_invalid_if_unready) {
                return;
            }
            auto model_msg = makeWorldModelMessage(
                stamp, mode_hint, false,
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0);
            model_msg.valid = false;
            world_model_pub_->publish(model_msg);
            return;
        }

        const auto model_msg = makeWorldModelMessage(
            stamp, mode_hint, false,
            last_observed_world_point_, last_observed_camera_point_,
            last_observation_confidence_, last_yolo_target_count_);
        world_model_pub_->publish(model_msg);
    }

    void targetCallback(const buff_interfaces::msg::BuffTarget::SharedPtr msg)
    {
        const rclcpp::Time stamp(msg->header.stamp);

        if (!msg->is_tracking || !msg->has_target_keypoints || !has_cam_info_) {
            publishPredictedWorldModel(stamp, msg->is_bigbuff, true);
            return;
        }

        if (last_mode_ != msg->is_bigbuff) {
            resetMotionState();
            last_mode_ = msg->is_bigbuff;
        }
        if (world_spin_direction_ == 0 && msg->spin_direction != 0) {
            world_spin_direction_ = msg->spin_direction;
        }

        Eigen::Vector3d p_camera;
        Eigen::Vector3d normal_camera;
        double reprojection_error = 0.0;
        if (!solvePnpHitPoint(*msg, p_camera, normal_camera, reprojection_error)) {
            publishPredictedWorldModel(stamp, msg->is_bigbuff, true);
            return;
        }
        if (reprojection_error > max_reprojection_error_px_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Reject PnP observation: reprojection_error=%.2f px", reprojection_error);
            publishPredictedWorldModel(stamp, msg->is_bigbuff, true);
            return;
        }

        Eigen::Vector3d p_world;
        if (!cameraToWorld(p_camera, msg->header.stamp, p_world)) {
            publishPredictedWorldModel(stamp, msg->is_bigbuff, true);
            return;
        }

        Eigen::Vector3d normal_world;
        if (!cameraNormalToWorld(normal_camera, msg->header.stamp, normal_world)) {
            publishPredictedWorldModel(stamp, msg->is_bigbuff, true);
            return;
        }

        const Eigen::Vector3d raw_p_world = p_world;
        updateObservationPlane(raw_p_world, normal_world);
        const cv::Point2f image_center = imageCenterFromTarget(*msg);
        const bool plane_constrained =
            constrainPointToObservationPlane(image_center, msg->header.stamp, p_world);

        const double now = rclcpp::Time(msg->header.stamp).seconds();
        if (!acceptContinuousObservation(now, p_world)) {
            writePointDebugRow(
                now, false, plane_constrained ? "discontinuous_plane" : "discontinuous_raw",
                raw_p_world, p_world, normal_world, p_camera, reprojection_error,
                msg->pose_confidence, msg->yolo_target_count);
            publishPredictedWorldModel(stamp, msg->is_bigbuff, true);
            return;
        }

        if (!circle_model_.valid ||
            max_model_residual_m_ <= 0.0 ||
            buff_estimator::WorldCircleFitter::radialResidual(circle_model_, p_world) < max_model_residual_m_) {
            point_buffer_.push(now, p_world, normal_world, msg->pose_confidence);
            writePointDebugRow(
                now, true, plane_constrained ? "used_plane" : "used_raw",
                raw_p_world, p_world, normal_world, p_camera, reprojection_error,
                msg->pose_confidence, msg->yolo_target_count);
        } else {
            writePointDebugRow(
                now, false, plane_constrained ? "model_outlier_plane" : "model_outlier_raw",
                raw_p_world, p_world, normal_world, p_camera, reprojection_error,
                msg->pose_confidence, msg->yolo_target_count);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Skip outlier point for circle model, residual=%.3f",
                buff_estimator::WorldCircleFitter::radialResidual(circle_model_, p_world));
            publishPredictedWorldModel(stamp, msg->is_bigbuff, true);
            return;
        }

        tryUpdateCircleModel(now);
        const bool has_angle = updateAngularState(now, p_world, msg->is_bigbuff);
        if (has_angle) {
            rememberObservation(
                now, p_world, p_camera, msg->pose_confidence, msg->yolo_target_count);
        }

        auto model_msg = makeWorldModelMessage(
            stamp, msg->is_bigbuff, true,
            p_world, p_camera, msg->pose_confidence, msg->yolo_target_count);
        writeDebugRow(now, p_world, model_msg);

        world_model_pub_->publish(model_msg);
    }

    void fillVelocityFields(buff_interfaces::msg::BuffWorldModel& model_msg, int8_t mode)
    {
        if (mode == 1 && predictor_->is_completed()) {
            const Eigen::Vector4f params = predictor_->get_velocity_fit_params();
            model_msg.speed_a = params[0];
            model_msg.speed_omega = params[1];
            model_msg.speed_phi = params[2];
            model_msg.speed_b = params[3];
        } else if (mode == 1) {
            model_msg.speed_a = 0.0f;
            model_msg.speed_omega = 0.0f;
            model_msg.speed_phi = 0.0f;
            model_msg.speed_b = static_cast<float>(std::abs(last_velocity_));
        } else {
            model_msg.speed_a = 0.0f;
            model_msg.speed_omega = 0.0f;
            model_msg.speed_phi = 0.0f;
            model_msg.speed_b = mode == -1 ? static_cast<float>(kSmallBuffAngularSpeed) : 0.0f;
        }

        model_msg.speed_fit_start_time_sec = fit_start_time_;
        model_msg.speed_fit_buffer_duration_sec = predictor_->get_fit_buffer_duration_sec();
        model_msg.speed_fit_sample_count = predictor_->get_fit_data_point_count();
    }

    void resetMotionState()
    {
        predictor_->reset();
        velocity_median_filter_ =
            std::make_unique<MedianFilter>(std::max(1, velocity_median_window_));
        has_last_angle_ = false;
        has_fit_start_time_ = false;
        fit_start_time_ = 0.0;
        last_theta_ = 0.0;
        last_angle_time_ = 0.0;
        last_velocity_ = 0.0;
        world_spin_direction_ = 0;
        last_blade_jump_ = false;
        has_last_accepted_observation_ = false;
        has_last_yolo_observation_ = false;
        last_accepted_observation_time_ = 0.0;
        last_yolo_observation_time_ = 0.0;
        last_accepted_world_point_ = Eigen::Vector3d::Zero();
        last_observed_world_point_ = Eigen::Vector3d::Zero();
        last_observed_camera_point_ = Eigen::Vector3d::Zero();
        last_observation_confidence_ = 0.0;
        last_yolo_target_count_ = 0;
        last_observed_depth_ = 0.0;
        has_observation_plane_ = false;
        observation_plane_normal_ = Eigen::Vector3d::UnitZ();
        observation_plane_offset_ = 0.0;
    }

    void writeDebugRow(
        double now,
        const Eigen::Vector3d& p_world,
        const buff_interfaces::msg::BuffWorldModel& model_msg)
    {
        if (!debug_csv_.is_open()) {
            return;
        }

        debug_csv_ << now << ','
                   << p_world.x() << ','
                   << p_world.y() << ','
                   << p_world.z() << ','
                   << (circle_model_.valid ? 1 : 0) << ','
                   << circle_model_.center.x() << ','
                   << circle_model_.center.y() << ','
                   << circle_model_.center.z() << ','
                   << circle_model_.normal.x() << ','
                   << circle_model_.normal.y() << ','
                   << circle_model_.normal.z() << ','
                   << circle_model_.radius << ','
                   << circle_model_.plane_rms << ','
                   << circle_model_.circle_rms << ','
                   << circle_model_.angle_span << ','
                   << model_msg.current_angle << ','
                   << last_velocity_ << '\n';
    }

    void writePointDebugRow(
        double now,
        bool used_for_fit,
        const std::string& reason,
        const Eigen::Vector3d& raw_p_world,
        const Eigen::Vector3d& p_world,
        const Eigen::Vector3d& normal_world,
        const Eigen::Vector3d& p_camera,
        double reprojection_error,
        double confidence,
        int yolo_target_count)
    {
        if (!point_debug_csv_.is_open()) {
            return;
        }

        point_debug_csv_ << now << ','
                         << (used_for_fit ? 1 : 0) << ','
                         << reason << ','
                         << raw_p_world.x() << ','
                         << raw_p_world.y() << ','
                         << raw_p_world.z() << ','
                         << p_world.x() << ','
                         << p_world.y() << ','
                         << p_world.z() << ','
                         << normal_world.x() << ','
                         << normal_world.y() << ','
                         << normal_world.z() << ','
                         << p_camera.x() << ','
                         << p_camera.y() << ','
                         << p_camera.z() << ','
                         << reprojection_error << ','
                         << confidence << ','
                         << yolo_target_count << ','
                         << point_buffer_.size() << '\n';
    }

    rclcpp::Subscription<buff_interfaces::msg::BuffTarget>::SharedPtr target_sub_;
    rclcpp::Publisher<buff_interfaces::msg::BuffWorldModel>::SharedPtr world_model_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::TimerBase::SharedPtr model_publish_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<Big_Buff_Predictor> predictor_;
    std::unique_ptr<MedianFilter> velocity_median_filter_;
    buff_estimator::PointBuffer point_buffer_;
    buff_estimator::CircleModel circle_model_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    // Same keypoint geometry as the measured blade target, translated so the hit point is
    // the object origin. This keeps solvePnP's tvec at the aiming point and avoids
    // amplifying pose jitter through a 0.7 m offset.
    std::vector<cv::Point3f> object_points_{
        cv::Point3f(0.0f, 0.0f, 127e-3f),
        cv::Point3f(0.0f, 127e-3f, 0.0f),
        cv::Point3f(0.0f, 0.0f, -127e-3f),
        cv::Point3f(0.0f, -127e-3f, 0.0f),
    };
    cv::Point3f hit_point_object_{0.0f, 0.0f, 0.0f};

    std::string target_frame_ = "odom";
    std::string camera_frame_ = "camera_optical_frame";
    bool debug_mode_ = true;
    std::string debug_csv_path_ = "buff_estimator_debug.csv";
    std::string debug_points_csv_path_ = "buff_estimator_points_debug.csv";
    std::ofstream debug_csv_;
    std::ofstream point_debug_csv_;

    bool has_cam_info_ = false;
    double max_reprojection_error_px_ = 12.0;
    double geometry_buffer_sec_ = 5.0;
    int geometry_min_samples_ = 20;
    double geometry_min_span_sec_ = 1.0;
    double max_model_residual_m_ = 0.25;
    double max_plane_rms_m_ = 0.08;
    double max_circle_rms_m_ = 0.10;
    double min_fit_angle_span_rad_ = 0.75;
    double pnp_normal_weight_ = 0.85;
    double known_circle_radius_m_ = 0.75;
    double model_publish_rate_hz_ = 60.0;
    bool use_plane_constrained_points_ = true;
    double plane_normal_alpha_ = 0.08;
    double plane_offset_alpha_ = 0.05;
    bool continuity_gate_enabled_ = true;
    double max_observation_jump_m_ = 0.55;
    double continuity_reset_sec_ = 0.35;

    int velocity_median_window_ = 3;
    double velocity_sample_window_sec_ = 4.0;
    double max_velocity_sample_abs_ = 8.0;
    double fit_window_sec_ = 1.5;
    int fit_min_samples_ = 10;
    double fit_offset_sum_ = 2.090;
    std::array<double, 2> fit_a_range_{0.780, 1.045};
    std::array<double, 2> fit_omega_range_{1.884, 2.000};
    std::array<double, 2> fit_phi_range_{-M_PI, M_PI};

    int8_t last_mode_ = 0;
    int8_t world_spin_direction_ = 0;
    bool has_last_angle_ = false;
    bool has_fit_start_time_ = false;
    bool has_last_accepted_observation_ = false;
    bool has_last_yolo_observation_ = false;
    bool has_observation_plane_ = false;
    bool last_blade_jump_ = false;
    Eigen::Vector3d last_accepted_world_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_observed_world_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_observed_camera_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d observation_plane_normal_ = Eigen::Vector3d::UnitZ();
    double last_observation_confidence_ = 0.0;
    int last_yolo_target_count_ = 0;
    double last_observed_depth_ = 0.0;
    double fit_start_time_ = 0.0;
    double last_theta_ = 0.0;
    double last_angle_time_ = 0.0;
    double last_velocity_ = 0.0;
    double last_accepted_observation_time_ = 0.0;
    double last_yolo_observation_time_ = 0.0;
    double observation_plane_offset_ = 0.0;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffEstimatorNode)

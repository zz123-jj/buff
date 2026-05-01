#include "predictor.hpp"
#include "world_model.hpp"

#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include "buff_interfaces/msg/buff_target.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr double kBladeInterval = 2.0 * M_PI / 5.0;

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

        aiming_pub_ = this->create_publisher<buff_interfaces::msg::BuffAimingData>(
            "/buff/aiming_data", rclcpp::SensorDataQoS());

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (debug_mode_) {
            debug_csv_.open(debug_csv_path_);
            if (debug_csv_.is_open()) {
                debug_csv_
                    << "t,px,py,pz,model_valid,cx,cy,cz,nx,ny,nz,radius,plane_rms,circle_rms,theta,omega\n";
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
    }

private:
    void declareParameters()
    {
        this->declare_parameter<std::string>("target_frame", "odom");
        this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<std::string>("debug_csv_path", "buff_estimator_debug.csv");
        this->declare_parameter<double>("max_reprojection_error_px", 12.0);
        this->declare_parameter<double>("geometry_buffer_sec", 5.0);
        this->declare_parameter<int>("geometry_min_samples", 20);
        this->declare_parameter<double>("geometry_min_span_sec", 1.0);
        this->declare_parameter<double>("max_model_residual_m", 0.25);
        this->declare_parameter<double>("max_plane_rms_m", 0.08);
        this->declare_parameter<double>("max_circle_rms_m", 0.10);
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
        max_reprojection_error_px_ = this->get_parameter("max_reprojection_error_px").as_double();
        geometry_buffer_sec_ = this->get_parameter("geometry_buffer_sec").as_double();
        geometry_min_samples_ = this->get_parameter("geometry_min_samples").as_int();
        geometry_min_span_sec_ = this->get_parameter("geometry_min_span_sec").as_double();
        max_model_residual_m_ = this->get_parameter("max_model_residual_m").as_double();
        max_plane_rms_m_ = this->get_parameter("max_plane_rms_m").as_double();
        max_circle_rms_m_ = this->get_parameter("max_circle_rms_m").as_double();
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
            false, cv::SOLVEPNP_ITERATIVE);
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

        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points_, rvec, tvec, camera_matrix_, dist_coeffs_, projected);
        double error_sum = 0.0;
        for (std::size_t i = 0; i < projected.size(); ++i) {
            error_sum += cv::norm(projected[i] - image_points[i]);
        }
        reprojection_error = error_sum / static_cast<double>(projected.size());

        return std::isfinite(reprojection_error);
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

    void tryUpdateCircleModel(double now)
    {
        if (point_buffer_.size() < static_cast<std::size_t>(geometry_min_samples_) ||
            point_buffer_.span() < geometry_min_span_sec_) {
            return;
        }

        buff_estimator::CircleModel fitted;
        if (!buff_estimator::WorldCircleFitter::fit(point_buffer_.samples(), circle_model_, fitted)) {
            return;
        }

        if (fitted.plane_rms > max_plane_rms_m_ || fitted.circle_rms > max_circle_rms_m_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Reject circle fit: plane_rms=%.3f circle_rms=%.3f samples=%d",
                fitted.plane_rms, fitted.circle_rms, fitted.sample_count);
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

    void targetCallback(const buff_interfaces::msg::BuffTarget::SharedPtr msg)
    {
        buff_interfaces::msg::BuffAimingData aiming_msg;
        aiming_msg.header = msg->header;
        aiming_msg.header.frame_id = target_frame_;
        aiming_msg.is_bigbuff = msg->is_bigbuff;

        if (!msg->is_tracking || !msg->has_target_keypoints || !has_cam_info_) {
            setDefaultAimingMsg(aiming_msg, msg->is_bigbuff);
            aiming_pub_->publish(aiming_msg);
            return;
        }

        if (last_mode_ != msg->is_bigbuff) {
            resetMotionState();
            last_mode_ = msg->is_bigbuff;
        }

        Eigen::Vector3d p_camera;
        double reprojection_error = 0.0;
        if (!solvePnpHitPoint(*msg, p_camera, reprojection_error)) {
            setDefaultAimingMsg(aiming_msg, msg->is_bigbuff);
            aiming_pub_->publish(aiming_msg);
            return;
        }
        if (reprojection_error > max_reprojection_error_px_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Reject PnP observation: reprojection_error=%.2f px", reprojection_error);
            setDefaultAimingMsg(aiming_msg, msg->is_bigbuff);
            aiming_pub_->publish(aiming_msg);
            return;
        }

        Eigen::Vector3d p_world;
        if (!cameraToWorld(p_camera, msg->header.stamp, p_world)) {
            setDefaultAimingMsg(aiming_msg, msg->is_bigbuff);
            aiming_pub_->publish(aiming_msg);
            return;
        }

        const double now = rclcpp::Time(msg->header.stamp).seconds();
        if (!circle_model_.valid ||
            max_model_residual_m_ <= 0.0 ||
            buff_estimator::WorldCircleFitter::radialResidual(circle_model_, p_world) < max_model_residual_m_) {
            point_buffer_.push(now, p_world, msg->pose_confidence);
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Skip outlier point for circle model, residual=%.3f",
                buff_estimator::WorldCircleFitter::radialResidual(circle_model_, p_world));
        }

        tryUpdateCircleModel(now);
        const bool has_angle = updateAngularState(now, p_world, msg->is_bigbuff);

        aiming_msg.is_tracking = circle_model_.valid && has_angle;
        aiming_msg.spin_direction = world_spin_direction_ != 0 ? world_spin_direction_ : msg->spin_direction;
        aiming_msg.r_cam_x_3d = static_cast<float>(p_camera.x());
        aiming_msg.r_cam_y_3d = static_cast<float>(p_camera.y());
        aiming_msg.r_cam_z_3d = static_cast<float>(p_camera.z());
        aiming_msg.depth = static_cast<float>(p_camera.z());
        aiming_msg.target_x_3d = static_cast<float>(p_world.x());
        aiming_msg.target_y_3d = static_cast<float>(p_world.y());
        aiming_msg.target_z_3d = static_cast<float>(p_world.z());
        aiming_msg.filter_radius = circle_model_.valid
            ? static_cast<float>(circle_model_.radius)
            : 0.0f;
        aiming_msg.angle = has_angle ? static_cast<float>(last_theta_) : 0.0f;

        if (circle_model_.valid) {
            aiming_msg.r_x_3d = static_cast<float>(circle_model_.center.x());
            aiming_msg.r_y_3d = static_cast<float>(circle_model_.center.y());
            aiming_msg.r_z_3d = static_cast<float>(circle_model_.center.z());
            aiming_msg.axis_x_3d = static_cast<float>(circle_model_.normal.x());
            aiming_msg.axis_y_3d = static_cast<float>(circle_model_.normal.y());
            aiming_msg.axis_z_3d = static_cast<float>(circle_model_.normal.z());
        }

        fillVelocityFields(aiming_msg, msg->is_bigbuff);
        writeDebugRow(now, p_world, aiming_msg);

        aiming_pub_->publish(aiming_msg);
    }

    void fillVelocityFields(buff_interfaces::msg::BuffAimingData& aiming_msg, int8_t mode)
    {
        if (mode == 1 && predictor_->is_completed()) {
            const Eigen::Vector4f params = predictor_->get_velocity_fit_params();
            aiming_msg.sin_a = params[0];
            aiming_msg.sin_omega = params[1];
            aiming_msg.sin_phi = params[2];
            aiming_msg.sin_b = params[3];
        } else if (mode == 1) {
            aiming_msg.sin_a = 0.0f;
            aiming_msg.sin_omega = 0.0f;
            aiming_msg.sin_phi = 0.0f;
            aiming_msg.sin_b = static_cast<float>(std::abs(last_velocity_));
        } else {
            aiming_msg.sin_a = 0.0f;
            aiming_msg.sin_omega = 0.0f;
            aiming_msg.sin_phi = 0.0f;
            aiming_msg.sin_b = 0.0f;
        }

        aiming_msg.fit_start_time_sec = fit_start_time_;
        aiming_msg.fit_buffer_duration_sec = predictor_->get_fit_buffer_duration_sec();
        aiming_msg.fit_data_point_count = predictor_->get_fit_data_point_count();
    }

    void setDefaultAimingMsg(buff_interfaces::msg::BuffAimingData& aiming_msg, int8_t mode)
    {
        aiming_msg.is_tracking = false;
        aiming_msg.is_bigbuff = mode;
        aiming_msg.spin_direction = world_spin_direction_;
        aiming_msg.r_x_3d = 0.0f;
        aiming_msg.r_y_3d = 0.0f;
        aiming_msg.r_z_3d = 0.0f;
        aiming_msg.r_cam_x_3d = 0.0f;
        aiming_msg.r_cam_y_3d = 0.0f;
        aiming_msg.r_cam_z_3d = 0.0f;
        aiming_msg.axis_x_3d = 0.0f;
        aiming_msg.axis_y_3d = 0.0f;
        aiming_msg.axis_z_3d = 0.0f;
        aiming_msg.depth = 0.0f;
        aiming_msg.target_x_3d = 0.0f;
        aiming_msg.target_y_3d = 0.0f;
        aiming_msg.target_z_3d = 0.0f;
        aiming_msg.sin_a = 0.0f;
        aiming_msg.sin_omega = 0.0f;
        aiming_msg.sin_phi = 0.0f;
        aiming_msg.sin_b = static_cast<float>(std::abs(last_velocity_));
        aiming_msg.fit_start_time_sec = fit_start_time_;
        aiming_msg.fit_buffer_duration_sec = predictor_->get_fit_buffer_duration_sec();
        aiming_msg.fit_data_point_count = predictor_->get_fit_data_point_count();
        aiming_msg.filter_radius = circle_model_.valid ? static_cast<float>(circle_model_.radius) : 0.0f;
        aiming_msg.angle = has_last_angle_ ? static_cast<float>(last_theta_) : 0.0f;
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
    }

    void writeDebugRow(
        double now,
        const Eigen::Vector3d& p_world,
        const buff_interfaces::msg::BuffAimingData& aiming_msg)
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
                   << aiming_msg.angle << ','
                   << last_velocity_ << '\n';
    }

    rclcpp::Subscription<buff_interfaces::msg::BuffTarget>::SharedPtr target_sub_;
    rclcpp::Publisher<buff_interfaces::msg::BuffAimingData>::SharedPtr aiming_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<Big_Buff_Predictor> predictor_;
    std::unique_ptr<MedianFilter> velocity_median_filter_;
    buff_estimator::PointBuffer point_buffer_;
    buff_estimator::CircleModel circle_model_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point3f> object_points_{
        cv::Point3f(0.0f, 0.0f, 827e-3f),
        cv::Point3f(0.0f, 127e-3f, 700e-3f),
        cv::Point3f(0.0f, 0.0f, 573e-3f),
        cv::Point3f(0.0f, -127e-3f, 700e-3f),
    };
    cv::Point3f hit_point_object_{0.0f, 0.0f, 700e-3f};

    std::string target_frame_ = "odom";
    std::string camera_frame_ = "camera_optical_frame";
    bool debug_mode_ = true;
    std::string debug_csv_path_ = "buff_estimator_debug.csv";
    std::ofstream debug_csv_;

    bool has_cam_info_ = false;
    double max_reprojection_error_px_ = 12.0;
    double geometry_buffer_sec_ = 5.0;
    int geometry_min_samples_ = 20;
    double geometry_min_span_sec_ = 1.0;
    double max_model_residual_m_ = 0.25;
    double max_plane_rms_m_ = 0.08;
    double max_circle_rms_m_ = 0.10;

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
    bool last_blade_jump_ = false;
    double fit_start_time_ = 0.0;
    double last_theta_ = 0.0;
    double last_angle_time_ = 0.0;
    double last_velocity_ = 0.0;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffEstimatorNode)

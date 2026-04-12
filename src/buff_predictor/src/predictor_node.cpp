#include "predictor.hpp"
#include "coordinate_solver.hpp"
#include "config.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "buff_interfaces/msg/buff_target.hpp"
#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <string>

class BuffPredictorNode : public rclcpp::Node
{
public:
    BuffPredictorNode() : Node("buff_predictor_node")
    {
        // 半径的滤波
        radius_filter_ = std::make_unique<MovAvg>(15);

        // 算法初始化（使用配置参数）
        predictor_ = std::make_unique<Big_Buff_Predictor>();
        
        // 角度连续化初始化
        angle_observer_ = std::make_unique<angleObserver>(clockMode::unknown);

        // 3D解算初始化（使用配置参数）
        CameraParams cam_params(1303.675283386667f, 1303.675283386667f, 720.0f, 540.0f);
        predictor_3d_ = std::make_unique<Predictor3D>(cam_params, PHYSICAL_ARM_LENGTH);
        // 坐标转换初始化
        CoordinateSolver::CameraIntrinsics coord_params = {1303.675283386667f, 1303.675283386667f, 720.0f, 540.0f};
        coord_solver_ = std::make_unique<CoordinateSolver>(coord_params);

    
        target_sub_ = this->create_subscription<buff_interfaces::msg::BuffTarget>(
            "/buff_target", 10,
            [this](const buff_interfaces::msg::BuffTarget::SharedPtr msg) { targetCallback(msg); });

        aiming_pub_ = this->create_publisher<buff_interfaces::msg::BuffAimingData>(
            "/buff/aiming_data", 10);

        // 接受tf
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        target_frame_ = this->declare_parameter<std::string>("target_frame", "camera_optical_frame");
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");

        if (DEBUG_MODE) {
            tracker_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/tracker/debug_point", 10);
            enable_angle_log_ = this->declare_parameter<bool>("enable_angle_log", true);
            angle_log_path_ = this->declare_parameter<std::string>("angle_log_path", "log/angle_time.csv");
            if (enable_angle_log_) {
                initAngleLogger();
            }
        }
    }

    ~BuffPredictorNode() override
    {
        if (angle_log_file_.is_open()) {
            angle_log_file_.flush();
            angle_log_file_.close();
        }
    }

private:
    void initAngleLogger()
    {
        try {
            const std::filesystem::path log_path(angle_log_path_);
            if (log_path.has_parent_path()) {
                std::filesystem::create_directories(log_path.parent_path());
            }
            angle_log_file_.open(log_path, std::ios::out | std::ios::trunc);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to create angle log file (%s): %s", angle_log_path_.c_str(), e.what());
            enable_angle_log_ = false;
            return;
        }

        if (!angle_log_file_.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open angle log file: %s", angle_log_path_.c_str());
            enable_angle_log_ = false;
            return;
        }

        angle_log_file_ << std::fixed << std::setprecision(9);
        angle_log_file_ << "stamp_sec,time_from_start_s,continues_angle_rad,continues_angle_deg,is_tracking,fit_success,sin_a,sin_omega,sin_phi,sin_b,fit_start_time_sec\n";
        RCLCPP_INFO(this->get_logger(), "Angle logging enabled: %s", angle_log_path_.c_str());
    }

    void logContinuousAngle(
        const rclcpp::Time& stamp,
        float continues_angle,
        bool is_tracking,
        bool fit_success,
        const Eigen::VectorXf& sin_para,
        double fit_start_time_sec)
    {
        if (!enable_angle_log_ || !angle_log_file_.is_open()) {
            return;
        }

        if (angle_log_count_ == 0) {
            angle_log_start_stamp_ = stamp;
        }

        const double stamp_sec = stamp.seconds();
        const double time_from_start_s = (stamp - angle_log_start_stamp_).seconds();
        const double angle_deg = static_cast<double>(continues_angle) * 180.0 / M_PI;
        double sin_a = 0.0;
        double sin_omega = 0.0;
        double sin_phi = 0.0;
        double sin_b = 0.0;

        if (fit_success && sin_para.size() >= 4) {
            sin_a = static_cast<double>(sin_para[0]);
            sin_omega = static_cast<double>(sin_para[1]);
            sin_phi = static_cast<double>(sin_para[2]);
            sin_b = static_cast<double>(sin_para[3]);
        }

        angle_log_file_ << stamp_sec << ","
                        << time_from_start_s << ","
                        << continues_angle << ","
                        << angle_deg << ","
                        << (is_tracking ? 1 : 0) << ","
                        << (fit_success ? 1 : 0) << ","
                        << sin_a << ","
                        << sin_omega << ","
                        << sin_phi << ","
                        << sin_b << ","
                        << fit_start_time_sec
                        << "\n";

        ++angle_log_count_;
        if ((angle_log_count_ % 50) == 0) {
            angle_log_file_.flush();
        }
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

        try
        {
            auto tf_stamped = tf_buffer_->lookupTransform(
                target_frame_,
                input.header.frame_id,
                stamp,
                tf2::durationFromSec(0.05)
            );
            tf2::doTransform(input, output, tf_stamped);
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "TF Error(target=%s, source=%s): %s",
                target_frame_.c_str(),
                input.header.frame_id.c_str(),
                ex.what()
            );
            return false;
        }
    }

    void targetCallback(const buff_interfaces::msg::BuffTarget::SharedPtr msg)
    {
        // 计算向量：扇叶框中心 - R框中心
        float vector_x = msg->target_center_x - msg->r_center_x;
        float vector_y = msg->target_center_y - msg->r_center_y;
        
        // 计算像素半径 hypot()返回传递的参数平方和的平方根
        // double pixel_arm_length = std::hypot(vector_x, vector_y);

        double pixel_arm_length = msg->radius;

        // // 半径滤波
        // pixel_arm_length = radius_filter_->update(pixel_arm_length);

        if (pixel_arm_length < 1.0) pixel_arm_length = 1.0;

        if (first_frame_)
        {
            first_frame_ = false;
        }

        // 角度连续化
        float continues_angle = angle_observer_->update(vector_x, vector_y, pixel_arm_length);
        frame_count_++;
        const auto stamp = rclcpp::Time(msg->header.stamp);

        // Reset log latch when tracking is lost so next segment can log once again.
        if (!msg->is_tracking) {
            params_published_ = false;
        }

        //更新预测器
        auto result = predictor_->update(continues_angle, stamp.seconds(), msg->is_tracking);
        bool success = result.first;
        const auto fit_sin_para = predictor_->get_sin_para();
        const double fit_start_time_sec = success ? predictor_->get_fit_start_time_sec() : 0.0;

        if (DEBUG_MODE)
        {
            logContinuousAngle(
                stamp,
                continues_angle,
                msg->is_tracking,
                success,
                fit_sin_para,
                fit_start_time_sec);
        }

        // Print fitted sine parameters once per tracking segment when fitting succeeds.
        if (success && !params_published_) {
            if (fit_sin_para.size() >= 4) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Fit success: dy/dt = %.6f * sin(%.6f * t + %.6f) + %.6f",
                    static_cast<double>(fit_sin_para[0]),
                    static_cast<double>(fit_sin_para[1]),
                    static_cast<double>(fit_sin_para[2]),
                    static_cast<double>(fit_sin_para[3])
                );
            }
            params_published_ = true;
        }

        geometry_msgs::msg::PointStamped point_odom;
        geometry_msgs::msg::PointStamped target_odom;

        // TF 变换与坐标解算
        // 计算深度
        float depth = predictor_3d_->compute_depth(pixel_arm_length);

        // R标坐标
        auto p_cam_pt = coord_solver_->pixelToCamera(msg->r_center_x, msg->r_center_y, depth);

        geometry_msgs::msg::PointStamped point_cam;
        point_cam.header.stamp = msg->header.stamp; // 对齐时间戳
        point_cam.header.frame_id = camera_frame_;
        point_cam.point.x = p_cam_pt.x;
        point_cam.point.y = p_cam_pt.y;
        point_cam.point.z = p_cam_pt.z;

        if (!transformPointToTarget(point_cam, msg->header.stamp, point_odom)) {
            return;
        }

        // Debug Point（仅在debug模式下发布）
        if (DEBUG_MODE)
        {
            tracker_pub_->publish(point_odom);
        }

        // target point
        auto target_point_pt = coord_solver_->pixelToCamera(
            msg->target_center_x, msg->target_center_y, depth);

        geometry_msgs::msg::PointStamped target_point_cam;
        target_point_cam.header.stamp = msg->header.stamp;
        target_point_cam.header.frame_id = camera_frame_;
        target_point_cam.point.x = target_point_pt.x;
        target_point_cam.point.y = target_point_pt.y;
        target_point_cam.point.z = target_point_pt.z;

        if (!transformPointToTarget(target_point_cam, msg->header.stamp, target_odom)) {
            return;
        }

        // 发布调试点 (仅在 DEBUG_MODE)
        if (DEBUG_MODE) {
            auto debug_point_msg = geometry_msgs::msg::PointStamped();
            debug_point_msg.header.stamp = msg->header.stamp;
            debug_point_msg.header.frame_id = target_frame_;
            debug_point_msg.point = target_odom.point;
            tracker_pub_->publish(debug_point_msg);
        }

        // buffAimingData 发布（仅在TF成功时有3D坐标）
        auto aiming_msg = buff_interfaces::msg::BuffAimingData();
        aiming_msg.header.stamp = this->now();
        aiming_msg.header.frame_id = target_frame_;
        aiming_msg.frame_number = frame_count_;
        aiming_msg.is_tracking = success;
        aiming_msg.r_center_x_3d = point_odom.point.x;
        aiming_msg.r_center_y_3d = point_odom.point.y;
        aiming_msg.r_center_z_3d = point_odom.point.z;
        aiming_msg.pixel_r_center_x = msg->r_center_x;
        aiming_msg.pixel_r_center_y = msg->r_center_y;
        aiming_msg.pixel_radius = pixel_arm_length;
        if (success) {
            if (fit_sin_para.size() >= 4) {
                aiming_msg.sin_a = fit_sin_para[0];
                aiming_msg.sin_omega = fit_sin_para[1];
                aiming_msg.sin_phi = fit_sin_para[2];
                aiming_msg.sin_b = fit_sin_para[3];
            }
            aiming_msg.fit_start_time_sec = fit_start_time_sec;
            aiming_msg.fit_buffer_duration_sec = predictor_->get_fit_buffer_duration_sec();
            aiming_msg.fit_data_point_count = predictor_->get_fit_data_point_count();
        } else {
            aiming_msg.fit_start_time_sec = 0.0;
            aiming_msg.fit_buffer_duration_sec = 0.0f;
            aiming_msg.fit_data_point_count = 0;
        }
        aiming_msg.target_x_3d = target_odom.point.x;
        aiming_msg.target_y_3d = target_odom.point.y;
        aiming_msg.target_z_3d = target_odom.point.z;
        aiming_pub_->publish(aiming_msg);
    }

    // ROS2 Components
    rclcpp::Subscription<buff_interfaces::msg::BuffTarget>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tracker_pub_;
    rclcpp::Publisher<buff_interfaces::msg::BuffAimingData>::SharedPtr aiming_pub_;

    // Algo Modules
    std::unique_ptr<Big_Buff_Predictor> predictor_;
    std::unique_ptr<angleObserver> angle_observer_;
    std::unique_ptr<Predictor3D> predictor_3d_;
    std::unique_ptr<CoordinateSolver> coord_solver_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<MovAvg> radius_filter_;

    // States
    float radius_pixel_ = 0.0;
    bool first_frame_ = true;
    bool params_published_ = false;
    int frame_count_ = 0;

    // TF frame settings
    std::string target_frame_;
    std::string camera_frame_;

    // Debug angle logging
    bool enable_angle_log_ = false;
    std::string angle_log_path_ = "log/angle_time.csv";
    std::ofstream angle_log_file_;
    rclcpp::Time angle_log_start_stamp_{0, 0, RCL_ROS_TIME};
    std::size_t angle_log_count_ = 0;

    // Joint
    double current_pitch_ = 0.0;
    double current_yaw_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuffPredictorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

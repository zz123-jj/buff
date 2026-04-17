#include "predictor.hpp"
#include "coordinate_solver.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "buff_interfaces/msg/buff_target.hpp"
#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <string>

class BuffPredictorNode : public rclcpp::Node
{
public:
    BuffPredictorNode() : Node("buff_predictor_node")
    {
        predictor_ = std::make_unique<Big_Buff_Predictor>();
        
        // 角度连续化初始化
        angle_observer_ = std::make_unique<angleObserver>(clockMode::unknown);

        this->declare_parameter<std::string>("target_frame", "odom");
        this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<double>("physical_arm_length", 1.5);
        this->declare_parameter<std::string>("debug_csv_path", "buff_predictor_debug.csv");
        this->declare_parameter<int>("velocity_median_window", 3);
        this->declare_parameter<double>("camera_fx", 1303.675283386667);
        this->declare_parameter<double>("camera_fy", 1303.675283386667);
        this->declare_parameter<double>("camera_cx", 720.0);
        this->declare_parameter<double>("camera_cy", 540.0);
        this->declare_parameter<double>("fit_window_sec", 1.5);
        this->declare_parameter<int>("fit_min_samples", 10);
        this->declare_parameter<double>("fit_offset_sum", 2.090);
        this->declare_parameter<double>("fit_a_lower", 0.780);
        this->declare_parameter<double>("fit_a_upper", 1.045);
        this->declare_parameter<double>("fit_omega_lower", 1.884);
        this->declare_parameter<double>("fit_omega_upper", 2.000);
        this->declare_parameter<double>("fit_phi_lower", -M_PI);
        this->declare_parameter<double>("fit_phi_upper", M_PI);
        target_frame_ = this->get_parameter("target_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
        physical_arm_length_ = this->get_parameter("physical_arm_length").as_double();
        debug_csv_path_ = this->get_parameter("debug_csv_path").as_string();
        velocity_median_window_ = this->get_parameter("velocity_median_window").as_int();
        camera_fx_ = this->get_parameter("camera_fx").as_double();
        camera_fy_ = this->get_parameter("camera_fy").as_double();
        camera_cx_ = this->get_parameter("camera_cx").as_double();
        camera_cy_ = this->get_parameter("camera_cy").as_double();
        fit_window_sec_ = this->get_parameter("fit_window_sec").as_double();
        fit_min_samples_ = this->get_parameter("fit_min_samples").as_int();
        fit_offset_sum_ = this->get_parameter("fit_offset_sum").as_double();
        fit_a_lower_ = this->get_parameter("fit_a_lower").as_double();
        fit_a_upper_ = this->get_parameter("fit_a_upper").as_double();
        fit_omega_lower_ = this->get_parameter("fit_omega_lower").as_double();
        fit_omega_upper_ = this->get_parameter("fit_omega_upper").as_double();
        fit_phi_lower_ = this->get_parameter("fit_phi_lower").as_double();
        fit_phi_upper_ = this->get_parameter("fit_phi_upper").as_double();

        velocity_median_filter_ =
            std::make_unique<MedianFilter>(std::max(1, velocity_median_window_));
        predictor_->set_debug_mode(debug_mode_);
        predictor_->set_fit_config(
            fit_offset_sum_,
            fit_window_sec_,
            fit_min_samples_,
            fit_a_lower_,
            fit_a_upper_,
            fit_omega_lower_,
            fit_omega_upper_,
            fit_phi_lower_,
            fit_phi_upper_);

        // 坐标转换初始化
        CoordinateSolver::CameraIntrinsics coord_params = {camera_fx_, camera_fy_, camera_cx_, camera_cy_};
        coord_solver_ = std::make_unique<CoordinateSolver>(coord_params, physical_arm_length_);

    
        target_sub_ = this->create_subscription<buff_interfaces::msg::BuffTarget>(
            "/buff/target", 10,
            [this](const buff_interfaces::msg::BuffTarget::SharedPtr msg) { targetCallback(msg); });

        aiming_pub_ = this->create_publisher<buff_interfaces::msg::BuffAimingData>(
            "/buff/aiming_data", 10);

        // 接受tf
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        //debug mode记录csv
        if (debug_mode_) {
        debug_csv_.open(debug_csv_path_);

        }

    }
    ~BuffPredictorNode()
    {
        if (debug_csv_.is_open()) {
            debug_csv_.close();
        }
    }

  

private:

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
        buff_interfaces::msg::BuffAimingData aiming_msg;
        aiming_msg.header = msg->header;
        aiming_msg.header.frame_id = target_frame_;
        aiming_msg.spin_direction = msg->spin_direction;
        aiming_msg.is_bigbuff = msg->is_bigbuff;
        aiming_msg.is_tracking = msg->is_tracking;

        bool is_tracking = msg->is_tracking;
        const bool is_big_buff = msg->is_bigbuff? true : false;

        //不在跟踪时重置状态并发布默认数据
        if (!is_tracking) {
            setParaDefaultAimingMsg(aiming_msg);
            predictor_->reset();
            angle_observer_->reset();
            velocity_median_filter_ =
            std::make_unique<MedianFilter>(std::max(1, velocity_median_window_));
            isFirst_frame_ = true;
            last_time_since_start_ = 0.0;
            time_since_start_ = 0.0;
            last_angle_ = 0.0f;
            aiming_pub_->publish(aiming_msg);
            return;
        }

        if (is_big_buff) {
            clockMode direction_mode = clockMode::unknown;
            if (msg->spin_direction == -1) {
                direction_mode = clockMode::anticlockwise;
            } else if (msg->spin_direction == 1) {
                direction_mode = clockMode::clockwise;
            }
            angle_observer_->setClockMode(direction_mode);

            // 计算向量：扇叶框中心 - R框中心
            float vector_x = msg->target_center_x - msg->r_center_x;
            float vector_y = msg->target_center_y - msg->r_center_y;
            double pixel_arm_length = msg->radius;

            // 角度连续化
            const float continues_angle = angle_observer_->update(vector_x, vector_y, pixel_arm_length);
            const auto now = rclcpp::Time(msg->header.stamp).seconds();

            // 记录第一帧时间戳
            if (isFirst_frame_) {
                fit_start_time_ = now;
                isFirst_frame_ = false;
            }

            time_since_start_ = now - fit_start_time_; // 第一帧这个值为0
            if (!predictor_->has_fit_attempted() && time_since_start_ > 0 && time_since_start_ < fit_window_sec_) {
                double median_time = (last_time_since_start_ + time_since_start_) / 2.0;
                float mean_angleVelocity =
                    (continues_angle - last_angle_) / (time_since_start_ - last_time_since_start_);
                float filtered_angle_velocity = velocity_median_filter_->update(mean_angleVelocity);
                const std::pair<double, float> time_w_pair(median_time, std::abs(filtered_angle_velocity));

                if (debug_mode_) {
                    debug_csv_ << median_time << "," << std::abs(filtered_angle_velocity) << std::endl;
                }

                predictor_->time_w_pairs_.push_back(time_w_pair);
            }
            if (!predictor_->is_completed() && time_since_start_ >= fit_window_sec_) {
                predictor_->try_fit_once_at_1p5s();
            }

            if (predictor_->is_completed()) {
                const Vector4f params = predictor_->get_velocity_fit_params();
                aiming_msg.sin_a = params[0];
                aiming_msg.sin_omega = params[1];
                aiming_msg.sin_phi = params[2];
                aiming_msg.sin_b = params[3];
            } else {
                aiming_msg.sin_a = 0.0f;
                aiming_msg.sin_omega = 0.0f;
                aiming_msg.sin_phi = 0.0f;
                aiming_msg.sin_b = 0.0f;
            }
            aiming_msg.fit_start_time_sec = fit_start_time_;
            aiming_msg.fit_buffer_duration_sec = predictor_->get_fit_buffer_duration_sec();
            aiming_msg.fit_data_point_count = predictor_->get_fit_data_point_count();

            last_time_since_start_ = time_since_start_;
            last_angle_ = angle_observer_->last_angle_;
        
            //小符仅做坐标转换，不进行拟合
        } else {
            predictor_->reset();
            angle_observer_->reset();
            velocity_median_filter_ =
             std::make_unique<MedianFilter>(std::max(1, velocity_median_window_));
            isFirst_frame_ = true;
            last_time_since_start_ = 0.0;
            time_since_start_ = 0.0;
            last_angle_ = 0.0f;
            setParaDefaultAimingMsg(aiming_msg);
        }

        const double depth = coord_solver_->computeDepthFromRadius(msg->radius);
        if (depth > 0.0) {
            geometry_msgs::msg::PointStamped r_cam;
            r_cam.header.stamp = msg->header.stamp;
            r_cam.header.frame_id = camera_frame_;
            r_cam.point = coord_solver_->pixelToCamera(msg->r_center_x, msg->r_center_y, depth);

            geometry_msgs::msg::PointStamped target_cam;
            target_cam.header.stamp = msg->header.stamp;
            target_cam.header.frame_id = camera_frame_;
            target_cam.point =
                coord_solver_->pixelToCamera(msg->target_center_x, msg->target_center_y, depth);

            geometry_msgs::msg::PointStamped target_odom;
            const bool target_ok = transformPointToTarget(target_cam, msg->header.stamp, target_odom);
            if (target_ok) {
                aiming_msg.target_x_3d = static_cast<float>(target_odom.point.x);
                aiming_msg.target_y_3d = static_cast<float>(target_odom.point.y);
                aiming_msg.target_z_3d = static_cast<float>(target_odom.point.z);
            }
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Invalid radius for depth solve: %.4f",
                msg->radius);
        }
        aiming_pub_->publish(aiming_msg);
    }

    void setDefaultAimingMsg(buff_interfaces::msg::BuffAimingData& aiming_msg)
    {
        aiming_msg.is_bigbuff = 0;
        aiming_msg.spin_direction = 0;
        aiming_msg.target_x_3d = 0.0f;
        aiming_msg.target_y_3d = 0.0f;
        aiming_msg.target_z_3d = 0.0f;
        aiming_msg.sin_a = 0.0f;
        aiming_msg.sin_omega = 0.0f;
        aiming_msg.sin_phi = 0.0f;
        aiming_msg.sin_b = 0.0f;
        aiming_msg.fit_start_time_sec = 0.0f;
        aiming_msg.fit_buffer_duration_sec = 0.0f; 
        aiming_msg.fit_data_point_count = 0.0f;
    }

    void setParaDefaultAimingMsg(buff_interfaces::msg::BuffAimingData& aiming_msg)
    {
        aiming_msg.sin_a = 0.0f;
        aiming_msg.sin_omega = 0.0f;
        aiming_msg.sin_phi = 0.0f;
        aiming_msg.sin_b = 0.0f;
        aiming_msg.fit_start_time_sec = 0.0f;
        aiming_msg.fit_buffer_duration_sec = 0.0f;
        aiming_msg.fit_data_point_count = 0.0f;
    }

    rclcpp::Subscription<buff_interfaces::msg::BuffTarget>::SharedPtr target_sub_;
    rclcpp::Publisher<buff_interfaces::msg::BuffAimingData>::SharedPtr aiming_pub_;

    // Algo Modules
    std::unique_ptr<Big_Buff_Predictor> predictor_;
    std::unique_ptr<angleObserver> angle_observer_;
    std::unique_ptr<CoordinateSolver> coord_solver_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<MedianFilter> velocity_median_filter_;

    // TF frame settings
    std::string target_frame_;
    std::string camera_frame_;
    bool debug_mode_ = true;
    double physical_arm_length_ = 1.5;
    std::string debug_csv_path_ = "buff_predictor_debug.csv";
    int velocity_median_window_ = 3;
    double camera_fx_ = 1303.675283386667;
    double camera_fy_ = 1303.675283386667;
    double camera_cx_ = 720.0;
    double camera_cy_ = 540.0;
    double fit_window_sec_ = 1.5;
    int fit_min_samples_ = 10;
    double fit_offset_sum_ = 2.090;
    double fit_a_lower_ = 0.780;
    double fit_a_upper_ = 1.045;
    double fit_omega_lower_ = 1.884;
    double fit_omega_upper_ = 2.000;
    double fit_phi_lower_ = -M_PI;
    double fit_phi_upper_ = M_PI;

    std::ofstream debug_csv_;
    bool isFirst_frame_ = true;
    double last_time_since_start_ = 0.0f;
    float last_angle_ = 0.0f;
    double fit_start_time_ = 0.0f;
    double time_since_start_ = 0.0f;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuffPredictorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

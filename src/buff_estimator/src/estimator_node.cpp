#include "predictor.hpp"
#include "coordinate_solver.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "buff_interfaces/msg/buff_target.hpp"
#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <string>

class BuffEstimatorNode : public rclcpp::Node
{
public:
    explicit BuffEstimatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("buff_estimator_node", options)
    {
        
        predictor_ = std::make_unique<Big_Buff_Predictor>();
        angle_observer_ = std::make_unique<angleObserver>(clockMode::unknown);

        this->declare_parameter<std::string>("target_frame", "odom");
        this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<double>("physical_arm_length", 0.75);
        this->declare_parameter<std::string>("debug_csv_path", "buff_estimator_debug.csv");
        this->declare_parameter<int>("velocity_median_window", 3);
        this->declare_parameter<int>("radius_mean_window", 15);
        this->declare_parameter<double>("fit_window_sec", 1.5);
        this->declare_parameter<int>("fit_min_samples", 10);
        this->declare_parameter<double>("fit_offset_sum", 2.090);
        this->declare_parameter<std::vector<double>>("fit_a_range", {0.780, 1.045});
        this->declare_parameter<std::vector<double>>("fit_omega_range", {1.884, 2.000});
        this->declare_parameter<std::vector<double>>("fit_phi_range", {-M_PI, M_PI});
        target_frame_ = this->get_parameter("target_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
        physical_arm_length_     = this->get_parameter("physical_arm_length").as_double();
        debug_csv_path_ = this->get_parameter("debug_csv_path").as_string();
        velocity_median_window_ = this->get_parameter("velocity_median_window").as_int();
        radius_mean_window_ = this->get_parameter("radius_mean_window").as_int();
        auto fit_window_sec = this->get_parameter("fit_window_sec").as_double();
        auto fit_min_samples = this->get_parameter("fit_min_samples").as_int();
        auto fit_offset_sum = this->get_parameter("fit_offset_sum").as_double();
        auto fit_a_range = this->get_parameter("fit_a_range").as_double_array();
        auto fit_omega_range = this->get_parameter("fit_omega_range").as_double_array();
        auto fit_phi_range = this->get_parameter("fit_phi_range").as_double_array();

        velocity_median_filter_ =
            std::make_unique<MedianFilter>(std::max(1, velocity_median_window_));
        radius_mean_filter_ =
            std::make_unique<MovAvg>(std::max(1, radius_mean_window_));
        // 配置预测器拟合参数
        predictor_->set_fit_config(
            fit_offset_sum,
            fit_window_sec,
            fit_min_samples,
            fit_a_range[0],
            fit_a_range[1],
            fit_omega_range[0],
            fit_omega_range[1],
            fit_phi_range[0],
            fit_phi_range[1]);
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            // 从 CameraInfo 的 K 矩阵中提取内参：fx = K[0], fy = K[4], cx = K[2], cy = K[5]
            CoordinateSolver::CameraIntrinsics cam_intri = {
                msg->k[0], msg->k[4], msg->k[2], msg->k[5]
            };
            
            // 重新初始化坐标解算器
            coord_solver_ = std::make_unique<CoordinateSolver>(cam_intri, physical_arm_length_);
            has_cam_info_ = true;
            
            // 拿到内参后可以取消订阅以节省开销（如果内参不会改变）
             cam_info_sub_.reset(); 
        });
    
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
    ~BuffEstimatorNode()
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
                std::chrono::milliseconds(10)
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
        if (!has_cam_info_ ) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for CameraInfo to initialize coordinate solver...");
            return;
        }
        buff_interfaces::msg::BuffAimingData aiming_msg;
        aiming_msg.header = msg->header;
        
        aiming_msg.header.frame_id = target_frame_;
        bool is_tracking = msg->is_tracking;
        aiming_msg.is_tracking = is_tracking;

        if (!is_tracking) {
            setDefaultAimingMsg(aiming_msg);
            predictor_->reset();
            angle_observer_->reset();
            velocity_median_filter_ =
            std::make_unique<MedianFilter>(std::max(1, velocity_median_window_));
            radius_mean_filter_ =
            std::make_unique<MovAvg>(std::max(1, radius_mean_window_));
            isFirst_frame_ = true;
            last_time_since_start_ = 0.0;
            time_since_start_ = 0.0;
            last_angle_ = 0.0f;
            aiming_pub_->publish(std::move(aiming_msg));
            return;
        }
        
        aiming_msg.is_bigbuff = msg->is_bigbuff;
        aiming_msg.spin_direction = msg->spin_direction;

        const float filtered_radius = radius_mean_filter_->update(msg->radius);
        aiming_msg.filter_radius = filtered_radius;
        // 计算深度并转换坐标
        const double depth = coord_solver_->computeDepthFromRadius(filtered_radius);
        if (depth > 0.0) {
            geometry_msgs::msg::PointStamped r_cam;
            r_cam.header.stamp = msg->header.stamp;
            r_cam.header.frame_id = camera_frame_;
            r_cam.point = coord_solver_->pixelToCamera(msg->r_center_x, msg->r_center_y, depth);
            //fuck2
            aiming_msg.r_cam_x_3d = static_cast<float>(r_cam.point.x);
            aiming_msg.r_cam_y_3d = static_cast<float>(r_cam.point.y);
            aiming_msg.r_cam_z_3d = static_cast<float>(r_cam.point.z);
            aiming_msg.depth = static_cast<float>(depth);
            geometry_msgs::msg::PointStamped target_cam;
            target_cam.header.stamp = msg->header.stamp;
            target_cam.header.frame_id = camera_frame_;
            target_cam.point =
                coord_solver_->pixelToCamera(msg->target_center_x, msg->target_center_y, depth);

            geometry_msgs::msg::PointStamped target_center;
            geometry_msgs::msg::PointStamped r_center;
            geometry_msgs::msg::PointStamped camera_origin_cam;
            camera_origin_cam.header.stamp = msg->header.stamp;
            camera_origin_cam.header.frame_id = camera_frame_;
            camera_origin_cam.point.x = 0.0;
            camera_origin_cam.point.y = 0.0;
            camera_origin_cam.point.z = 0.0;

            geometry_msgs::msg::PointStamped camera_origin;
            const bool r_ok = transformPointToTarget(r_cam, msg->header.stamp, r_center);
            const bool target_ok = transformPointToTarget(target_cam, msg->header.stamp, target_center);
            const bool origin_ok = transformPointToTarget(camera_origin_cam, msg->header.stamp, camera_origin);
            if (target_ok&&r_ok) {
                aiming_msg.r_x_3d = static_cast<float>(r_center.point.x);
                aiming_msg.r_y_3d = static_cast<float>(r_center.point.y);
                aiming_msg.r_z_3d = static_cast<float>(r_center.point.z);
                aiming_msg.target_x_3d = static_cast<float>(target_center.point.x);
                aiming_msg.target_y_3d = static_cast<float>(target_center.point.y);
                aiming_msg.target_z_3d = static_cast<float>(target_center.point.z);
                if (origin_ok) {
                    const double axis_x = r_center.point.x - camera_origin.point.x;
                    const double axis_y = r_center.point.y - camera_origin.point.y;
                    const double axis_z = r_center.point.z - camera_origin.point.z;
                    const double axis_norm = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
                    if (axis_norm > 1e-6) {
                        aiming_msg.axis_x_3d = static_cast<float>(axis_x / axis_norm);
                        aiming_msg.axis_y_3d = static_cast<float>(axis_y / axis_norm);
                        aiming_msg.axis_z_3d = static_cast<float>(axis_z / axis_norm);
                    }
                }
            }
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Invalid radius for depth solve(raw=%.4f, filtered=%.4f)",
                msg->radius,
                filtered_radius);
        }

        //不在跟踪时重置状态并发布默认数据
        if (msg->is_bigbuff == 1) {
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
            double pixel_arm_length = filtered_radius;

            // 角度连续化
            const float continues_angle = angle_observer_->update(vector_x, vector_y, pixel_arm_length);
            aiming_msg.angle = continues_angle;
            const auto now = rclcpp::Time(msg->header.stamp).seconds();

            // 记录第一帧时间戳
            if (isFirst_frame_) {
                fit_start_time_ = now;
                isFirst_frame_ = false;
            }

            time_since_start_ = now - fit_start_time_; // 第一帧这个值为0
            if (!predictor_->has_fit_attempted() && time_since_start_ > 0 && time_since_start_ < predictor_->get_fit_window_sec()) {
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
            if (!predictor_->is_completed() && time_since_start_ >= predictor_->get_fit_window_sec()) {
                predictor_->try_fit_once_at_1p5s();
                if(debug_mode_){
                    debug_csv_<<'\n' << predictor_->get_velocity_fit_params()[0] << ","
                                     << predictor_->get_velocity_fit_params()[1] << ","
                                     << predictor_->get_velocity_fit_params()[2] << ","
                                     << predictor_->get_velocity_fit_params()[3] << '\n'<<std::endl;
                }
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
            radius_mean_filter_ =
             std::make_unique<MovAvg>(std::max(1, radius_mean_window_));
            isFirst_frame_ = true;
            last_time_since_start_ = 0.0;
            time_since_start_ = 0.0;
            last_angle_ = 0.0f;
            setParaDefaultAimingMsg(aiming_msg);
        }

        
        aiming_pub_->publish(aiming_msg);
    }

    void setDefaultAimingMsg(buff_interfaces::msg::BuffAimingData& aiming_msg)
    {
        aiming_msg.is_bigbuff = 0;
        aiming_msg.spin_direction = 0;
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
    std::unique_ptr<MovAvg> radius_mean_filter_;

    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    bool has_cam_info_ = false;

    std::string target_frame_;
    std::string camera_frame_;
    bool debug_mode_ = true;
    std::string debug_csv_path_ = "buff_estimator_debug.csv";
    int velocity_median_window_ = 3;
    int radius_mean_window_ = 15;
    std::ofstream debug_csv_;
    bool isFirst_frame_ = true;
    double last_time_since_start_ = 0.0f;
    float last_angle_ = 0.0f;
    double fit_start_time_ = 0.0f;
    double time_since_start_ = 0.0f;
    double physical_arm_length_ = 0.75;
};



// 注册为组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffEstimatorNode)

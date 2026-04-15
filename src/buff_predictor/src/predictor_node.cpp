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
        
        //debug mode记录csv
        if(DEBUG_MODE){
        debug_csv_.open("buff_predictor_debug.csv");

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
    {   if(!msg->is_tracking){
        predictor_->reset();
        isFirst_frame_ = true;
        return;
    }
        // 计算向量：扇叶框中心 - R框中心
        float vector_x = msg->target_center_x - msg->r_center_x;
        float vector_y = msg->target_center_y - msg->r_center_y;

        double pixel_arm_length = msg->radius;

       // 角度连续化
        const float cont  void targetCallback(const buff_interfaces::msg::BuffTarget::SharedPtr msg)
    {   if(!msg->is_tracking){
        predictor_->reset();
        isFirst_frame_ = true;
        return;
    }
        // 计算向量：扇叶框中心 - R框中心
        float vector_x = msg->target_center_x - msg->r_center_x;
        float vector_y = msg->target_center_y - msg->r_center_y;

        double pixel_arm_length = msg->radius;

       // 角度连续化
        const float continues_angle = angle_observer_->update(vector_x, vector_y, pixel_arm_length);
        const auto now = rclcpp::Time(msg->header.stamp).seconds();
        //记录第一帧时间戳
        if(isFirst_frame_){
        fit_start_time_ = now;
        isFirst_frame_ = false;
        }
        time_since_start_ = now - fit_start_time_;//第一帧这个值为0
        if(time_since_start_>0 && time_since_start_< 1.5){
        double median_time = last_time_since_start_ + time_since_start_ / 2.0;
        float mean_angleVelocity = (continues_angle - last_angle_) / (time_since_start_- last_time_since_start_);
        const std::pair<double,float> time_w_pair (median_time, mean_angleVelocity);

        if(DEBUG_MODE){
            debug_csv_<<median_time<<","<<mean_angleVelocity<<std::endl;
        }

        predictor_->time_w_pairs_.push_back(time_w_pair);
    }
      inues_angle = angle_observer_->update(vector_x, vector_y, pixel_arm_length);
        const auto now = rclcpp::Time(msg->header.stamp).seconds();
        //记录第一帧时间戳
        if(isFirst_frame_){
        fit_start_time_ = now;
        isFirst_frame_ = false;
        }
        time_since_start_ = now - fit_start_time_;//第一帧这个值为0
        if(time_since_start_>0 && time_since_start_< 1.5){
        double median_time = (last_time_since_start_ + time_since_start_ )/ 2.0;
        float mean_angleVelocity = (continues_angle - last_angle_) / (time_since_start_- last_time_since_start_);
        const std::pair<double,float> time_w_pair (median_time, mean_angleVelocity);

        if(DEBUG_MODE){
            debug_csv_<<median_time<<","<<std::abs(mean_angleVelocity)<<std::endl;
        }

        predictor_->time_w_pairs_.push_back(time_w_pair);
    }
        last_time_since_start_ = time_since_start_;
        last_angle_ = angle_observer_ ->last_angle_;}


        geometry_msgs::msg::PointStamped point_odom;
        geometry_msgs::msg::PointStamped target_odom;

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
    bool params_published_ = false;
    int frame_count_ = 0;

    // TF frame settings
    std::string target_frame_;
    std::string camera_frame_;

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

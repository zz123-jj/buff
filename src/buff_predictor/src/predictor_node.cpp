#include "predictor.hpp"
#include "coordinate_solver.hpp"
#include "config.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "buff_interfaces/msg/buff_target.hpp"
#include "buff_interfaces/msg/buff_prediction.hpp"
#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/utils.h>
#include <fstream>
#include <iomanip>
#include <cmath>

class BuffPredictorNode : public rclcpp::Node
{
public:
    BuffPredictorNode() : Node("buff_predictor_node")
    {
        // 半径的滤波
        radius_filter_ = std::make_unique<MovAvg>(15);

        // 算法初始化（使用配置参数）
        predictor_ = std::make_unique<Big_Buff_Predictor>(DELTA_T);
        
        // 角度连续化初始化
        angle_observer_ = std::make_unique<angleObserver>(clockMode::anticlockwise);

        // 3D解算初始化（使用配置参数）
        CameraParams cam_params(1303.675283386667f, 1303.675283386667f, 720.0f, 540.0f);
        predictor_3d_ = std::make_unique<Predictor3D>(cam_params, PHYSICAL_ARM_LENGTH);
        // 坐标转换初始化
        CoordinateSolver::CameraIntrinsics coord_params = {1303.675283386667f, 1303.675283386667f, 720.0f, 540.0f};
        coord_solver_ = std::make_unique<CoordinateSolver>(coord_params);

        // Sub & Pub
        target_sub_ = this->create_subscription<buff_interfaces::msg::BuffTarget>(
            "/buff_target", 10,
            std::bind(&BuffPredictorNode::targetCallback, this, std::placeholders::_1));

        aiming_pub_ = this->create_publisher<buff_interfaces::msg::BuffAimingData>(
            "/buff/aiming_data", 10);

        // 接受tf
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        prediction_pub_ = this->create_publisher<buff_interfaces::msg::BuffPrediction>("/buff_prediction", 10);
        if (DEBUG_MODE) {
            tracker_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/tracker/debug_point", 10);
        }
    }

private:
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
        // 相机帧的时间戳（整数秒 + 纳秒）
        const auto stamp = rclcpp::Time(msg->header.stamp);
        const int32_t stamp_sec_int = stamp.seconds();
        const uint32_t stamp_nsec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
        // 仅保留小数点前后四位：秒取后四位，纳秒取前四位
        const int32_t stamp_sec_4 = std::abs(stamp_sec_int) % 10000;
        const uint32_t stamp_nsec_4 = stamp_nsec / 100000; // 取纳秒前四位
        const double stamp_sec_limited = static_cast<double>(stamp_sec_4) +
                         static_cast<double>(stamp_nsec_4) / 10000.0;
        // 记录连续角度与时间戳到 CSV（仅在debug模式下）
        if (DEBUG_MODE)
        {
            static std::ofstream csv_file("log/angle_time.csv", std::ios::app);
            static bool wrote_header = false;
            if (csv_file.is_open())
            {
                if (!wrote_header)
                {
                    csv_file << "stamp_sec,continues_angle\n";
                    wrote_header = true;
                }
                csv_file << std::setw(4) << std::setfill('0') << stamp_sec_4
                         << "." << std::setw(4) << std::setfill('0') << stamp_nsec_4
                         << "," << continues_angle << "\n";
            }
        }
        //更新预测器
        auto result = predictor_->update(continues_angle, stamp_sec_limited);
        bool success = result.first;
        float angle_delta = result.second;

        geometry_msgs::msg::PointStamped point_odom;
        geometry_msgs::msg::PointStamped target_odom;


        //发布buffPrediction（不依赖TF，始终发布）
        auto pred_msg = buff_interfaces::msg::BuffPrediction();
        pred_msg.header.stamp = msg->header.stamp;
        pred_msg.header.frame_id = "odom";
        pred_msg.is_fitted = success;
        pred_msg.angle_delta = angle_delta;
        pred_msg.frame_number = frame_count_;
        pred_msg.current_angle = continues_angle;

        // 填入sin参数
        if (success) {
            const auto sin_para = predictor_->get_sin_para();
            if (sin_para.size() >= 4) {
                pred_msg.sin_a = sin_para[0];
                pred_msg.sin_omega = sin_para[1];
                pred_msg.sin_phi = sin_para[2];
                pred_msg.sin_b = sin_para[3];
            }
        }
        prediction_pub_->publish(pred_msg);

        // TF 变换与坐标解算
        try
        {
            // 计算深度
            float depth = predictor_3d_->compute_depth(pixel_arm_length);

            // R标坐标
            auto p_cam_pt = coord_solver_->pixelToCamera(msg->r_center_x, msg->r_center_y, depth);

            geometry_msgs::msg::PointStamped point_cam;
            point_cam.header.stamp = msg->header.stamp; // 对齐时间戳
            point_cam.header.frame_id = "camera_optical_frame";
            point_cam.point.x = p_cam_pt.x;
            point_cam.point.y = p_cam_pt.y;
            point_cam.point.z = p_cam_pt.z;


            // TF2 转换到 Odom
            // tf_buffer_->transform 处理旋转和平移
            geometry_msgs::msg::TransformStamped tf_stamped =
                tf_buffer_->lookupTransform(
                    "odom", // 目标帧
                    point_cam.header.frame_id, // 源帧（point_cam的坐标系）
                    msg->header.stamp, // 时间戳
                    tf2::durationFromSec(0.05) // 超时时间
                );

            tf2::doTransform(point_cam, point_odom, tf_stamped);

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
            target_point_cam.header.frame_id = "camera_optical_frame";
            target_point_cam.point.x = target_point_pt.x;
            target_point_cam.point.y = target_point_pt.y;
            target_point_cam.point.z = target_point_pt.z;

            

            geometry_msgs::msg::TransformStamped tf_stamped_target =
                tf_buffer_->lookupTransform(
                    "odom", // 目标帧
                    target_point_cam.header.frame_id, // 源帧（target_point_cam的坐标系）
                    msg->header.stamp, // 时间戳
                    tf2::durationFromSec(0.05) // 超时时间
                );

            tf2::doTransform(target_point_cam, target_odom, tf_stamped_target);

        

        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error: %s", ex.what());
            return; // TF失败时跳过3D坐标填充，prediction已在前面发布
        }

        // TF成功时填入3D坐标并发布aiming数据
        pred_msg.r_center_odom.x = point_odom.point.x;
        pred_msg.r_center_odom.y = point_odom.point.y;
        pred_msg.r_center_odom.z = point_odom.point.z;

        // 发布调试点 (仅在 DEBUG_MODE)
        if (DEBUG_MODE) {
            auto debug_point_msg = geometry_msgs::msg::PointStamped();
            debug_point_msg.header.stamp = msg->header.stamp;
            debug_point_msg.header.frame_id = "odom";
            debug_point_msg.point = target_odom.point;
            tracker_pub_->publish(debug_point_msg);
        }

        // buffAimingData 发布（仅在TF成功时有3D坐标）
        auto aiming_msg = buff_interfaces::msg::BuffAimingData();
        aiming_msg.header.stamp = this->now();
        aiming_msg.header.frame_id = "odom";
        aiming_msg.frame_number = frame_count_;
        aiming_msg.is_tracking = success;
        aiming_msg.r_center_x_3d = point_odom.point.x;
        aiming_msg.r_center_y_3d = point_odom.point.y;
        aiming_msg.r_center_z_3d = point_odom.point.z;
        aiming_msg.angle_delta = angle_delta;
        aiming_msg.pixel_r_center_x = msg->r_center_x;
        aiming_msg.pixel_r_center_y = msg->r_center_y;
        aiming_msg.pixel_radius = pixel_arm_length;
        aiming_msg.target_x_3d = target_odom.point.x;
        aiming_msg.target_y_3d = target_odom.point.y;
        aiming_msg.target_z_3d = target_odom.point.z;
        aiming_pub_->publish(aiming_msg);
    }

    // ROS2 Components
    rclcpp::Subscription<buff_interfaces::msg::BuffTarget>::SharedPtr target_sub_;
    rclcpp::Publisher<buff_interfaces::msg::BuffPrediction>::SharedPtr prediction_pub_;
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

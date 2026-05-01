#include "buff_detector/detector.hpp"
#include "buff_interfaces/msg/buff_target.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#if __has_include("cv_bridge/cv_bridge.hpp")
  #include "cv_bridge/cv_bridge.hpp"
#elif __has_include("cv_bridge/cv_bridge.h")
  #include "cv_bridge/cv_bridge.h"
#else
  #error "cv_bridge headers not found"
#endif
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>


class BuffDetectorNode : public rclcpp::Node
{
public:
    explicit BuffDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("buff_detector_node", options)
    {
        this->declare_parameter<std::string>("model_path", "model/yolo11_buff_int8.xml");
        this->declare_parameter<std::string>("openvino_device", "GPU");
        this->declare_parameter<double>("confidence_threshold", 0.88);
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<bool>("image_reliable", true);
        this->declare_parameter<std::string>("buff_mode", "auto");
        this->declare_parameter<double>("mode_judge_far_distance_px", 80.0);
        this->declare_parameter<bool>("yolo_target_lock_enabled", true);
        this->declare_parameter<double>("yolo_lock_hold_sec", 2.5);
        this->declare_parameter<double>("yolo_lock_min_confidence_ratio", 0.55);
        this->declare_parameter<double>("yolo_lock_max_angle_error_rad", 0.60);


        std::string model_path = this->get_parameter("model_path").as_string();
        if (std::filesystem::path(model_path).is_relative()) {
            model_path =
                ament_index_cpp::get_package_share_directory("buff_detector") + "/" + model_path;
        }
        detector_config_.model_path = model_path;
        detector_config_.openvino_device = this->get_parameter("openvino_device").as_string();
        detector_config_.confidence_threshold =
            static_cast<float>(this->get_parameter("confidence_threshold").as_double());
        detector_config_.debug_mode = this->get_parameter("debug_mode").as_bool();
        image_reliable_ = this->get_parameter("image_reliable").as_bool();
        detector_config_.buff_mode = this->get_parameter("buff_mode").as_string();
        detector_config_.mode_judge_far_distance_px =
            static_cast<float>(this->get_parameter("mode_judge_far_distance_px").as_double());
        detector_config_.yolo_target_lock_enabled =
            this->get_parameter("yolo_target_lock_enabled").as_bool();
        detector_config_.yolo_lock_hold_sec =
            static_cast<float>(this->get_parameter("yolo_lock_hold_sec").as_double());
        detector_config_.yolo_lock_min_confidence_ratio =
            static_cast<float>(this->get_parameter("yolo_lock_min_confidence_ratio").as_double());
        detector_config_.yolo_lock_max_angle_error_rad =
            static_cast<float>(this->get_parameter("yolo_lock_max_angle_error_rad").as_double());


        // 初始化检测器
        detector_ = std::make_unique<BuffDetector>();
        detector_->set_config(detector_config_);
        //video_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::QoS(rclcpp::KeepLast(1)).reliable()); 
        // 订阅图像
        auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        if (image_reliable_) {
            image_qos.reliable();
        } else {
            image_qos.best_effort();
        }
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw",
    image_qos,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { 
        imageCallback(msg); 
    });
        // 发布目标信息
        target_pub_ = this->create_publisher<buff_interfaces::msg::BuffTarget>("/buff/target", 10);

        if (detector_config_.debug_mode) {
            debug_image_pub_ = image_transport::create_publisher(this, "/buff/debug_image");
            preprocessed_image_pub_ = image_transport::create_publisher(this, "/buff/preprocessed_image");
            roi_image_pub_ = image_transport::create_publisher(this, "/buff/roi_image");
        }
       preprocessed_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/buff/preprocessed_image", 10);
         fan_roi_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/buff/fan_roi_image", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 将ROS图像消息转换为OpenCV格式
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame.");
            return;
        }

        try {
            processFrame(frame, msg->header.stamp);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "processFrame threw exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "processFrame threw unknown exception");
        }
        //sendVedioImage(video_frame_);
    }

    void processFrame(cv::Mat frame, const rclcpp::Time& frame_stamp)
    {
        if (!detector_->update(frame, frame_stamp.seconds()))
        {
            auto target_msg = buff_interfaces::msg::BuffTarget();
            target_msg.header.stamp = frame_stamp;
            target_msg.header.frame_id = "camera_optical_frame";
            target_msg.is_tracking = false;
            set_defaultBuffTarget(target_msg);
            target_pub_->publish(std::move(target_msg));
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "YOLOPose did not find a valid buff target.");
            return;
        }

        // 获取中心点坐标
        cv::Point2f fan_center = detector_->get_first_target_blade_bbox().get_center_2f();
        cv::Point2f r_center = detector_->get_current_R_box().get_center_2f();
     // 发布目标信息
        auto target_msg = buff_interfaces::msg::BuffTarget();
        target_msg.is_tracking = true;
        target_msg.header.stamp = frame_stamp;
        target_msg.header.frame_id = "camera_optical_frame";
        target_msg.is_bigbuff = (detector_->get_buff_type() == BuffType::big_buff ? 1 : -1); // 获取当前buff模式
        target_msg.spin_direction = detector_->get_spin_direction();
        target_msg.target_center_x = fan_center.x;
        target_msg.target_center_y = fan_center.y;
        target_msg.r_center_x = r_center.x;
        target_msg.r_center_y = r_center.y;
        target_msg.radius = detector_->get_radius(); // 获取能量机关半径
        target_msg.pose_confidence = detector_->get_pose_confidence();
        target_msg.yolo_target_count = detector_->get_yolo_target_count();
        const auto& keypoints = detector_->get_current_target_keypoints();
        target_msg.has_target_keypoints = keypoints.size() >= 4;
        target_msg.target_keypoints.fill(0.0f);
        for (std::size_t i = 0; i < keypoints.size() && i < 4; ++i) {
            target_msg.target_keypoints[i * 2] = keypoints[i].x;
            target_msg.target_keypoints[i * 2 + 1] = keypoints[i].y;
        }
        target_pub_->publish(std::move(target_msg));

        // 显示调试信息
        if (detector_config_.debug_mode)
        {   
            // cv::imshow("visualization", detector_->debug_frame_);
            // cv::waitKey(1);
            
            // 发布调试图像到 ROS Topic
            if (!detector_->debug_frame_.empty())
            {   
                auto debug_frame = detector_->debug_frame_;
                cv::circle(detector_->debug_frame_, 
           cv::Point(detector_->debug_frame_.cols/2, detector_->debug_frame_.rows/2), 
           4, 
           cv::Scalar(0, 255, 0), 
           2);
                cv::resize(debug_frame, debug_frame,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
                auto debug_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_frame).toImageMsg();
                debug_img_msg->header.stamp = frame_stamp;
                debug_img_msg->header.frame_id = "camera";
                debug_image_pub_.publish(std::move(debug_img_msg));
            }
          if (!detector_->preprocessed_frame_.empty())
            {
                auto preprocessed_img_msg =
                    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", detector_->preprocessed_frame_).toImageMsg();
                preprocessed_img_msg->header.stamp = frame_stamp;
                preprocessed_img_msg->header.frame_id = "camera";
                preprocessed_image_pub_.publish(preprocessed_img_msg);
            }

            if (!detector_->roi_frame_.empty())
            {
                auto roi_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", detector_->roi_frame_).toImageMsg();
                roi_img_msg->header.stamp = frame_stamp;
                roi_img_msg->header.frame_id = "camera";
                roi_image_pub_.publish(roi_img_msg);
            }
        }
       
    }

    void set_defaultBuffTarget(buff_interfaces::msg::BuffTarget &msg) {
        msg.is_bigbuff = 0;
        msg.spin_direction = 0;
        msg.target_center_x = 0.0f;
        msg.target_center_y = 0.0f;
        msg.r_center_x = 0.0f;
        msg.r_center_y = 0.0f;
        msg.radius = 0.0f;
        msg.has_target_keypoints = false;
        msg.target_keypoints.fill(0.0f);
        msg.pose_confidence = 0.0f;
        msg.yolo_target_count = 0;
       }
  
    // ROS2 通信
    rclcpp::Publisher<buff_interfaces::msg::BuffTarget>::SharedPtr target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preprocessed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fan_roi_pub_;
    // 检测器
    std::unique_ptr<BuffDetector> detector_;
    // 配置
    BuffDetectorConfig detector_config_;
    bool image_reliable_ = true;

    image_transport::Publisher debug_image_pub_;
    image_transport::Publisher preprocessed_image_pub_;
    image_transport::Publisher roi_image_pub_;
    cv::Mat video_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuffDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// 注册为组件
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(BuffDetectorNode)

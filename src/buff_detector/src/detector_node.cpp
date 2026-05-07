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
        this->declare_parameter<double>("iou_threshold", 0.5);
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<bool>("image_reliable", true);
        this->declare_parameter<double>("inside_shade_rate", 0.7);
        this->declare_parameter<double>("outside_shade_rate", 1.39);
        this->declare_parameter<std::vector<int64_t>>("hsv_limits.lower", {0, 40, 220});
        this->declare_parameter<std::vector<int64_t>>("hsv_limits.upper", {70, 255, 255});
        this->declare_parameter<int>("dilate_kernel_size", 7);
        this->declare_parameter<int>("max_lost_frame", 30);


        std::string model_path = this->get_parameter("model_path").as_string();
        if (std::filesystem::path(model_path).is_relative()) {
            model_path =
                ament_index_cpp::get_package_share_directory("buff_detector") + "/" + model_path;
        }
        detector_config_.model_path = model_path;
        detector_config_.openvino_device = this->get_parameter("openvino_device").as_string();
        detector_config_.confidence_threshold =
            static_cast<float>(this->get_parameter("confidence_threshold").as_double());
        detector_config_.iou_threshold =
            static_cast<float>(this->get_parameter("iou_threshold").as_double());
        detector_config_.debug_mode = this->get_parameter("debug_mode").as_bool();
        image_reliable_ = this->get_parameter("image_reliable").as_bool();
        detector_config_.inside_shade_rate =
            static_cast<float>(this->get_parameter("inside_shade_rate").as_double());
        detector_config_.outside_shade_rate =
            static_cast<float>(this->get_parameter("outside_shade_rate").as_double());
            auto lower_vec = this->get_parameter("hsv_limits.lower").as_integer_array();
            auto upper_vec = this->get_parameter("hsv_limits.upper").as_integer_array();    
        detector_config_.lower_hsv = cv::Scalar(
            lower_vec[0],
            lower_vec[1],
            lower_vec[2]);
        detector_config_.upper_hsv = cv::Scalar(
            upper_vec[0],
            upper_vec[1],
            upper_vec[2]);
        detector_config_.dilate_kernel_size = this->get_parameter("dilate_kernel_size").as_int();
        detector_config_.max_lost_frame = this->get_parameter("max_lost_frame").as_int();


        // 初始化检测器
        detector_ = std::make_unique<BuffDetector>();
        detector_->set_config(detector_config_);
    
        auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        if (image_reliable_) {
            image_qos.reliable();
            image_qos.durability_volatile();
        } else {
            image_qos.best_effort();
            image_qos.durability_volatile();
        }
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        image_qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { 
            imageCallback(msg); 
        });
        // 发布目标信息
        target_pub_ = this->create_publisher<buff_interfaces::msg::BuffTarget>("/buff/target", rclcpp::SensorDataQoS());

        if (detector_config_.debug_mode) {
            debug_image_pub_ = image_transport::create_publisher(this, "/buff/debug_image");
            preprocessed_image_pub_ = image_transport::create_publisher(this, "/buff/preprocessed_image");
            roi_image_pub_ = image_transport::create_publisher(this, "/buff/roi_image");
        }
        preprocessed_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/buff/preprocessed_image", rclcpp::SensorDataQoS());
        fan_roi_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/buff/fan_roi_image", rclcpp::SensorDataQoS());
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
        // 处理第一帧
        if (!is_tracking_)
        {
            cv::Mat frame_copy = frame.clone(); // 克隆帧以供检测器使用
            if (detector_->init(frame_copy))
            {
                is_tracking_ = true;
                RCLCPP_INFO(this->get_logger(), "Detector initialized.");
            }else{
                RCLCPP_INFO(this->get_logger(),"Failed to initialize detector.");
                auto msg = buff_interfaces::msg::BuffTarget();
                set_defaultBuffTarget(msg);
                msg.is_tracking = false;
                msg.header.stamp = frame_stamp;
                target_pub_->publish(msg);

            if (detector_config_.debug_mode)
            {   
            // 发布调试图像到 ROS Topic
            if (!detector_->debug_frame_.empty())
            {   
                auto debug_frame = frame_copy;
                cv::circle(detector_->debug_frame_, 
                cv::Point(detector_->debug_frame_.cols/2, detector_->debug_frame_.rows/2), 
                4, 
                cv::Scalar(0, 255, 0), 
                2);
                cv::resize(debug_frame, debug_frame,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
                auto debug_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_frame).toImageMsg();
                debug_img_msg->header.stamp = frame_stamp;
                debug_img_msg->header.frame_id = "camera_optical_frame";
                debug_image_pub_.publish(std::move(debug_img_msg));
            }
            }
            return;
            }}

        // 更新检测
        if (is_tracking_)
        {
            if (!detector_->update(frame))
            {
                is_tracking_ = false;
                RCLCPP_WARN(this->get_logger(), "Detector lost target, re-initialization required.");
                auto msg = buff_interfaces::msg::BuffTarget();
                set_defaultBuffTarget(msg);
                msg.is_tracking = false;
                msg.header.stamp = frame_stamp;
                target_pub_->publish(msg);
                return;
            }

        // 获取中心点坐标
        cv::Point2f fan_center = detector_->get_first_target_blade_bbox().get_center_2f();
        cv::Point2f r_center = detector_->get_current_R_box().get_center_2f();
     // 发布目标信息
        auto target_msg = buff_interfaces::msg::BuffTarget();
        target_msg.is_tracking = true;
        target_msg.header.stamp = frame_stamp;
        target_msg.is_bigbuff = (detector_->get_buff_type() == BuffType::big_buff ? 1 : -1); // 获取当前buff模式
        target_msg.spin_direction = detector_->get_spin_direction();
        target_msg.target_center_x = fan_center.x;
        target_msg.target_center_y = fan_center.y;
        target_msg.r_center_x = r_center.x;
        target_msg.r_center_y = r_center.y;
        target_msg.radius = detector_->get_radius(); // 获取能量机关半径
        target_pub_->publish(std::move(target_msg));
        }
        // 显示调试信息
        if (detector_config_.debug_mode)
        {   
           
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
    
                debug_img_msg->header.frame_id = "camera_optical_frame";
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
    bool is_tracking_ = false;
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

#include "buff_detector/detector.hpp"
#include "buff_interfaces/msg/buff_target.hpp"
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>


class BuffDetectorNode : public rclcpp::Node
{
public:
    BuffDetectorNode() : Node("buff_detector_node")
    {
        this->declare_parameter<std::string>("model_path", "src/buff_detector/model/Fan.onnx");
        this->declare_parameter<bool>("use_cuda", false);
        this->declare_parameter<double>("confidence_threshold", 0.5);
        this->declare_parameter<double>("iou_threshold", 0.5);
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<double>("inside_shade_rate", 0.7);
        this->declare_parameter<double>("outside_shade_rate", 1.39);
        this->declare_parameter<int>("lower_h", 0);
        this->declare_parameter<int>("lower_s", 40);
        this->declare_parameter<int>("lower_v", 220);
        this->declare_parameter<int>("upper_h", 70);
        this->declare_parameter<int>("upper_s", 255);
        this->declare_parameter<int>("upper_v", 255);
        this->declare_parameter<int>("dilate_kernel_size", 7);
        this->declare_parameter<int>("max_lost_frame", 5);
        this->declare_parameter<std::string>("debug_image_frame_id", "camera");

        detector_config_.model_path = this->get_parameter("model_path").as_string();
        detector_config_.use_cuda = this->get_parameter("use_cuda").as_bool();
        detector_config_.confidence_threshold =
            static_cast<float>(this->get_parameter("confidence_threshold").as_double());
        detector_config_.iou_threshold =
            static_cast<float>(this->get_parameter("iou_threshold").as_double());
        detector_config_.debug_mode = this->get_parameter("debug_mode").as_bool();
        detector_config_.inside_shade_rate =
            static_cast<float>(this->get_parameter("inside_shade_rate").as_double());
        detector_config_.outside_shade_rate =
            static_cast<float>(this->get_parameter("outside_shade_rate").as_double());
        detector_config_.lower_hsv = cv::Scalar(
            this->get_parameter("lower_h").as_int(),
            this->get_parameter("lower_s").as_int(),
            this->get_parameter("lower_v").as_int());
        detector_config_.upper_hsv = cv::Scalar(
            this->get_parameter("upper_h").as_int(),
            this->get_parameter("upper_s").as_int(),
            this->get_parameter("upper_v").as_int());
        detector_config_.dilate_kernel_size = this->get_parameter("dilate_kernel_size").as_int();
        detector_config_.max_lost_frame = this->get_parameter("max_lost_frame").as_int();
        //debug_image_frame_id_ = this->get_parameter("debug_image_frame_id").as_string();

        // 初始化检测器
        detector_ = std::make_unique<BuffDetector>();
        detector_->set_config(detector_config_);
        //video_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::QoS(rclcpp::KeepLast(1)).reliable()); 
        // 订阅图像
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });

        // 发布目标信息
        target_pub_ = this->create_publisher<buff_interfaces::msg::BuffTarget>("/buff_target", 10);

        if (detector_config_.debug_mode) {
            debug_image_pub_ = image_transport::create_publisher(this, "/buff_detector/debug_image");
        }
        //video_capture_.open("video/8mm_red_bright.mp4"); 
        //sendVedioImage(video_frame_);
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
        }}

        // 更新检测
        if (is_tracking_)
        {
            if (!detector_->update(frame))
            {
                is_tracking_ = false;
                RCLCPP_WARN(this->get_logger(), "Detector lost target, re-initialization required.");
            }

        // 获取中心点坐标
        cv::Point2f fan_center = detector_->get_first_target_blade_bbox().get_center_2f();
        cv::Point2f r_center = detector_->get_current_R_box().get_center_2f();
     // 发布目标信息
        auto target_msg = buff_interfaces::msg::BuffTarget();
        target_msg.header.stamp = frame_stamp;
        target_msg.buff_type = static_cast<uint8_t>(detector_->get_buff_type()); // 获取当前buff模式
        target_msg.radius = detector_->get_radius(); // 获取能量机关半径
        target_msg.target_center_x = fan_center.x;
        target_msg.target_center_y = fan_center.y;
        target_msg.r_center_x = r_center.x;
        target_msg.r_center_y = r_center.y;
        target_msg.is_tracking = is_tracking_;
        target_msg.spin_direction = detector_->get_spin_direction();
        target_pub_->publish(target_msg);
        }
        // 显示调试信息
        if (detector_config_.debug_mode)
        {   
            // cv::imshow("visualization", detector_->debug_frame_);
            // cv::waitKey(1);
            
            // 发布调试图像到 ROS Topic
            if (!detector_->debug_frame_.empty())
            {
                auto debug_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", detector_->debug_frame_).toImageMsg();
                debug_img_msg->header.stamp = frame_stamp;
                debug_img_msg->header.frame_id = debug_image_frame_id_;
                debug_image_pub_.publish(debug_img_msg);
            }
          
        }
       
    }
    // void sendVedioImage(cv::Mat frame){
    //  video_capture_.read(frame);
    //     auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",frame).toImageMsg();
    //     msg->header.stamp = this->now();
    //     video_img_pub_->publish(*msg);}

    // ROS2 通信
    rclcpp::Publisher<buff_interfaces::msg::BuffTarget>::SharedPtr target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_img_pub_; 
    
    // 检测器
    std::unique_ptr<BuffDetector> detector_;
    // 配置
    BuffDetectorConfig detector_config_;

    image_transport::Publisher debug_image_pub_;
    
    std::string debug_image_frame_id_ = "camera";    

  // 状态
    bool is_tracking_ = false;
 
   // cv::VideoCapture video_capture_;
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

#include "buff_detector/config.hpp"
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
        // 初始化检测器
        detector_ = std::make_unique<BuffDetector>();
        video_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::QoS(rclcpp::KeepLast(1)).reliable()); 
        // 订阅图像
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });

        // 发布目标信息
        target_pub_ = this->create_publisher<buff_interfaces::msg::BuffTarget>("/buff_target", 10);

        if (IS_DEBUG) {
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
            }
            RCLCPP_INFO(this->get_logger(),"Failed to initialize detector.");
        }

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
        target_pub_->publish(target_msg);
        }
        // 显示调试信息
        if constexpr (IS_DEBUG)
        {   
            // cv::imshow("visualization", detector_->debug_frame_);
            // cv::waitKey(1);
            
            // 发布调试图像到 ROS Topic
            if (!detector_->debug_frame_.empty())
            {
                auto debug_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", detector_->debug_frame_).toImageMsg();
                debug_img_msg->header.stamp = frame_stamp;
                debug_img_msg->header.frame_id = "camera";
                debug_image_pub_.publish(debug_img_msg);
            }
          
        }
       
    }
    void sendVedioImage(cv::Mat frame){
     video_capture_.read(frame);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",frame).toImageMsg();
        msg->header.stamp = this->now();
        video_img_pub_->publish(*msg);
}

    // ROS2 通信
    rclcpp::Publisher<buff_interfaces::msg::BuffTarget>::SharedPtr target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_img_pub_; 
    
    // 检测器
    std::unique_ptr<BuffDetector> detector_;
    image_transport::Publisher debug_image_pub_;

    // 状态
    bool is_tracking_ = false;
    cv::VideoCapture video_capture_;
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

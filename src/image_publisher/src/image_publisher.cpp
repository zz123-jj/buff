#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <std_msgs/msg/empty.hpp>

std::string const VIDEO_PATH = "video/8mm_red_bright.mp4";
int const PUB_FPS = 50; 

class ImagePublisher : public rclcpp::Node
{
private:
cv::VideoCapture video_capture_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()              // 可靠传输
            .durability_volatile();  // 不保存历史
    int pub_interval_ms = static_cast<int>(1000.0 / PUB_FPS);

    void publishFrame()
    {
        cv::Mat frame;
        // 循环播放
        if (!video_capture_.read(frame)) {
                RCLCPP_INFO(this->get_logger(), "Video ended");
            return;
            }
        if (frame.empty()) return;
        
        // 将 OpenCV 图像转换为 ROS 消息并发布
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        img_pub_->publish(*msg);
       
    
    }
public:
    ImagePublisher() : Node("image_publisher")
    {
        if(!video_capture_.open(VIDEO_PATH)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", VIDEO_PATH.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "video opened, preparing to publish at %d FPS", PUB_FPS);
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", qos_);
        timer_= this->create_wall_timer(
            std::chrono::milliseconds(pub_interval_ms),
            [this]{this->publishFrame();}
        );
    }
};
    int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
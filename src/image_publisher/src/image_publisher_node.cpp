#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <std_msgs/msg/empty.hpp>

//调试开关
constexpr bool IS_DEBUG = true;
constexpr int CAMERA_ID = -1;  // 默认摄像头ID （-1表示使用视频文件,0表示摄像头）

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode() : Node("image_publisher_node"), is_camera_(false), frame_count_(0)
    {
        std::string const video_path = "video/bigbuff2.mp4"; 

        if (CAMERA_ID >= 0) {
            cap_.open(CAMERA_ID);
            is_camera_ = true;
            RCLCPP_INFO(this->get_logger(), "Opening camera with ID: %d", CAMERA_ID);
        } else {
            cap_.open(video_path);
            is_camera_ = false;
            RCLCPP_INFO(this->get_logger(), "Opening video file: %s", video_path.c_str());
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video/camera");
            rclcpp::shutdown();
            return;
        }

        // 获取FPS（摄像头可能不支持，设为30）
        double fps = cap_.get(cv::CAP_PROP_FPS);
        if (fps <= 0 || is_camera_) fps = 30.0;  // 摄像头默认30FPS
        frame_delay_ms_ = static_cast<int>(1000.0 / fps);
        
        RCLCPP_INFO(this->get_logger(), "FPS: %.2f, Frame delay: %d ms", fps, frame_delay_ms_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()              // 可靠传输
            .durability_volatile();  // 不保存历史
            
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", qos);

        // pull 模式：收到 detector 的 /frame_request 后才发布下一帧
        frame_request_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/frame_request",
            rclcpp::QoS(10).reliable(),
            [this](const std_msgs::msg::Empty::SharedPtr) {
                publishFrame();
            }
        );

        RCLCPP_INFO(this->get_logger(), "Image publisher ready (pull mode), waiting for frame requests.");

        if constexpr (IS_DEBUG) {
            // 创建性能统计定时器（每5秒输出一次）
            stats_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&ImagePublisherNode::printStats, this)
            );
        }

        // 持续驱动 OpenCV GUI 事件循环，保持窗口可见和响应
        gui_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            []() { cv::waitKey(1); }
        );

    }

    ~ImagePublisherNode()
    {
        cv::destroyAllWindows();
    }

private:
    void publishFrame()
    {
        std::chrono::high_resolution_clock::time_point start_total, start_read, end_read, start_convert, end_convert, start_publish, end_publish, end_total;
        double read_time = 0.0, convert_time = 0.0, publish_time = 0.0, total_time = 0.0;

        if constexpr (IS_DEBUG) {
            start_total = std::chrono::high_resolution_clock::now();
            // 1. 测量读取帧的时间
            start_read = std::chrono::high_resolution_clock::now();
        }
        cv::Mat frame;
        if (!cap_.read(frame)) {
            if (is_camera_) {
                RCLCPP_WARN(this->get_logger(), "Failed to read from camera, retrying...");
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "Video ended, restarting...");
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);  // 循环播放
                cap_.read(frame);
            }
        }

        if constexpr (IS_DEBUG) {
            end_read = std::chrono::high_resolution_clock::now();
            read_time = std::chrono::duration<double, std::milli>(end_read - start_read).count();
        }

        if (frame.empty()) return;

        // 显示当前待发布帧（窗口由 gui_timer_ 持续刷新）
        cv::imshow("Publishing Frame", frame);

        if constexpr (IS_DEBUG) {
            // 2. 测量图像转换时间
            start_convert = std::chrono::high_resolution_clock::now();
        }
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";
        
        if constexpr (IS_DEBUG) {
            end_convert = std::chrono::high_resolution_clock::now();
            convert_time = std::chrono::duration<double, std::milli>(end_convert - start_convert).count();
            // 3. 测量发布时间
            start_publish = std::chrono::high_resolution_clock::now();
        }
        image_pub_->publish(*msg);
        
        if constexpr (IS_DEBUG) {
            end_publish = std::chrono::high_resolution_clock::now();
            publish_time = std::chrono::duration<double, std::milli>(end_publish - start_publish).count();
            
            end_total = std::chrono::high_resolution_clock::now();
            total_time = std::chrono::duration<double, std::milli>(end_total - start_total).count();
            
            // 累积统计
            frame_count_++;
            total_read_time_ += read_time;
            total_convert_time_ += convert_time;
            total_publish_time_ += publish_time;
            total_callback_time_ += total_time;
        }
    }
    
    void printStats()
    {
        if constexpr (!IS_DEBUG) return;
        if (frame_count_ == 0) return;
        
        double avg_read = total_read_time_ / frame_count_;
        double avg_convert = total_convert_time_ / frame_count_;
        double avg_publish = total_publish_time_ / frame_count_;
        double avg_total = total_callback_time_ / frame_count_;
        
        // 检查订阅者数量
        size_t subscriber_count = image_pub_->get_subscription_count();
        
        RCLCPP_WARN(this->get_logger(), 
            "\n==== Performance Stats (Frames: %ld) ====\n"
            "  Subscribers:      %zu\n"
            "  Avg Read Time:    %.2f ms\n"
            "  Avg Convert Time: %.2f ms\n"
            "  Avg Publish Time: %.2f ms\n"
            "  Avg Total Time:   %.2f ms\n"
            "  Callback FPS:     %.2f Hz\n"
            "======================================",
            frame_count_, subscriber_count,
            avg_read, avg_convert, avg_publish, avg_total,
            1000.0 / avg_total);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr frame_request_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr gui_timer_;
    bool is_camera_;
    int frame_delay_ms_;
    
    // 性能统计变量
    size_t frame_count_;
    double total_read_time_ = 0.0;
    double total_convert_time_ = 0.0;
    double total_publish_time_ = 0.0;
    double total_callback_time_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "buff_detector/config.hpp"
#include "buff_detector/detector.hpp"
#include "buff_interfaces/msg/buff_target.hpp"
#include "buff_interfaces/msg/buff_prediction.hpp"
#include <functional>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/empty.hpp>


class BuffDetectorNode : public rclcpp::Node
{
public:
    BuffDetectorNode() : Node("buff_detector_node")
    {
        // 初始化检测器
        detector_ = std::make_unique<BuffDetector>();

        // 订阅图像
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1));

        // 发布目标信息
        target_pub_ = this->create_publisher<buff_interfaces::msg::BuffTarget>("/buff_target", 10);

        // pull 模式：处理完每帧后发布请求，通知 image_publisher 发下一帧
        frame_request_pub_ = this->create_publisher<std_msgs::msg::Empty>("/frame_request", rclcpp::QoS(10).reliable());
        if (IS_DEBUG) {
            debug_image_pub_ = image_transport::create_publisher(this, "/buff_detector/debug_image");
        }

        // CSV记录
        csv_path_ = "/log/buff_angle_delta.csv";
        csv_.open(csv_path_, std::ios::out);
        if (csv_.is_open())
        {
            csv_ << std::fixed << std::setprecision(9);
            csv_ << "stamp_sec,angle_delta_rad,angle_delta_deg\n";
        }
         // 订阅预测结果
        prediction_sub_ = this->create_subscription<buff_interfaces::msg::BuffPrediction>(
            "/buff_prediction", 10, std::bind(&BuffDetectorNode::predictionCallback, this, std::placeholders::_1)
        );
        
        // 性能统计定时器
        if constexpr (IS_DEBUG) {
            stats_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&BuffDetectorNode::printStats, this)
            );
        }

        // 等待 image_publisher 订阅 /frame_request，确认就绪后发送第一帧请求启动 pipeline
        startup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (frame_request_pub_->get_subscription_count() == 0) {
                    RCLCPP_INFO(this->get_logger(), "Waiting for image_publisher to subscribe /frame_request...");
                    return;
                }
                startup_timer_->cancel();
                frame_request_pub_->publish(std_msgs::msg::Empty{});
                RCLCPP_INFO(this->get_logger(), "Pipeline started: sent initial frame request.");
            }
        );
    }

private:
   void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        //初始化时间统计变量
        std::chrono::high_resolution_clock::time_point start_callback, end_decode;
        double decode_time = 0.0;
        
        //调试模式下记录回调开始时间
        if constexpr (IS_DEBUG) {
            start_callback = std::chrono::high_resolution_clock::now();
        }
        
        // 将ROS图像消息转换为OpenCV格式
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        //调试模式下记录解码结束时间并计算解码时间
        if constexpr (IS_DEBUG) {
            end_decode = std::chrono::high_resolution_clock::now();
            decode_time = std::chrono::duration<double, std::milli>(end_decode - start_callback).count();
        }
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame.");
            return;
        }

        try {
            processFrame(frame, msg->header.stamp, decode_time);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "processFrame threw exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "processFrame threw unknown exception");
        }

        // 处理完毕，通知上游发下一帧（pull 模式，不丢帧）
        frame_request_pub_->publish(std_msgs::msg::Empty{});
    } 

    void processFrame(cv::Mat frame, const rclcpp::Time& frame_stamp, double decode_time = 0.0)
    {
        std::chrono::high_resolution_clock::time_point start_init, end_init, start_update, end_update, start_debug, end_debug, start_pub, end_pub;
        double init_time = 0.0, update_time = 0.0, debug_time = 0.0, pub_time = 0.0;
        
        frame_count_++;

        // 处理第一帧
        if (!is_tracking_)
        {
            if constexpr (IS_DEBUG) {
                start_init = std::chrono::high_resolution_clock::now();
                init_count_++;
            }

            auto frame_clone = frame.clone(); // 克隆帧以避免修改原始帧
            if (detector_->init(frame_clone))
            {
                is_tracking_ = true;
                if constexpr (IS_DEBUG) {
                    RCLCPP_INFO(this->get_logger(), "Detector initialized (attempt #%d).", init_count_);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Detector initialized.");
                }
            }

            if constexpr (IS_DEBUG) {
                end_init = std::chrono::high_resolution_clock::now();
                init_time = std::chrono::duration<double, std::milli>(end_init - start_init).count();
                RCLCPP_INFO(this->get_logger(), "Init time for this frame: %.2f ms", init_time);
            }
        }

        // 更新检测
        if (is_tracking_)
        {
            if constexpr (IS_DEBUG) {
                start_update = std::chrono::high_resolution_clock::now();
            }
            
            if (!detector_->update(frame))
            {
                is_tracking_ = false;
                RCLCPP_WARN(this->get_logger(), "Detector lost target, re-initialization required.");
            }
            
            if constexpr (IS_DEBUG) {
                end_update = std::chrono::high_resolution_clock::now();
                update_time = std::chrono::duration<double, std::milli>(end_update - start_update).count();
            }
        }

        cv::Mat debug_frame;
        if constexpr (IS_DEBUG) {
            start_debug = std::chrono::high_resolution_clock::now();
            // 获取调试图像
            debug_frame = detector_->get_debug_frame();
            end_debug = std::chrono::high_resolution_clock::now();
            debug_time = std::chrono::duration<double, std::milli>(end_debug - start_debug).count();
        }

        // 获取中心点坐标
        cv::Point2f fan_center = detector_->get_first_target_blade_bbox().get_center_2f();
        cv::Point2f r_center = detector_->get_current_R_box().get_center_2f();

        // 发布目标信息
        if constexpr (IS_DEBUG) {
            start_pub = std::chrono::high_resolution_clock::now();
        }
        
        auto target_msg = buff_interfaces::msg::BuffTarget();
        target_msg.header.stamp = frame_stamp;
        target_msg.radius = detector_->get_radius(); // 获取能量机关半径
        target_msg.target_center_x = fan_center.x;
        target_msg.target_center_y = fan_center.y;
        target_msg.r_center_x = r_center.x;
        target_msg.r_center_y = r_center.y;
        target_msg.is_tracking = is_tracking_;
        target_pub_->publish(target_msg);
        
        if constexpr (IS_DEBUG) {
            end_pub = std::chrono::high_resolution_clock::now();
            pub_time = std::chrono::duration<double, std::milli>(end_pub - start_pub).count();
            
            // 累积统计
            total_decode_time_ += decode_time;
            total_init_time_ += init_time;
            total_update_time_ += update_time;
            total_debug_time_ += debug_time;
            total_pub_time_ += pub_time;
            double total_frame_time = decode_time + init_time + update_time + debug_time + pub_time;
            total_frame_time_ += total_frame_time;
            
        }

        // 显示调试信息
        if constexpr (IS_DEBUG)
        {
            // 绘制预测信息
            float vector_x = fan_center.x - r_center.x;
            float vector_y = fan_center.y - r_center.y;
            visualizePrediction(debug_frame, r_center, vector_x, vector_y);
            
            // debug_frame 为空时回退显示原始帧，保证 imshow 始终有效输入
            const cv::Mat& display_frame = debug_frame.empty() ? frame : debug_frame;
            try
            {
                cv::imshow("visualization", display_frame);
            }
            catch (const cv::Exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "imshow failed %s", e.what());
            }
            // 移到 try 外：保证无论 imshow 是否成功都会调用 waitKey
            // 否则 imshow 抛异常时 waitKey 被跳过，导致 OpenCV 事件循环不运行，窗口不显示
            cv::waitKey(1);
            
            // 发布调试图像到 ROS Topic
            if (!debug_frame.empty())
            {
                auto debug_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_frame).toImageMsg();
                debug_img_msg->header.stamp = frame_stamp;
                debug_img_msg->header.frame_id = "camera";
                debug_image_pub_.publish(debug_img_msg);
            }
        }
    }

    void predictionCallback(const buff_interfaces::msg::BuffPrediction::SharedPtr msg)
    {
        //时间戳检查
        static rclcpp::Time last_prediction_time(0, 0, RCL_ROS_TIME);
        rclcpp::Time current_time = msg->header.stamp;
        if (current_time > last_prediction_time)
        {
            current_angle_delta_ = msg->angle_delta;
            has_prediction_ = msg->is_fitted;
            prediction_stamp_ = current_time; // 保存预测时的时间戳
            last_prediction_time = current_time;
            
            // 保存sin参数
            if (has_prediction_) {
                sin_a_ = msg->sin_a;
                sin_omega_ = msg->sin_omega;
                sin_phi_ = msg->sin_phi;
                sin_b_ = msg->sin_b;
                has_sin_params_ = true;
            }

            if (has_prediction_)
            {
                const double t_sec = prediction_stamp_.seconds();
                angle_delta_history_.emplace_back(t_sec, static_cast<double>(current_angle_delta_));
                if (angle_delta_history_.size() > max_history_size_)
                {
                    angle_delta_history_.pop_front();
                }
                //写入csv
                if (csv_.is_open())
                {
                    const double delta_deg = current_angle_delta_ * 180.0 / M_PI;
                    csv_ << t_sec << "," << current_angle_delta_ << "," << delta_deg << "\n";
                }
            }
        }
       
    }

    void printStats()
    {
        if constexpr (!IS_DEBUG) return;
        if (frame_count_ == 0) return;
        
        double avg_decode = total_decode_time_ / frame_count_;
        double avg_init = total_init_time_ / frame_count_;
        double avg_update = total_update_time_ / frame_count_;
        double avg_debug = total_debug_time_ / frame_count_;
        double avg_pub = total_pub_time_ / frame_count_;
        double avg_total = total_frame_time_ / frame_count_;
        double avg_init_per_init = (init_count_ > 0) ? (total_init_time_ / init_count_) : 0.0;
        
        RCLCPP_WARN(this->get_logger(),
            "\n==== Detector Performance (Frames: %d, Tracking: %s) ====\n"
            "  Init Count:       %d times (%.1f%% of frames)\n"
            "  Avg Decode Time:  %.2f ms\n"
            "  Avg Init Time:    %.2f ms (per frame)\n"
            "  Avg Init Time:    %.2f ms (per init only)\n"
            "  Avg Update Time:  %.2f ms\n"
            "  Avg Debug Time:   %.2f ms\n"
            "  Avg Publish Time: %.2f ms\n"
            "  Avg Total Time:   %.2f ms\n"
            "  Processing FPS:   %.2f Hz\n"
            "==========================================",
            frame_count_, is_tracking_ ? "YES" : "NO",
            init_count_, (init_count_ * 100.0 / frame_count_),
            avg_decode, avg_init, avg_init_per_init,
            avg_update, avg_debug, avg_pub, avg_total,
            1000.0 / avg_total);
    }
    
    void visualizePrediction(
        cv::Mat& frame, const cv::Point2f& r_center, float current_vector_x, float current_vector_y
    )
    {
        if constexpr (!IS_DEBUG) return;
        if (frame.empty()) return;
        if (!has_prediction_) return;
        
        // 当前向量的角度
        float current_angle = std::atan2(current_vector_y, current_vector_x);

        // 预测角度 = 当前角度 + 弧度差
        float predicted_angle = current_angle + current_angle_delta_;

        // 当前半径
        float radius = std::sqrt(current_vector_x * current_vector_x + current_vector_y * current_vector_y);

        // 预测的像素坐标
        float pred_x = r_center.x + radius * std::cos(predicted_angle);
        float pred_y = r_center.y + radius * std::sin(predicted_angle);
        cv::Point2f predicted_point(pred_x, pred_y);

        // 绘制预测点（绿色圆圈）
        cv::circle(frame, predicted_point, 8, cv::Scalar(0, 255, 0), 2);

        // 绘制从R框中心到预测点的线
        cv::line(frame, r_center, predicted_point, cv::Scalar(0, 255, 0), 2);

        // 绘制当前点（红色圆圈）
        cv::Point2f current_point(r_center.x + current_vector_x, r_center.y + current_vector_y);
        cv::circle(frame, current_point, 8, cv::Scalar(0, 0, 255), 2);  

        // 显示预测角度差（时间戳 + 度/弧度）
        const double pred_time_sec = prediction_stamp_.seconds();
        std::string text = cv::format(
            "Pred t=%.3f s  Delta: %.1f deg (%.3f rad)",
            pred_time_sec, current_angle_delta_ * 180.0 / M_PI, current_angle_delta_
        );
        cv::putText(frame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // 显示最近N条连续数据
        int line = 0;
        for (auto it = angle_delta_history_.rbegin(); it != angle_delta_history_.rend(); ++it)
        {
            const double t_sec = it->first;
            const double delta_rad = it->second;
            const double delta_deg = delta_rad * 180.0 / M_PI;
            std::string row = cv::format("t=%.3f  d=%.2f deg (%.3f rad)", t_sec, delta_deg, delta_rad);
            cv::putText(
                frame, row, cv::Point(10, 55 + line * 18), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1
            );
            if (++line >= static_cast<int>(max_history_size_))
            {
                break;
            }
        }

        // 显示最新的速度正弦拟合参数
        if (has_sin_params_)
        {
            std::string sin_row = cv::format(
                "sin: a=%.3f  w=%.3f  phi=%.3f  b=%.3f",
                sin_a_, sin_omega_, sin_phi_, sin_b_
            );
            cv::putText(
                frame, sin_row, cv::Point(10, 55 + line * 18), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 0), 1
            );
        }
    }

    // ROS2 通信
    rclcpp::Publisher<buff_interfaces::msg::BuffTarget>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr frame_request_pub_;
    rclcpp::Subscription<buff_interfaces::msg::BuffPrediction>::SharedPtr prediction_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // 检测器
    std::unique_ptr<BuffDetector> detector_;
    image_transport::Publisher debug_image_pub_;

    // 状态
    bool is_tracking_ = false;
    int frame_count_ = 0;

    // 预测信息
    float current_angle_delta_ = 0.0;
    bool has_prediction_ = false;
    rclcpp::Time prediction_stamp_{0, 0, RCL_ROS_TIME}; // 预测时的时间戳
    std::deque<std::pair<double, double>> angle_delta_history_;
    size_t max_history_size_ = 20;
    std::ofstream csv_;
    std::string csv_path_;
    
    // Sin参数
    float sin_a_ = 0.0f;
    float sin_omega_ = 0.0f;
    float sin_phi_ = 0.0f;
    float sin_b_ = 0.0f;
    bool has_sin_params_ = false;
    
    // 性能统计变量
    int init_count_ = 0;
    double total_decode_time_ = 0.0;
    double total_init_time_ = 0.0;
    double total_update_time_ = 0.0;
    double total_debug_time_ = 0.0;
    double total_pub_time_ = 0.0;
    double total_frame_time_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuffDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

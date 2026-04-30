/**
 * @file CamNode.cpp
 * @brief ROS 2 node that wraps a DH camera and publishes image and calibration streams.
 * Copyright (c) 2025 "XJTLU GMaster". All Rights Reserved.
 */

#include <rclcpp/version.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>

#include "CamWrapper.h"
#include "CamWrapperDH.h"
#include "camera_params.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

/**
 * @brief ROS 2 node responsible for streaming camera images and CameraInfo messages.
 */
class CameraPublisher : public rclcpp::Node {
   public:
    /**
     * @brief Construct a publisher node and initialize the underlying camera.
     * @param options Node options for ROS 2 component.
     */
    explicit CameraPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("camera_publisher", options) {
        // declare and initialize the param struct
        CameraParams::declare_on(*this);
        params_ = CameraParams::load(*this);
        if (params_.show_params)
            params_.showConfig(this->get_logger());  // wheter to print the prarms

        // iniitalize the camera
        const int eff_w = params_.sensor.width / params_.binning;
        const int eff_h = params_.sensor.height / params_.binning;
        offx_ = (eff_w - params_.roi.width) / 2;
        offy_ = (eff_h - params_.roi.height) / 2;
        // use reset to relase the old camera if any, then "new" one
        camera_.reset(new DHCamera(params_.device.serial.c_str(), params_.roi.width,
                                   params_.roi.height));  // the SN is here.

        bool started = false;
        for (int i = 0; i < 5; ++i) {
            if (camera_->init(offx_, offy_, params_.roi.width, params_.roi.height, params_.exposure,
                              params_.gain, params_.fps, params_.binning,
                              params_.is_rotate ? params_.rotate_type : -1) &&
                camera_->start()) {
                started = true;
                break;
            }
            RCLCPP_WARN(get_logger(), "Camera not ready, retry %d/5 ...", i + 1);
            std::this_thread::sleep_for(500ms);
        }
        if (!started) {
            RCLCPP_ERROR(get_logger(), "Failed to start camera, exiting.");
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }

        auto qos = rclcpp::SensorDataQoS().keep_last(1);
        qos.deadline(rclcpp::Duration::from_seconds(0.08));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(params_.camera_topic, qos);

        RCLCPP_INFO(get_logger(), "Intra-process comms: %s",
                    this->get_node_options().use_intra_process_comms()
                        ? "ENABLED" : "DISABLED");

        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()
            .transient_local();
        caminfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            params_.camera_info_topic, info_qos);

        // timers
        info_timer_ = this->create_wall_timer(std::chrono::milliseconds(params_.timers.info_ms),
                                              std::bind(&CameraPublisher::publish_cam_info, this));

        // Query tick frequency for hardware timestamp conversion
        tick_freq_ = camera_->tick_frequency();
        if (tick_freq_ > 0) {
            RCLCPP_INFO(get_logger(), "Hardware timestamps enabled, tick_freq=%lu Hz", tick_freq_);
        } else {
            RCLCPP_WARN(get_logger(), "Hardware timestamps unavailable, using system clock");
        }

        // Start a dedicated publisher thread so that SDK callback is never blocked by DDS
        pub_thread_running_ = true;
        pub_thread_ = std::thread(&CameraPublisher::publish_loop, this);

        // SDK callback: lightweight — just move the buffer and wake the publisher thread
        camera_->set_frame_callback([this](std::vector<uint8_t>&& buf, uint64_t hw_ts) {
            {
                std::lock_guard<std::mutex> lk(frame_mutex_);
                pending_frame_ = std::move(buf);
                pending_hw_timestamp_ = hw_ts;
                pending_sys_time_ = this->get_clock()->now();
                frame_ready_ = true;
            }
            frame_cv_.notify_one();
        });

        RCLCPP_INFO(get_logger(),
                    "CameraPublisher ready. serial=%s ROI=%dx%d@off(%d,%d) binning=%d fps=%.2f",
                    params_.device.serial.c_str(), params_.roi.width, params_.roi.height, offx_,
                    offy_, params_.binning, params_.fps);
    }

    /**
     * @brief Stop the camera on shutdown to ensure access to the hardware is released.
     */
    ~CameraPublisher() override {
        // Stop the publisher thread first
        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            pub_thread_running_ = false;
        }
        frame_cv_.notify_one();
        if (pub_thread_.joinable()) pub_thread_.join();

        if (camera_) {
            RCLCPP_INFO(get_logger(), "Stopping camera...");
            camera_->stop();
        }
    }

   private:
    /** Camera configuration loaded from parameters. */
    CameraParams params_;
    /** Owning handle to the active camera implementation. */
    std::unique_ptr<Camera> camera_;
    /** Publisher for image frames. */
    // NOTE: now we use image_transport now, the original imple is the
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    //image_transport::Publisher image_pub_;
    /** Publisher for camera calibration data. */
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_pub_;
    /** Timers that drive CameraInfo publication. */
    rclcpp::TimerBase::SharedPtr info_timer_;
    /** Dedicated publisher thread + synchronization. */
    std::thread pub_thread_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::vector<uint8_t> pending_frame_;
    uint64_t pending_hw_timestamp_{0};
    rclcpp::Time pending_sys_time_;
    bool frame_ready_{false};
    bool pub_thread_running_{false};
    /** Hardware timestamp support. */
    uint64_t tick_freq_{0};
    bool hw_ts_calibrated_{false};
    uint64_t first_hw_ts_{0};
    rclcpp::Time first_sys_time_;
    /** ROI offsets derived from sensor dimensions and binning. */
    int offx_{0}, offy_{0};

   private:
    /**
     * @brief Publisher loop running on a dedicated thread.
     *        Waits for the SDK callback to deposit a frame, then stamps and publishes.
     */
    void publish_loop() {
        const int width = params_.roi.width;
        const int height = params_.roi.height;
        const size_t expected = static_cast<size_t>(height) * width * 3;

        while (true) {
            std::vector<uint8_t> buf;
            uint64_t hw_ts = 0;
            rclcpp::Time sys_time;
            {
                std::unique_lock<std::mutex> lk(frame_mutex_);
                frame_cv_.wait(lk, [this]{ return frame_ready_ || !pub_thread_running_; });
                if (!pub_thread_running_) return;
                buf = std::move(pending_frame_);
                hw_ts = pending_hw_timestamp_;
                sys_time = pending_sys_time_;
                frame_ready_ = false;
            }
            if (buf.size() != expected) continue;

            // Compute accurate image timestamp using hardware clock
            rclcpp::Time stamp;
            if (tick_freq_ > 0 && hw_ts > 0) {
                if (!hw_ts_calibrated_) {
                    // First frame: establish mapping hw_clock → system_clock
                    first_hw_ts_ = hw_ts;
                    first_sys_time_ = sys_time;
                    hw_ts_calibrated_ = true;
                    stamp = sys_time;
                } else {
                    // Subsequent frames: compute elapsed time from hardware delta
                    double hw_elapsed_sec = static_cast<double>(hw_ts - first_hw_ts_)
                                          / static_cast<double>(tick_freq_);
                    stamp = first_sys_time_ + rclcpp::Duration::from_seconds(hw_elapsed_sec);
                }
            } else {
                stamp = sys_time;
            }

            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stamp;
            msg->header.frame_id = params_.frame_id;
            msg->height = static_cast<uint32_t>(height);
            msg->width  = static_cast<uint32_t>(width);
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->step = static_cast<uint32_t>(width * 3);
            msg->data = std::move(buf);
            image_pub_->publish(std::move(msg));
        }
    }

    /**
     * @brief Publish calibration data adjusted for binning and ROI offsets.
     */
    void publish_cam_info() {
        sensor_msgs::msg::CameraInfo info;
        info.header.stamp = this->get_clock()->now();
        info.header.frame_id = params_.frame_id;

        info.width = params_.roi.width;
        info.height = params_.roi.height;
        info.distortion_model = params_.calibration.model;

        // original k from params_
        const auto& k = params_.calibration.k;
        const auto& d = params_.calibration.d;

        // 依据 binning & 中心裁剪调整 fx, fy, cx, cy
        const double b = static_cast<double>(params_.binning);

        const double fx0 = k[0], fy0 = k[4], cx0 = k[2], cy0 = k[5];

        // 先按 binning 缩放，再按中心裁剪偏移
        const double fx = fx0 / b;
        const double fy = fy0 / b;
        const double cx = (cx0 / b) - static_cast<double>(offx_);
        const double cy = (cy0 / b) - static_cast<double>(offy_);

        info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};

        info.d = d;  // copy distortion coeffs

        // R (this is identity matrix here as we do not do stereo rectification)
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        // calcualte the P
        if (params_.calibration.compute_rectified_p && d.size() >= 4) {
            cv::Mat K = (cv::Mat_<double>(3, 3) << info.k[0], info.k[1], info.k[2], info.k[3],
                         info.k[4], info.k[5], info.k[6], info.k[7], info.k[8]);
            cv::Mat D(1, static_cast<int>(d.size()), CV_64F);
            for (int i = 0; i < static_cast<int>(d.size()); ++i) D.at<double>(0, i) = d[i];

            cv::Mat P = cv::getOptimalNewCameraMatrix(
                K, D, cv::Size(params_.roi.width, params_.roi.height), 0.0);

            info.p = {P.at<double>(0, 0), P.at<double>(0, 1), P.at<double>(0, 2),
                      P.at<double>(1, 0), P.at<double>(1, 1), P.at<double>(1, 2),
                      P.at<double>(2, 0), P.at<double>(2, 1), P.at<double>(2, 2)};
        } else {
            // if not compute, just fill in the fx,fy,cx,cy
            info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        }

        // write in the binning factor (note: ROS field semantics may vary by driver; here we
        // directly fill in the factor)
        info.binning_x = params_.binning;
        info.binning_y = params_.binning;

        caminfo_pub_->publish(info);
    }
};

// /**
//  * @brief Entry point that spins the camera publisher node.
//  */
// int main(int argc, char** argv) {
//     // std::system("clear");
//     std::cout << "                                                  \n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡻⣹⣵⡅⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⠜⣼⣟⡿⡏⢬⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠯⣱⡹⡺⣼⠐⡍⡕⢝⢇⢿⣛⣯⣽⣽⣾⣾⣾⣾⣾⣷⣷⣽⢽⣽⣻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢟⣯⢖⢍⣷⢿⣔⡙⠌⡸⡘⡌⢵⡿⣿⣿⡿⡿⣟⣯⣯⣯⣯⣯⣯⣯⣷⢯⣷⡿⣯⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣵⡿⣝⢌⢿⢯⠿⢝⠪⡐⡸⡨⡢⢹⢟⣯⣳⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣯⣟⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⡷⣫⡾⣞⡿⣔⡍⠍⢌⢢⢡⣑⢸⢸⢘⢔⢩⢛⠿⣻⣽⣿⣿⣻⢯⣯⣞⣿⣽⡿⣽⣿⣻⣽⣿⣿⣷⣝⡯⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢟⢮⡾⡯⣯⢯⡟⡇⡱⡬⡢⢅⡕⣌⠪⡪⡪⡸⡸⡸⡸⡸⣹⢿⡹⡴⣻⡝⣾⣟⣟⣾⣿⣳⣿⣿⣿⣿⣿⣻⣿⣮⡯⣿⣿⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠻⡉⣦⢳⡬⣸⡸⡴⡵⡚⢍⡔⣎⢇⡷⡱⡇⡇⡕⣵⣿⣾⣿⡺⡭⣞⢜⣼⠣⡞⣷⡳⣽⣟⢮⣾⣿⣿⣿⣯⢿⣿⣮⣻⣿⡯⣟⣿⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⡿⡕⣵⣫⣷⡏⡆⣿⢽⠹⡨⡼⣱⢽⢣⢳⡱⣿⢱⢕⣾⡿⣿⣳⢗⠵⡟⡅⡷⣱⢝⣞⣗⢝⣗⡏⣷⢿⣿⣻⣗⣿⢽⣿⣿⣷⢽⣿⡻⣟⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣷⣷⣿⡯⣾⣽⣕⢋⠎⢬⢎⠏⣎⣷⡻⡲⡯⡇⢧⣻⣷⣿⢿⠕⢌⢌⠢⡘⡋⡃⣷⡳⣎⢟⡞⡼⣽⣻⣿⡿⣱⣏⣿⣿⣿⢿⡯⣿⣯⢻⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⡪⣸⣣⢾⢸⣾⢟⡥⡓⡛⣨⣟⣿⣿⢯⡓⣸⠏⡋⣭⠉⡘⢦⡅⢝⣇⡿⣵⢫⡷⡯⡷⣫⡟⡼⣯⣿⣿⣿⡿⣽⣿⢸⣿⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣞⣽⢵⣧⢿⣹⢾⣸⣷⠹⡼⡾⣿⡿⡝⣔⠃⢅⠐⡐⡐⣐⠨⣿⣦⢃⡷⣯⣫⢯⣟⡵⣟⡜⡞⡼⣾⣿⣟⣟⢾⣿⢸⣺⣿\n"
//                  "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣟⣯⡷⡿⣫⡯⣟⣵⣿⢯⡗⣿⢿⢸⣸⣟⡿⡽⡣⣷⢽⢢⡡⢂⣢⣦⣵⣿⣿⣷⣿⣿⢎⢷⡫⣵⡷⡹⡜⣎⣿⣿⢯⣗⣿⢿⢸⣳⣿\n"
//                  "⣿⣿⣿⣿⣿⢿⣟⣯⣯⡾⡾⡫⣧⣳⣫⢷⣽⣿⡿⡯⣗⢯⢯⡫⢟⢶⡳⣵⢽⣸⣿⣷⣵⣟⡜⣞⣻⣿⣿⣿⣿⣿⣿⣾⣿⢯⠗⢕⠫⡫⣮⣿⢽⢯⡺⣝⡿⣸⣗⢿\n"
//                  "⣿⣟⣿⣷⢿⣟⢯⢳⢳⢝⢞⣽⢵⣷⣿⣿⣿⠿⠝⣍⣷⣿⠿⢵⢯⣳⣛⣎⣺⢸⣿⣾⢷⣻⣿⣿⡿⡿⢿⢿⣿⣿⣻⢉⢔⣴⣵⣶⢣⡽⡾⡽⣽⢣⠏⣮⡇⣷⢯⢻\n"
//                  "⣿⡻⡽⣺⡽⣪⣺⣼⣾⡿⣟⣷⣿⡿⠏⡇⡇⠣⡿⡿⡽⠊⢌⡐⡔⡜⢜⡷⡺⣙⢿⣿⣿⣿⡿⣫⣾⣿⣯⣗⡝⣿⣿⣷⣧⣎⠙⠕⣯⣽⢾⣏⢇⢧⢯⡷⡱⣫⡏⣾\n"
//                  "⢗⣽⡾⣫⣞⣾⢯⣟⢮⣾⡿⡿⡋⠅⡃⢅⢆⢧⢒⢖⢜⢎⢇⢇⢗⡈⢧⢹⢎⡧⢣⡝⢿⢿⡯⣿⣻⣾⣷⢷⣣⣿⣿⣽⢿⣿⣏⢾⣿⢿⢝⢎⣾⣷⣝⢇⢕⣟⣜⣽\n"
//                  "⣟⢯⣾⢟⣽⢿⡿⣱⣿⡯⣿⣿⣷⣵⢵⠱⣑⠕⡕⢵⢱⡱⡕⣕⢕⢕⠌⡪⠕⡟⣆⠻⣗⢭⣙⠽⣻⢽⣞⣷⣿⣻⣽⣾⣿⠷⣛⣼⢟⢕⣽⣿⣿⣿⣿⡿⡰⣿⣵⣿\n"
//                  "⣺⢽⣳⣿⣳⡟⣵⣿⡯⣿⣿⣿⡿⡏⡵⡱⣾⣟⣞⣦⣂⠅⠣⠣⡣⡳⡱⡈⠕⡕⡜⠕⡑⡴⡹⡫⡦⣗⣮⢮⢭⡭⣭⡧⡷⣸⡿⣱⢝⢼⣿⣿⣿⣿⣿⡧⣹⣿⣿⣿\n"
//                  "⢾⣹⣿⢿⡟⣼⣿⣗⡿⣽⣿⣾⢯⡞⡝⣼⣿⣞⢗⢏⢊⢅⡪⡀⠢⢤⢡⣂⢅⢐⠰⢑⠄⢂⢢⢚⢿⡽⣞⡟⡾⣽⣳⡟⣝⣵⢹⣏⣿⡹⣿⣿⣿⣿⣿⣯⣪⣿⣿⣿\n"
//                  "⢹⢝⣿⣿⢹⣿⣳⢯⣻⣿⣿⢏⣾⢣⢯⠿⠻⢔⢥⣕⡍⡇⣃⢂⠩⡨⣑⠪⡍⡏⡞⣤⢢⢘⢜⢜⢘⣯⢟⢜⡵⣗⣯⣿⣟⣞⣺⣗⣽⣧⢻⣿⣿⣿⣿⣿⣷⣿⣿⣿\n"
//                  "⡘⡌⣪⡗⢽⡇⣯⡯⣷⡟⣽⣮⣯⣬⣴⣼⣜⣔⣕⣔⣄⣆⣆⣇⣅⣬⣢⣇⣥⣢⣬⣤⣇⣧⣧⣧⣢⣧⣳⣓⣟⣟⣟⣿⣝⣖⣟⣧⣫⣟⣯⣻⣿⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⢸⢸⣻⣏⢺⡕⢵⢿⢽⡧⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n"
//                  "⢸⢸⡽⣷⢩⣏⣧⢻⡽⡇⣿⣿⣿⣿⢟⣹⣛⢿⣿⣿⡟⡟⣿⣿⣿⣿⣿⣿⣿⢹⣿⣿⢟⣿⣿⣿⣿⣿⡿⣿⣿⣿⡿⢟⣿⣯⢻⣿⣿⣿⡟⣿⣿⣿⣿⣯⣿⣿⣿⣿\n"
//                  "⢸⢰⢿⣟⣧⠳⣽⡎⣟⢎⣿⣿⣿⣿⡸⠿⢟⢵⡱⠆⡇⡇⣏⠶⣹⣯⣮⣾⣷⢹⡟⠵⡻⣿⣿⣧⣿⣿⡸⡋⡇⣧⣜⣻⣿⣿⢸⣵⣽⣷⢠⠰⣿⣿⣿⣷⣿⣿⣿⣿\n"
//                  "⡘⡎⣿⣟⢎⢧⣓⣿⣎⠣⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n";

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(CameraPublisher)

/**
 * @file CamNode.cpp
 * @brief ROS 2 node that wraps a DH camera and publishes image and calibration streams.
 * Copyright (c) 2025 "XJTLU GMaster". All Rights Reserved.
 */

#include <rclcpp/version.h>

#include <atomic>
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

        // Start a dedicated publisher thread so that SDK callback is never blocked by DDS
        reset_stream_state(0);
        last_reconnect_attempt_time_ = this->get_clock()->now();
        pub_thread_running_ = true;
        pub_thread_ = std::thread(&CameraPublisher::publish_loop, this);

        const bool started = start_camera_with_retries(params_.reconnect.startup_retry_count);
        if (!started) {
            if (!params_.reconnect.enable) {
                RCLCPP_ERROR(get_logger(), "Failed to start camera, exiting.");
                rclcpp::shutdown();
                std::exit(EXIT_FAILURE);
            }
            RCLCPP_ERROR(get_logger(),
                         "Failed to start camera, keep node alive and wait for reconnect.");
        }

        if (params_.reconnect.enable) {
            watchdog_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(params_.reconnect.watchdog_ms),
                std::bind(&CameraPublisher::watchdog, this));
        }

        RCLCPP_INFO(get_logger(),
                    "CameraPublisher ready. serial=%s ROI=%dx%d@off(%d,%d) binning=%d fps=%.2f reconnect=%s",
                    params_.device.serial.c_str(), params_.roi.width, params_.roi.height, offx_,
                    offy_, params_.binning, params_.fps,
                    params_.reconnect.enable ? "enabled" : "disabled");
    }

    /**
     * @brief Stop the camera on shutdown to ensure access to the hardware is released.
     */
    ~CameraPublisher() override {
        shutting_down_ = true;
        if (watchdog_timer_) watchdog_timer_->cancel();
        if (reconnect_thread_.joinable()) reconnect_thread_.join();

        // Stop the publisher thread first
        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            pub_thread_running_ = false;
        }
        frame_cv_.notify_one();
        if (pub_thread_.joinable()) pub_thread_.join();

        std::unique_ptr<Camera> camera_to_stop;
        {
            std::lock_guard<std::mutex> lk(camera_mutex_);
            camera_to_stop = std::move(camera_);
        }
        if (camera_to_stop) {
            RCLCPP_INFO(get_logger(), "Stopping camera...");
            camera_to_stop->stop();
        }
    }

   private:
    /** Camera configuration loaded from parameters. */
    CameraParams params_;
    /** Owning handle to the active camera implementation. */
    std::unique_ptr<Camera> camera_;
    std::mutex camera_mutex_;
    /** Publisher for image frames. */
    // NOTE: now we use image_transport now, the original imple is the
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    //image_transport::Publisher image_pub_;
    /** Publisher for camera calibration data. */
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_pub_;
    /** Timers that drive CameraInfo publication. */
    rclcpp::TimerBase::SharedPtr info_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    /** Dedicated publisher thread + synchronization. */
    std::thread pub_thread_;
    std::thread reconnect_thread_;
    std::atomic_bool reconnecting_{false};
    std::atomic_bool shutting_down_{false};
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::vector<uint8_t> pending_frame_;
    uint64_t pending_hw_timestamp_{0};
    rclcpp::Time pending_sys_time_;
    rclcpp::Time last_frame_time_;
    rclcpp::Time last_reconnect_attempt_time_;
    bool frame_ready_{false};
    bool pub_thread_running_{false};
    bool have_frame_{false};
    /** Hardware timestamp support. */
    uint64_t tick_freq_{0};
    bool hw_ts_calibrated_{false};
    uint64_t first_hw_ts_{0};
    rclcpp::Time first_sys_time_;
    /** ROI offsets derived from sensor dimensions and binning. */
    int offx_{0}, offy_{0};

   private:
    std::unique_ptr<Camera> make_camera() {
        auto camera = std::make_unique<DHCamera>(params_.device.serial.c_str(), params_.roi.width,
                                                 params_.roi.height);
        camera->set_frame_callback(
            [this](std::vector<uint8_t>&& buf, uint64_t hw_ts) {
                handle_frame(std::move(buf), hw_ts);
            });
        return camera;
    }

    bool start_camera_with_retries(int max_attempts) {
        if (max_attempts < 1) max_attempts = 1;

        for (int i = 0; i < max_attempts && !shutting_down_; ++i) {
            auto new_camera = make_camera();
            const bool initialized =
                new_camera->init(offx_, offy_, params_.roi.width, params_.roi.height,
                                  params_.exposure, params_.gain, params_.fps, params_.binning,
                                  params_.is_rotate ? params_.rotate_type : -1);

            if (initialized) {
                const uint64_t new_tick_freq = new_camera->tick_frequency();
                reset_stream_state(new_tick_freq);

                if (new_camera->start()) {
                    {
                        std::lock_guard<std::mutex> lk(camera_mutex_);
                        camera_ = std::move(new_camera);
                    }
                    log_timestamp_mode(new_tick_freq);
                    return true;
                }
            }

            RCLCPP_WARN(get_logger(), "Camera not ready, retry %d/%d ...", i + 1,
                        max_attempts);
            if (i + 1 < max_attempts && !shutting_down_) {
                std::this_thread::sleep_for(500ms);
            }
        }

        return false;
    }

    void handle_frame(std::vector<uint8_t>&& buf, uint64_t hw_ts) {
        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            pending_frame_ = std::move(buf);
            pending_hw_timestamp_ = hw_ts;
            pending_sys_time_ = this->get_clock()->now();
            last_frame_time_ = pending_sys_time_;
            have_frame_ = true;
            frame_ready_ = true;
        }
        frame_cv_.notify_one();
    }

    void reset_stream_state(uint64_t new_tick_freq) {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        pending_frame_.clear();
        pending_hw_timestamp_ = 0;
        pending_sys_time_ = this->get_clock()->now();
        frame_ready_ = false;
        have_frame_ = false;
        last_frame_time_ = pending_sys_time_;
        tick_freq_ = new_tick_freq;
        hw_ts_calibrated_ = false;
        first_hw_ts_ = 0;
        first_sys_time_ = pending_sys_time_;
    }

    void log_timestamp_mode(uint64_t new_tick_freq) {
        if (new_tick_freq > 0) {
            RCLCPP_INFO(get_logger(), "Hardware timestamps enabled, tick_freq=%lu Hz",
                        new_tick_freq);
        } else {
            RCLCPP_WARN(get_logger(), "Hardware timestamps unavailable, using system clock");
        }
    }

    void watchdog() {
        if (!params_.reconnect.enable || shutting_down_) return;

        rclcpp::Time last_frame_time;
        bool have_frame = false;
        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            last_frame_time = last_frame_time_;
            have_frame = have_frame_;
        }

        const auto now = this->get_clock()->now();
        const double frame_elapsed_ms = (now - last_frame_time).seconds() * 1000.0;
        if (frame_elapsed_ms < static_cast<double>(params_.reconnect.frame_timeout_ms)) return;

        const double retry_elapsed_ms =
            (now - last_reconnect_attempt_time_).seconds() * 1000.0;
        if (retry_elapsed_ms < static_cast<double>(params_.reconnect.retry_interval_ms)) return;

        bool expected = false;
        if (!reconnecting_.compare_exchange_strong(expected, true)) return;

        last_reconnect_attempt_time_ = now;
        if (reconnect_thread_.joinable()) reconnect_thread_.join();

        RCLCPP_WARN(get_logger(), "%s for %.0f ms, reconnecting camera...",
                    have_frame ? "No camera frame" : "No camera frame received",
                    frame_elapsed_ms);
        reconnect_thread_ = std::thread(&CameraPublisher::reconnect_camera, this);
    }

    void reconnect_camera() {
        std::unique_ptr<Camera> old_camera;
        {
            std::lock_guard<std::mutex> lk(camera_mutex_);
            old_camera = std::move(camera_);
        }

        if (old_camera) {
            old_camera->stop();
            old_camera.reset();
        }

        reset_stream_state(0);
        const bool started = start_camera_with_retries(1);
        if (started) {
            RCLCPP_INFO(get_logger(), "Camera reconnected.");
        } else if (!shutting_down_) {
            RCLCPP_WARN(get_logger(), "Camera reconnect attempt failed.");
        }

        reconnecting_ = false;
    }

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
            rclcpp::Time stamp;
            {
                std::unique_lock<std::mutex> lk(frame_mutex_);
                frame_cv_.wait(lk, [this]{ return frame_ready_ || !pub_thread_running_; });
                if (!pub_thread_running_) return;
                buf = std::move(pending_frame_);
                hw_ts = pending_hw_timestamp_;
                sys_time = pending_sys_time_;
                frame_ready_ = false;

                // Compute accurate image timestamp using hardware clock.
                if (tick_freq_ > 0 && hw_ts > 0) {
                    if (!hw_ts_calibrated_) {
                        // First frame: establish mapping hw_clock -> system_clock.
                        first_hw_ts_ = hw_ts;
                        first_sys_time_ = sys_time;
                        hw_ts_calibrated_ = true;
                        stamp = sys_time;
                    } else {
                        const double hw_elapsed_sec = static_cast<double>(hw_ts - first_hw_ts_) /
                                                      static_cast<double>(tick_freq_);
                        stamp = first_sys_time_ + rclcpp::Duration::from_seconds(hw_elapsed_sec);
                    }
                } else {
                    stamp = sys_time;
                }
            }
            if (buf.size() != expected) continue;

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

            info.p = {P.at<double>(0, 0), P.at<double>(0, 1), P.at<double>(0, 2), 0.0,
                      P.at<double>(1, 0), P.at<double>(1, 1), P.at<double>(1, 2), 0.0,
                      P.at<double>(2, 0), P.at<double>(2, 1), P.at<double>(2, 2), 0.0};
        } else {
            // if not compute, just fill in the fx,fy,cx,cy
            info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        }

        // K/P already describe the published ROI-cropped and binned image. Keep the metadata
        // neutral so camera-model consumers do not apply those adjustments a second time.
        info.binning_x = 1;
        info.binning_y = 1;
        info.roi.x_offset = 0;
        info.roi.y_offset = 0;
        info.roi.width = 0;
        info.roi.height = 0;
        info.roi.do_rectify = false;

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

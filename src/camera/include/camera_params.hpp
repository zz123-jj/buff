/**
 * @file camera_params.hpp
 * @brief Parameter container and helpers for configuring the camera publisher node.
 */

#pragma once

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @brief Strongly-typed wrapper around the ROS parameter set controlling camera behavior.
 */
struct CameraParams {
    /** Toggle runtime rotation of captured images. */
    bool is_rotate = true;
    /** OpenCV rotate flag determining how the image is rotated. */
    int rotate_type = 1;  // 0:90cw, 1:180, 2:90ccw（按 OpenCV RotateFlags）
    /** Print the configuration once the node boots. */
    bool show_params = true;

    /** Hardware identifier information. */
    struct Device {
        /** Camera serial number targeted at initialization. */
        std::string serial = "FGV22100004";
    } device;
    /** Raw sensor dimensions before binning. */
    struct Sensor {
        int width = 1280;
        int height = 1024;
    } sensor;
    /** Region of interest dimensions after binning. */
    struct ROI {
        int width = 1024;
        int height = 768;
    } roi;

    /** Sensor binning factor applied in both axes. */
    int binning = 1;
    /** Target acquisition frame rate. */
    double fps = 100.0;
    /** Exposure time in microseconds. */
    int exposure = 1500;
    /** Analog gain value. */
    int gain = 16;
    /** Topic used to publish `sensor_msgs::Image`. */
    std::string camera_topic = "image_raw";
    /** Frame id assigned to outgoing messages. */
    std::string frame_id = "camera_optical_frame";

    /** Topic used to publish `sensor_msgs::CameraInfo`. */
    std::string camera_info_topic = "camera_info";

    /** Period configuration (milliseconds) for publishing timers. */
    struct Timers {
        int info_ms = 10;
    } timers;

    /** Camera disconnect watchdog and reconnect behavior. */
    struct Reconnect {
        bool enable = true;
        int frame_timeout_ms = 500;
        int retry_interval_ms = 1000;
        int startup_retry_count = 5;
        int watchdog_ms = 200;
    } reconnect;

    /** Intrinsic calibration data and configuration flags. */
    struct Calibration {
        std::vector<double> k = {1557.2, 0.2065, 638.7311, 0.0, 1557.5, 515.1176, 0.0, 0.0, 1.0};
        std::vector<double> d = {-0.1295, 0.0804, 0.000485, 0.000637, 0.2375};
        std::string model = "plumb_bob";
        bool compute_rectified_p = true;
    } calibration;

    /**
     * @brief Declare all parameters on the provided ROS node.
     * @param node Node that owns the parameters.
     */
    static void declare_on(rclcpp::Node& node) {
        node.declare_parameter<bool>("is_rotate", true);
        node.declare_parameter<int>("rotate_type", 1);
        node.declare_parameter<bool>("show_params", true);

        node.declare_parameter<std::string>("device.serial", "FGV22100004");
        node.declare_parameter<int>("sensor.width", 1280);
        node.declare_parameter<int>("sensor.height", 1024);
        node.declare_parameter<int>("roi.width", 1024);
        node.declare_parameter<int>("roi.height", 768);

        node.declare_parameter<int>("binning", 1);
        node.declare_parameter<double>("fps", 100.0);
        node.declare_parameter<int>("exposure", 1500);
        node.declare_parameter<int>("gain", 16);
        node.declare_parameter<std::string>("camera_topic", "image_raw");
        node.declare_parameter<std::string>("frame_id", "camera_optical_frame");

        node.declare_parameter<std::string>("camera_info_topic", "camera_info");

        node.declare_parameter<int>("timers.info_ms", 10);

        node.declare_parameter<bool>("reconnect.enable", true);
        node.declare_parameter<int>("reconnect.frame_timeout_ms", 500);
        node.declare_parameter<int>("reconnect.retry_interval_ms", 1000);
        node.declare_parameter<int>("reconnect.startup_retry_count", 5);
        node.declare_parameter<int>("reconnect.watchdog_ms", 200);

        node.declare_parameter<std::vector<double>>(
            "calibration.k", {1557.2, 0.2065, 638.7311, 0.0, 1557.5, 515.1176, 0.0, 0.0, 1.0});
        node.declare_parameter<std::vector<double>>("calibration.d",
                                                    {-0.1295, 0.0804, 0.000485, 0.000637, 0.2375});
        node.declare_parameter<std::string>("calibration.model", "plumb_bob");
        node.declare_parameter<bool>("calibration.compute_rectified_p", true);
    }

    /**
     * @brief Populate a `CameraParams` instance from the node's parameter store.
     * @param node Node from which parameters are read.
     * @return Fully initialized parameter structure.
     */
    static CameraParams load(rclcpp::Node& node) {
        CameraParams p;
        p.is_rotate = node.get_parameter("is_rotate").as_bool();
        p.rotate_type = node.get_parameter("rotate_type").as_int();
        p.show_params = node.get_parameter("show_params").as_bool();

        node.get_parameter("device.serial", p.device.serial);
        p.sensor.width = node.get_parameter("sensor.width").as_int();
        p.sensor.height = node.get_parameter("sensor.height").as_int();
        p.roi.width = node.get_parameter("roi.width").as_int();
        p.roi.height = node.get_parameter("roi.height").as_int();

        p.binning = node.get_parameter("binning").as_int();
        p.fps = node.get_parameter("fps").as_double();
        p.exposure = node.get_parameter("exposure").as_int();
        p.gain = node.get_parameter("gain").as_int();
        node.get_parameter("camera_topic", p.camera_topic);
        node.get_parameter("frame_id", p.frame_id);

        node.get_parameter("camera_info_topic", p.camera_info_topic);

        p.timers.info_ms = node.get_parameter("timers.info_ms").as_int();

        p.reconnect.enable = node.get_parameter("reconnect.enable").as_bool();
        p.reconnect.frame_timeout_ms = node.get_parameter("reconnect.frame_timeout_ms").as_int();
        p.reconnect.retry_interval_ms = node.get_parameter("reconnect.retry_interval_ms").as_int();
        p.reconnect.startup_retry_count =
            node.get_parameter("reconnect.startup_retry_count").as_int();
        p.reconnect.watchdog_ms = node.get_parameter("reconnect.watchdog_ms").as_int();

        node.get_parameter("calibration.k", p.calibration.k);
        node.get_parameter("calibration.d", p.calibration.d);
        node.get_parameter("calibration.model", p.calibration.model);
        p.calibration.compute_rectified_p =
            node.get_parameter("calibration.compute_rectified_p").as_bool();

        p.validate(node);
        return p;
    }

    /**
     * @brief Log the current configuration in a human-friendly layout.
     * @param logger Logger instance used for output.
     */
    void showConfig(const rclcpp::Logger& logger) const {
        std::ostringstream oss;
        oss << "CameraParams {\n";
        oss << "  is_rotate: " << std::boolalpha << is_rotate << "\n";
        oss << "  rotate_type: " << rotate_type << "\n";
        oss << "  device.serial: " << device.serial << "\n";
        oss << "  sensor: { width: " << sensor.width << ", height: " << sensor.height << " }\n";
        oss << "  roi: { width: " << roi.width << ", height: " << roi.height << " }\n";
        oss << "  binning: " << binning << "\n";
        oss << "  fps: " << fps << "\n";
        oss << "  exposure: " << exposure << "\n";
        oss << "  gain: " << gain << "\n";

        oss << "  camera_topic: " << camera_topic << "\n";
        oss << "  camera_info_topic: " << camera_info_topic << "\n";
        oss << "  frame_id: " << frame_id << "\n";

        oss << "  timers: { info_ms: " << timers.info_ms << " }\n";
        oss << "  reconnect: { enable: " << reconnect.enable
            << ", frame_timeout_ms: " << reconnect.frame_timeout_ms
            << ", retry_interval_ms: " << reconnect.retry_interval_ms
            << ", startup_retry_count: " << reconnect.startup_retry_count
            << ", watchdog_ms: " << reconnect.watchdog_ms << " }\n";

        oss << "  calibration.k: " << format_vector(calibration.k) << "\n";
        oss << "  calibration.d: " << format_vector(calibration.d) << "\n";
        oss << "  calibration.model: " << calibration.model << "\n";
        oss << "  calibration.compute_rectified_p: " << calibration.compute_rectified_p << "\n";
        oss << "}";
        RCLCPP_INFO(logger, "%s", oss.str().c_str());
    }

    /**
     * @brief Convert textual reliability names into ROS QoS policies.
     * @param s Input string (case insensitive).
     * @return Matching QoS reliability enum.
     */
    /**
     * @brief Translate the rotation type into OpenCV rotate flags.
     * @return One of the `cv::RotateFlags` values accepted by `cv::rotate`.
     */
    int cv_rotate_flag() const {
        switch (rotate_type) {
            case 0:
                return 0;  // ROTATE_90_CLOCKWISE
            case 1:
                return 1;  // ROTATE_180
            case 2:
                return 2;  // ROTATE_90_COUNTERCLOCKWISE
            default:
                return 1;
        }
    }

    /**
     * @brief Sanity-check and adjust parameters received from the ROS layer.
     * @param node Node used to emit warnings via its logger.
     */
    void validate(rclcpp::Node& node) {
        auto clamp_pos = [](int& v, int lo, int hi, const char* name, rclcpp::Logger lg) {
            if (v < lo) {
                RCLCPP_WARN(lg, "%s < %d, clamp to %d", name, lo, lo);
                v = lo;
            }
            if (hi > 0 && v > hi) {
                RCLCPP_WARN(lg, "%s > %d, clamp to %d", name, hi, hi);
                v = hi;
            }
        };
        auto lg = node.get_logger();

        clamp_pos(sensor.width, 1, 16384, "sensor.width", lg);
        clamp_pos(sensor.height, 1, 16384, "sensor.height", lg);

        if (binning < 1) {
            RCLCPP_WARN(lg, "binning < 1, set to 1");
            binning = 1;
        }
        // ROI 的上限应基于 binning 之后的有效尺寸
        {
            const int eff_w = sensor.width / std::max(1, binning);
            const int eff_h = sensor.height / std::max(1, binning);
            clamp_pos(roi.width, 1, eff_w, "roi.width", lg);
            clamp_pos(roi.height, 1, eff_h, "roi.height", lg);
        }

        if (fps <= 0.0) {
            RCLCPP_WARN(lg, "fps <= 0, set to 1.0");
            fps = 1.0;
        }

        if (reconnect.frame_timeout_ms < 100) {
            RCLCPP_WARN(lg, "reconnect.frame_timeout_ms < 100, set to 100");
            reconnect.frame_timeout_ms = 100;
        }
        if (reconnect.retry_interval_ms < 100) {
            RCLCPP_WARN(lg, "reconnect.retry_interval_ms < 100, set to 100");
            reconnect.retry_interval_ms = 100;
        }
        if (reconnect.startup_retry_count < 1) {
            RCLCPP_WARN(lg, "reconnect.startup_retry_count < 1, set to 1");
            reconnect.startup_retry_count = 1;
        }
        if (reconnect.watchdog_ms < 50) {
            RCLCPP_WARN(lg, "reconnect.watchdog_ms < 50, set to 50");
            reconnect.watchdog_ms = 50;
        }

        if (calibration.k.size() != 9) {
            RCLCPP_WARN(lg, "calibration.k size != 9, reset to identity");
            calibration.k = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        }
        if (calibration.d.size() < 4) {
            RCLCPP_WARN(lg, "calibration.d size < 4, pad zeros");
            calibration.d.resize(4, 0.0);
        }
    }

   private:
    template <typename T>
    static std::string format_vector(const std::vector<T>& data) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < data.size(); ++i) {
            if (i > 0) oss << ", ";
            oss << data[i];
        }
        oss << "]";
        return oss.str();
    }

    static std::string lower(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return s;
    }
};

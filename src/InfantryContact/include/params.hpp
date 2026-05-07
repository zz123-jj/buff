#pragma once

#include <pid.h>
#include <unistd.h>

#include <filesystem>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

struct Params {
    // data start
    struct Pid {
        double kp;
        double ki;
        double kd;
        double i_max;
        double i_min;
    };

    struct Filter {
        double yaw_cutoff;
    };

    struct Rx {
        int rx_timerwall_ms = 10;
        bool verbose = true;
    };

    struct Tx {
        bool verbose = true;
    };

    struct PortConf {
        std::string port_name = "/dev/ttyACM0";
        uint32_t baudrate = 115200;
    };

    Pid pid_yaw;
    Pid pid_pitch;
    Filter filter;
    Rx rx;
    Tx tx;
    PortConf port;

    Params(rclcpp::Node& node) {
        declare_on(node);
        *this = load(node);
    }

    Params() = default;

    void declare_on(rclcpp::Node& node) {
        // Launch file marker - must be set by launch file
        node.declare_parameter<bool>("launched_properly", false);

        // pid for yaw
        node.declare_parameter<double>("pid.yaw.kp", 0.7);
        node.declare_parameter<double>("pid.yaw.ki", 0.08);
        node.declare_parameter<double>("pid.yaw.kd", 1.5);
        node.declare_parameter<double>("pid.yaw.i_max", 10.0);
        node.declare_parameter<double>("pid.yaw.i_min", -10.0);

        // pid for pitch
        node.declare_parameter<double>("pid.pitch.kp", 0.35);
        node.declare_parameter<double>("pid.pitch.ki", 0.03);
        node.declare_parameter<double>("pid.pitch.kd", 0.0);
        node.declare_parameter<double>("pid.pitch.i_max", 10.0);
        node.declare_parameter<double>("pid.pitch.i_min", -10.0);

        // filter set
        node.declare_parameter<double>("filter.yaw_cutoff", 10020.0);

        // Rx set
        node.declare_parameter<int>("rx.timerwall_ms", 10);
        node.declare_parameter<bool>("rx.verbose", true);

        // Tx set
        node.declare_parameter<bool>("tx.verbose", true);

        // port set
        node.declare_parameter<std::string>("port.name", "/dev/ttyACM0");
        // WARNING: this cast may lead bug as it convert the uint32_t to int
        node.declare_parameter<int>("port.baudrate", 115200);
    }

    static Params load(rclcpp::Node& node) {
        Params p;

        // Check if launched properly via launch file
        bool launched_properly = node.get_parameter("launched_properly").as_bool();
        if (!launched_properly) {
            RCLCPP_FATAL(node.get_logger(),
                         "\n\n"  // clang-format off
                         "╔══════════════════════════════════════════════════════════════════════════════════╗\n"
                         "║                                FATAL ERROR                                       ║\n"
                         "║                                                                                  ║\n"
                         "║  This node MUST be launched with 'ros2 launch', NOT 'ros2 run'!                  ║\n"
                         "║                                                                                  ║\n"
                         "║  Reason: PID parameters and filter configuration are required.                   ║\n"
                         "║                                                                                  ║\n"
                         "║  Correct usage:                                                                  ║\n"
                         "║    ros2 launch contact contact_node.launch.py                                    ║\n"
                         "║                                                                                  ║\n"
                         "║  Using 'ros2 run' will result in VERY LAGGY control due to:                      ║\n"
                         "║    - PID gains = 0 (no control if you have enabled pid in the ContactNode's code)║\n"
                         "║    - Filter cutoff = 1Hz (blocks fast response)                                  ║\n"
                         "╚══════════════════════════════════════════════════════════════════════════════════╝\n");
            rclcpp::shutdown();  // clang-format on
            exit(EXIT_FAILURE);
        }

        // pid for yaw
        p.pid_yaw.kp = node.get_parameter("pid.yaw.kp").as_double();
        p.pid_yaw.ki = node.get_parameter("pid.yaw.ki").as_double();
        p.pid_yaw.kd = node.get_parameter("pid.yaw.kd").as_double();
        p.pid_yaw.i_max = node.get_parameter("pid.yaw.i_max").as_double();
        p.pid_yaw.i_min = node.get_parameter("pid.yaw.i_min").as_double();

        // pid for pitch
        p.pid_pitch.kp = node.get_parameter("pid.pitch.kp").as_double();
        p.pid_pitch.ki = node.get_parameter("pid.pitch.ki").as_double();
        p.pid_pitch.kd = node.get_parameter("pid.pitch.kd").as_double();
        p.pid_pitch.i_max = node.get_parameter("pid.pitch.i_max").as_double();
        p.pid_pitch.i_min = node.get_parameter("pid.pitch.i_min").as_double();

        // filter
        p.filter.yaw_cutoff = node.get_parameter("filter.yaw_cutoff").as_double();

        // Rx
        p.rx.rx_timerwall_ms = node.get_parameter("rx.timerwall_ms").as_int();
        p.rx.verbose = node.get_parameter("rx.verbose").as_bool();

        // Tx
        p.tx.verbose = node.get_parameter("tx.verbose").as_bool();

        // port
        p.port.port_name = node.get_parameter("port.name").as_string();
        p.port.baudrate = static_cast<uint32_t>(node.get_parameter("port.baudrate").as_int());
        // check if the port existed
        if (std::filesystem::exists(p.port.port_name) == false) {
            RCLCPP_FATAL(node.get_logger(),
                         "port name %s not existed, checking available ports now",
                         p.port.port_name.c_str());
            for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
                std::string path_str = entry.path().string();
                if (path_str.find("ttyACM") != std::string::npos) {
                    p.port.port_name = path_str;
                    RCLCPP_WARN(node.get_logger(), "Using first available ttyACM port: %s",
                                p.port.port_name.c_str());
                    break;
                }
            }
        }

        // change port permissions
        std::string chmod_cmd = "chmod 777 " + p.port.port_name;
        int result = system(chmod_cmd.c_str());
        if (result == 0) {
            RCLCPP_INFO(node.get_logger(), "Successfully set permissions for %s",
                        p.port.port_name.c_str());
        } else {
            RCLCPP_WARN(node.get_logger(), "Failed to set permissions for %s, you may need sudo",
                        p.port.port_name.c_str());
        }

        std::cout << "========== README ==========" << std::endl;
        std::cerr << "1. Run this node using `ros2 launch` only (not `ros2 run`)." << std::endl;
        std::cout << "2. Enable/disable PID by editing the pid_midware section in ContactNode."
                  << std::endl;
        std::cout << "3. Yaw cutoff: If yaw angle exceeds the cutoff, it is deprecated."
                  << std::endl;
        std::cout << "4. RX msgs: Received from C board (lower machine) via timer callback."
                  << std::endl;
        std::cout << "   Higher frequency = more precise angle for autoaim node." << std::endl;
        std::cout << "5. TX msgs: Sent to lower machine via USB (/dev/ttyACM0) immediately on "
                     "/autoaim/msg subscription."
                  << std::endl;
        std::cout << "   No need to worry about delay." << std::endl;
        std::cout << "6. Port: Configure in config/contact_node.yaml. Ensure baudrate and portname "
                     "are correct."
                  << std::endl;
        std::cout << "============================" << std::endl;

        std::cout << "\n========== Yaw Filter ==========" << std::endl;
        std::cout << "Cutoff boundary: " << p.filter.yaw_cutoff << std::endl;
        std::cout << "================================" << std::endl;

        std::cout << "\n========== RX Timerwall ==========" << std::endl;
        std::cout << "Interval (ms): " << p.rx.rx_timerwall_ms << std::endl;
        std::cout << "==================================" << std::endl;

        std::cout << "\n========== TX Log ==========" << std::endl;
        std::cout << "Verbose: " << (p.tx.verbose ? "Enabled" : "Disabled") << std::endl;
        std::cout << "============================" << std::endl;

        std::cout << "\n========== PID Config ==========" << std::endl;
        std::cout << "Check ContactNode code to confirm PID midware is enabled." << std::endl;

        std::cout << "\n--- PID Pitch ---" << std::endl;
        std::cout << "P: " << p.pid_pitch.kp << "  I: " << p.pid_pitch.ki
                  << "  D: " << p.pid_pitch.kd << std::endl;
        std::cout << "I_max: " << p.pid_pitch.i_max << "  I_min: " << p.pid_pitch.i_min
                  << std::endl;

        std::cout << "\n--- PID Yaw ---" << std::endl;
        std::cout << "P: " << p.pid_yaw.kp << "  I: " << p.pid_yaw.ki << "  D: " << p.pid_yaw.kd
                  << std::endl;
        std::cout << "I_max: " << p.pid_yaw.i_max << "  I_min: " << p.pid_yaw.i_min << std::endl;
        std::cout << "===============================" << std::endl;

        std::cout << "\n========== Port Config ==========" << std::endl;
        std::cout << "Port name: " << p.port.port_name << std::endl;
        std::cout << "Baudrate: " << p.port.baudrate << std::endl;
        std::cout << "=================================" << std::endl;
        return p;
    }

    void description(rclcpp::Node& node) {}

    pid_config fillPidYawConfig() {
        pid_config conf{};
        conf.PID_KP = pid_yaw.kp;
        conf.PID_KI = pid_yaw.ki;
        conf.PID_KD = pid_yaw.kd;
        conf.I_max = pid_yaw.i_max;
        conf.I_min = pid_yaw.i_min;
        return conf;
    }

    pid_config fillPidPitchConfig() {
        pid_config conf{};
        conf.PID_KP = pid_pitch.kp;
        conf.PID_KI = pid_pitch.ki;
        conf.PID_KD = pid_pitch.kd;
        conf.I_max = pid_pitch.i_max;
        conf.I_min = pid_pitch.i_min;
        return conf;
    }
};

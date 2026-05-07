#pragma once

#include <Content.hpp>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "RxMsg.hpp"

#define RAD 57.2957795f
#define PI 3.141592653589793

class TxMsg {
   public:
    TxMsg() { memset(&tx_content_, 0, sizeof(TxContent)); }

    using midware_plain_cb_t = std::function<void(float*, float*)>;
    using midware_state_cb_t = std::function<void(float*, float*, RxMsg&)>;
    using comm_port_cb_t = std::function<void(uint8_t* buffer, size_t size, bool safe_write)>;

    TxContent tx_content_;
    size_t tx_struct_len = sizeof(TxContent);

    void inline toBuffer(uint8_t* buffer) { std::memcpy(buffer, &tx_content_, tx_struct_len); }

    void autoaim_callback(
        TxContent::autoAim_t msg, RxMsg& rx_msg, rclcpp::Node& node,
        const std::function<void(uint8_t*, size_t, bool)>& comm_port_cb,
        const midware_plain_cb_t& midware_unit_convert_cb =
            [](float* pitch, float* yaw) {
                *pitch = *pitch * RAD;
                *yaw = *yaw * RAD;
            },
        const midware_state_cb_t& midware_pid_cb =
            [](float* pitch, float* yaw, RxMsg& rx) {
                (void)pitch;
                (void)yaw;
                (void)rx;
            },
        const midware_state_cb_t& midware_filter_cb =
            [](float* pitch, float* yaw, RxMsg& rx) {
                (void)pitch;
                (void)yaw;
                (void)rx;
            },
        const midware_state_cb_t& midware_boundary_angle_cb =
            [](float* pitch, float* yaw, RxMsg& rx) {
                (void)pitch;
                (void)yaw;
                (void)rx;
            },
        bool verbose = true) {
        this->tx_content_.header = 0x3A;
        if (rx_msg.isInvalid()) return;

        // convert the msg to angle
        midware_unit_convert_cb(&msg.pitch, &msg.yaw);

        float pitch = msg.pitch;
        float yaw = msg.yaw;

        midware_pid_cb(&pitch, &yaw, rx_msg);
        midware_filter_cb(&pitch, &yaw, rx_msg);
        // pitch += rx_msg.rx_content_.pitch;
        // yaw += rx_msg.rx_content_.yaw;

        midware_boundary_angle_cb(&pitch, &yaw, rx_msg);

        this->tx_content_.pitch = pitch;
        this->tx_content_.yaw = yaw;

        this->tx_content_.found = msg.target_distance ? 1 : 0;
        this->tx_content_.shoot_or_not =
            msg.shoot_command == RxContent::autoaim_t::ALLOW_SHOOT ? 1 : 0;

        this->tx_content_.single_shoot_flag = msg.shoot_mode;

        uint8_t tx_buffer[sizeof(TxContent)]{};
        toBuffer(tx_buffer);

        comm_port_cb(tx_buffer, tx_struct_len, true);

        if (verbose) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2);
            oss << "[TxMsg] pitch: " << this->tx_content_.pitch << " yaw: " << this->tx_content_.yaw
                << " found: " << static_cast<int>(this->tx_content_.found)
                << " shoot: " << static_cast<int>(this->tx_content_.shoot_or_not) << " header: 0x"
                << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(this->tx_content_.header) << std::dec << std::setfill(' ')
                << " pitch:" << std::setw(7) << this->tx_content_.pitch << " yaw:" << std::setw(7)
                << this->tx_content_.yaw << " found:" << static_cast<int>(this->tx_content_.found)
                << " shoot:" << static_cast<int>(this->tx_content_.shoot_or_not)
                << " single_shoot?: " << static_cast<int>(this->tx_content_.single_shoot_flag);
            RCLCPP_INFO(node.get_logger(), "%s", oss.str().c_str());
        }
    }

   private:
    rclcpp::Time update_time_{0};
};

#pragma once

#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
// the ros msg lot
#include <gary_msgs/msg/auto_aim.hpp>
#include <gary_msgs/msg/detail/shoot_data__struct.hpp>
#include <gary_msgs/msg/shoot_data.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
//

struct RxContent {
    using header_t = int;
    uint8_t header;  // 0xA3 for InfantryDL

    // ====================================
    using autoaim_t = gary_msgs::msg::AutoAIM;
    using autoaim_pub_t = rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr;
    float roll;
    float pitch;
    float yaw;
    // 13

    // ====================================
    using quat_pub_t = rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr;
    using jointAngle_t = sensor_msgs::msg::JointState;
    using quat_joint_pub_t = rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr;
    float q[4];
    // 29

    // ====================================
    using color_t = std_msgs::msg::Int32;
    using color_pub_t = rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr;
    uint8_t color;
    // 30

    // ====================================
    using autoaim_mode_t = std_msgs::msg::Int32;
    using autoaim_mode_pub_t = rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr;
    uint8_t auto_aim_mode;
    // 31

    // ====================================
    using autoaim_shoot_decision_t = std_msgs::msg::Int32;
    using autoaim_shoot_decision_pub_t = rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr;
    uint8_t shoot_decision;
    // 32

    // ====================================
    using shoot_data_t = gary_msgs::msg::ShootData;
    using shoot_data_pub_t = rclcpp::Publisher<gary_msgs::msg::ShootData>::SharedPtr;
    uint16_t bullet_spped;
    // 34

    uint8_t toogle_target_key_pressed = 0;
    // 35
    //

    // ====================================
    uint8_t EOF_;  // 0xAA for InfantryDL // EOF_ not EOF (variable name)
    // 36

    bool operator==(const RxContent& other) const {
        return std::memcmp(this, &other, sizeof(RxContent)) == 0;
    }
} __attribute__((packed));

struct TxContent {
    using autoAim_t = gary_msgs::msg::AutoAIM;
    using autoaim_sub_t = rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr;

    // ====================================
    using header_t = int;
    uint8_t header;  // 0xA3 for InfantryDL

    // ====================================
    using jointAngle_t = sensor_msgs::msg::JointState;
    float pitch;
    float yaw;

    // ====================================
    uint8_t found;
    uint8_t shoot_or_not;
    uint8_t single_shoot_flag;

    uint8_t done_fitting;

    // ====================================
    uint8_t patrolling;  // 0xAA for InfantryDL // EOF_ not EOF (variable name)

    // ====================================
    uint8_t is_updated;

    // ====================================
    uint8_t checksum;

    bool operator==(TxContent const& other) {
        return std::memcmp(this, &other, sizeof(TxContent)) == 0;
    }
} __attribute__((packed));

#pragma once

#include <Content.hpp>
#include <mutex>
#include <rclcpp/clock.hpp>

class RxMsg {
   public:
    RxMsg() { memset(&rx_content_, 0, sizeof(RxContent)); }

    RxContent rx_content_;
    uint8_t rx_buffer_[sizeof(RxContent)]{};

    RxContent parseBuffer(const uint8_t* buffer) {
        RxContent content{};
        std::memcpy(&content, buffer, sizeof(content));
        return content;
    }

    void updateFromBuffer(const uint8_t* buffer) {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        std::memcpy(rx_buffer_, buffer, sizeof(RxContent));
        rx_content_ = parseBuffer(rx_buffer_);
    }

    RxContent snapshot() const {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        return rx_content_;
    }

    constexpr inline float convertUintToFloat(int x_int, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    [[nodiscard]]
    bool loadMsgAndPublish(RxContent::autoaim_pub_t& _autoaim_pub_,                          //
                           RxContent::quat_pub_t& _quat_pub_,                                //
                           RxContent::quat_joint_pub_t& _joint_state_pub_,                   //
                           RxContent::color_pub_t& _color_pub_,                              //
                           RxContent::autoaim_mode_pub_t& _autoaim_mode_pub_,                //
                           RxContent::autoaim_shoot_decision_pub_t& _autoaim_decision_pub_,  //
                           RxContent::shoot_data_pub_t& _shoot_data_pub_,                    //
                           rclcpp::Node& node,                                               //
                           bool verbose = true

    ) {
        const RxContent content = snapshot();

        // Example: Publish AutoAIM message
        RxContent::autoaim_t autoaim_msg;
        autoaim_msg.pitch = content.pitch;
        autoaim_msg.yaw = content.yaw;
        _autoaim_pub_->publish(
            std::move(autoaim_msg));  // NOTE: Idk whether the std::move is necessary

        // Publish JointState message
        sensor_msgs::msg::JointState joint_state_msg;
        fillJointStateMsg(joint_state_msg, node, content);
        _joint_state_pub_->publish(std::move(joint_state_msg));

        // Publish Quaternion message
        geometry_msgs::msg::QuaternionStamped quat_msg;
        fillQuatMsg(quat_msg, content);
        _quat_pub_->publish(std::move(quat_msg));

        // Publish Color message
        std_msgs::msg::Int32 color_msg;
        color_msg.data = content.color;
        _color_pub_->publish(std::move(color_msg));

        // Publish AutoAIM Mode message
        std_msgs::msg::Int32 autoaim_mode_msg;
        // autoaim_mode_msg.data = rx_content_.autoaim_mode;
        autoaim_mode_msg.data = content.auto_aim_mode;
        _autoaim_mode_pub_->publish(std::move(autoaim_mode_msg));

        // Publish AutoAIM Decision message
        std_msgs::msg::Int32 autoaim_shoot_decision_msg;
        autoaim_shoot_decision_msg.data = content.shoot_decision;
        _autoaim_decision_pub_->publish(std::move(autoaim_shoot_decision_msg));

        // Publish ShootData message
        gary_msgs::msg::ShootData shoot_data_msg;
        shoot_data_msg.bullet_speed = convertUintToFloat(content.bullet_spped, 15, 30, 16);
        _shoot_data_pub_->publish(std::move(shoot_data_msg));

        // autoaim target toogle logic
        static uint8_t last_toogle_target_pressed = 0;
        bool toogle_target = last_toogle_target_pressed == 1u
                                 ? (content.toogle_target_key_pressed == 0u ? 1 : 0)
                                 : 0;
        last_toogle_target_pressed = content.toogle_target_key_pressed;

        // update the update time
        update_time_ = node.now();

        if (verbose) {
            RCLCPP_INFO(node.get_logger(),
                        "======================== RX MSG ==========================");
            RCLCPP_INFO(node.get_logger(), "autoaim pub:    yaw: %.4f, pitch: %.4f",
                        autoaim_msg.yaw, autoaim_msg.pitch);
            RCLCPP_INFO(node.get_logger(), "jointState pub: yaw: %.4f, pitch: %.4f",
                        joint_state_msg.position[0], joint_state_msg.position[1]);
            RCLCPP_INFO(node.get_logger(), "quat pub:       x: %.4f, y: %.4f, z: %.4f, w: %.4f",
                        quat_msg.quaternion.x, quat_msg.quaternion.y, quat_msg.quaternion.z,
                        quat_msg.quaternion.w);
            RCLCPP_INFO(node.get_logger(), "color pub:      color: %d", color_msg.data);
            RCLCPP_INFO(node.get_logger(), "autoaimMode pub: %s",
                        (autoaim_mode_msg.data == 0) ? "vivid mode" : "use static mode");
            RCLCPP_INFO(node.get_logger(), "autoaimDecision: %s",
                        (autoaim_shoot_decision_msg.data == 0) ? "don't shoot" : "shoot");
            RCLCPP_INFO(node.get_logger(), "shoot data pub:  bullet speed: %f",
                        shoot_data_msg.bullet_speed);
            RCLCPP_INFO(node.get_logger(),
                        "======================== RX END ==========================");
        }
        return toogle_target;
    }

    bool isInvalid() const {
        const RxContent content = snapshot();
        return content.pitch == 0.f && content.yaw == 0.f && content.color == 0 &&
               content.bullet_spped == 0.f;
    }

   private:
    mutable std::mutex rx_mutex_;
    rclcpp::Time update_time_{0};

    inline void fillQuatMsg(geometry_msgs::msg::QuaternionStamped& msg, const RxContent& content) {
        msg.quaternion.w = content.q[0];
        msg.quaternion.x = content.q[1];
        msg.quaternion.y = content.q[2];
        msg.quaternion.z = content.q[3];
        msg.header.stamp = rclcpp::Clock().now();
        // msg.header.frame_id = ? // may use in future
    }

    inline void fillJointStateMsg(sensor_msgs::msg::JointState& msg, rclcpp::Node& node,
                                  const RxContent& content) {
        msg.header.stamp = node.now();
        msg.header.frame_id = "";
        msg.name.push_back("yaw_joint");
        msg.name.push_back("pitch_joint");
        float yaw_joint = content.yaw;
        msg.position.push_back(yaw_joint);
        float pitch_joint = content.pitch;
        msg.position.push_back(pitch_joint);
    }
};

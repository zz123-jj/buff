#include <Checksum.h>
#include <CommPort.h>
#include <filter.h>
#include <pid.h>

#include <RxMsg.hpp>
#include <TxMsg.hpp>
#include <atomic>
#include <chrono>
#include <cmath>  // M_PI
#include <functional>
#include <gary_msgs/msg/auto_aim.hpp>
#include <gary_msgs/msg/detail/shoot_data__struct.hpp>
#include <gary_msgs/msg/shoot_data.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <params.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>

#include "std_msgs/msg/int32.hpp"

#define RAD 57.2957795f

using namespace std::chrono_literals;

class ContactNode : public rclcpp::Node {
   public:
    ContactNode(int argc, char** argv)
        : Node("contact"),
          p_(*this),
          pid_yaw(p_.fillPidYawConfig()),
          pid_pitch(p_.fillPidPitchConfig()),
          low_p_filter_yaw_(p_.filter.yaw_cutoff),
          comm_(*this, p_.port.port_name, p_.port.baudrate) {
        comm_.Start();

        // ======================= the tx midware ========================
        comm_port_cb_ = [this](uint8_t* buffer, size_t size, bool safe_write) {
            comm_.Write(buffer, size, safe_write);
        };

        midware_unit_convert_ = [](float* pitch, float* yaw) {
            (void)pitch;
            (void)yaw;
            // *pitch = *pitch * RAD;
            // *yaw = *yaw * RAD;
        };

        midware_pid_ = [this](float* pitch, float* yaw, RxMsg& rx_state) {
            // (void)pitch;
            // const float yaw_command =
            //     pid_yaw.compute(pid_yaw.target_angle, rx_state.rx_content_.yaw) +
            //     rx_state.rx_content_.yaw;
            // const float pitch_command =
            //     pid_pitch.compute(pid_pitch.target_angle, rx_state.rx_content_.pitch) +
            //     rx_state.rx_content_.pitch;
            // *yaw = yaw_command;
            // *pitch = pitch_command;
        };

        midware_filter_ = [this](float* pitch, float* yaw, RxMsg& rx_state) {
            (void)pitch;
            (void)yaw;
            // if (low_p_filter_yaw_.get_initial() == 0.0f) {
            //     low_p_filter_yaw_.set_initial(rx_state.rx_content_.yaw);
            // }
            // *yaw = low_p_filter_yaw_.update(*yaw);
        };

        midware_boundary_ = [](float* pitch, float* yaw, RxMsg&) {
            // *yaw = (*yaw > 180.0f) ? (*yaw - 360.0f) : (*yaw <= -180.0f ? *yaw + 360.0f : *yaw);
            // *pitch = std::clamp(*pitch, -15.0f, 25.0f);
        };

        // ========================= subscriber =============================
        _autoaim_sub_ = this->create_subscription<gary_msgs::msg::AutoAIM>(
            "/autoaim/target", rclcpp::SensorDataQoS(),
            [this](const gary_msgs::msg::AutoAIM::SharedPtr msg) {
                auto& tx_msg = comm_.tx_msg();
                auto& rx_msg = comm_.rx_msg();

                tx_msg.autoaim_callback(*msg, rx_msg, *this, comm_port_cb_, midware_unit_convert_,
                                        midware_pid_, midware_filter_, midware_boundary_,
                                        p_.tx.verbose);
            });

        // ========================= publisher ==============================
        const auto rx_timer_period = std::chrono::milliseconds(p_.rx.rx_timerwall_ms);
        __timer_rx__ = this->create_wall_timer(rx_timer_period,
                                               std::bind(&ContactNode::rx_timer_callback, this));
        _autoaim_pub_ = this->create_publisher<gary_msgs::msg::AutoAIM>("/autoaim/status", 10);
        _quaternion_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "/quaternion", rclcpp::SensorDataQoS());
        _color_pub_ = this->create_publisher<std_msgs::msg::Int32>("/color", 10);
        _autoaim_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/autoaim/mode", 10);
        _autoaim_decision_pub_ =
            this->create_publisher<std_msgs::msg::Int32>("/autoaim/decision", 10);
        _ins_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        _joint_state_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
        _shoot_data_pub_ = this->create_publisher<gary_msgs::msg::ShootData>("/shoot_data", 10);
        _tracker_change_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/change");
    }

    ~ContactNode() {
        // shutdown all the resource first, or the CommPort may crash
        comm_.Stop();
    }

    void rx_timer_callback() {
        bool toogle_target = comm_.rx_msg().loadMsgAndPublish(
            _autoaim_pub_, _quaternion_pub_, _joint_state_pub_, _color_pub_, _autoaim_mode_pub_,
            _autoaim_decision_pub_, _shoot_data_pub_, *this, p_.rx.verbose);
        if (toogle_target) {
            if (_tracker_change_request_in_flight_.exchange(true)) {
                RCLCPP_WARN(this->get_logger(),
                            "Skip /tracker/change: previous request still in flight");
                return;
            }

            if (!_tracker_change_client_->service_is_ready()) {
                _tracker_change_request_in_flight_ = false;
                RCLCPP_WARN(this->get_logger(), "Service /tracker/change is not ready");
                return;
            }

            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            _tracker_change_client_->async_send_request(
                request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                    const auto response = future.get();
                    _tracker_change_request_in_flight_ = false;
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Called /tracker/change successfully: %s",
                                    response->message.c_str());
                        return;
                    }

                    RCLCPP_WARN(this->get_logger(), "Call to /tracker/change failed: %s",
                                response->message.c_str());
                });
        }
    }

   private:
    Params p_;
    PIDController pid_yaw;
    PIDController pid_pitch;
    LowPassFilter low_p_filter_yaw_;
    CommPort comm_;

    rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr _autoaim_sub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr _autoaim_pub_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr _quaternion_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _color_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _autoaim_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _autoaim_decision_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _ins_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_state_pub_;
    rclcpp::Publisher<gary_msgs::msg::ShootData>::SharedPtr _shoot_data_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _tracker_change_client_;
    std::atomic_bool _tracker_change_request_in_flight_{false};

    rclcpp::TimerBase::SharedPtr __timer_rx__;

    TxMsg::comm_port_cb_t comm_port_cb_;
    TxMsg::midware_plain_cb_t midware_unit_convert_;
    TxMsg::midware_state_cb_t midware_pid_;
    TxMsg::midware_state_cb_t midware_filter_;
    TxMsg::midware_state_cb_t midware_boundary_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContactNode>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

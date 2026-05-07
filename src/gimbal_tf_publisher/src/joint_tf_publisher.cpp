#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <string>
#include <vector>

class JointTFPublisher : public rclcpp::Node {
public:
    JointTFPublisher() : Node("joint_tf_publisher") {
        yaw_offset_ = read_pose_offset("yaw");
        roll_offset_ = read_pose_offset("roll");
        pitch_offset_ = read_pose_offset("pitch");

        // 订阅关节状态话题[1,2](@ref)
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&JointTFPublisher::joint_state_callback, this, std::placeholders::_1));
            
        // 初始化TF广播器[4,5](@ref)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    struct PoseOffset {
        tf2::Vector3 translation{0.0, 0.0, 0.0};
        tf2::Quaternion rotation{0.0, 0.0, 0.0, 1.0};
    };

    std::vector<double> read_vector3_param(const std::string& name) {
        this->declare_parameter<std::vector<double>>(name, {0.0, 0.0, 0.0});
        auto value = this->get_parameter(name).as_double_array();
        if (value.size() != 3) {
            RCLCPP_WARN(
                this->get_logger(), "Parameter %s must contain 3 numbers, use zeros",
                name.c_str());
            return {0.0, 0.0, 0.0};
        }
        return value;
    }

    PoseOffset read_pose_offset(const std::string& prefix) {
        const auto xyz = read_vector3_param(prefix + "_xyz");
        const auto rpy = read_vector3_param(prefix + "_rpy");

        PoseOffset offset;
        offset.translation = tf2::Vector3(xyz[0], xyz[1], xyz[2]);
        offset.rotation.setRPY(rpy[0], rpy[1], rpy[2]);
        offset.rotation.normalize();
        return offset;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        double yaw_pos = 0.0, roll_pos = 0.0, pitch_pos = 0.0;
        bool has_yaw = false, has_pitch = false;

        // 解析目标关节位置[1](@ref)
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "yaw_joint") {
                yaw_pos = msg->position[i];
                has_yaw = true;
            } else if (msg->name[i] == "roll_joint") {
                roll_pos = msg->position[i];
            } else if (msg->name[i] == "pitch_joint") {
                pitch_pos =  msg->position[i];
                has_pitch = true;
            }
        }

        if (!has_yaw || !has_pitch) {
            RCLCPP_WARN(this->get_logger(), "Missing required joint data");
            return;
        }

        const auto stamp = this->now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        tf2::Quaternion q_yaw, q_roll, q_pitch;
        q_yaw.setRPY(0, 0, yaw_pos);    // Z轴旋转（Yaw）
        q_yaw.normalize();
        transforms.emplace_back(
            make_transform(stamp, "odom", "gimbal_yaw_link", q_yaw, yaw_offset_));

        q_roll.setRPY(roll_pos, 0, 0);
        q_roll.normalize();
        transforms.emplace_back(
            make_transform(stamp, "gimbal_yaw_link", "gimbal_roll_link", q_roll, roll_offset_));

        q_pitch.setRPY(0, pitch_pos, 0); // Y轴旋转（Pitch）
        q_pitch.normalize();
        transforms.emplace_back(
            make_transform(stamp, "gimbal_roll_link", "gimbal_pitch_link", q_pitch, pitch_offset_));

        // 发布TF变换[5](@ref)
        tf_broadcaster_->sendTransform(transforms);
    }

    geometry_msgs::msg::TransformStamped make_transform(
        const rclcpp::Time& stamp, const char* parent, const char* child,
        const tf2::Quaternion& dynamic_rotation, const PoseOffset& offset) const {
        tf2::Quaternion rotation = offset.rotation * dynamic_rotation;
        rotation.normalize();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = parent;
        transform.child_frame_id = child;
        transform.transform.translation.x = offset.translation.x();
        transform.transform.translation.y = offset.translation.y();
        transform.transform.translation.z = offset.translation.z();
        transform.transform.rotation.x = rotation.x();
        transform.transform.rotation.y = rotation.y();
        transform.transform.rotation.z = rotation.z();
        transform.transform.rotation.w = rotation.w();
        return transform;
    }

    PoseOffset yaw_offset_;
    PoseOffset roll_offset_;
    PoseOffset pitch_offset_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTFPublisher>());
    rclcpp::shutdown();
    return 0;
}

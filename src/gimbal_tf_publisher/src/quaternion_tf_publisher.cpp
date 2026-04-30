#include <cmath>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

class QuaternionTFPublisher : public rclcpp::Node {
public:
    QuaternionTFPublisher() : Node("quaternion_tf_publisher") {
        yaw_offset_ = read_pose_offset("yaw");
        roll_offset_ = read_pose_offset("roll");
        pitch_offset_ = read_pose_offset("pitch");
        angle_mapping_ = read_angle_mapping();

        RCLCPP_INFO(
            this->get_logger(),
            "Quaternion TF chain: odom -> gimbal_yaw_link -> gimbal_roll_link -> "
            "gimbal_pitch_link; input axes: x->%s, y->%s, z->%s; decomposition=%s; "
            "angle mapping: roll=%g*%s, yaw=%g*%s, pitch=%g*%s",
            angle_mapping_.input_x_axis.c_str(),
            angle_mapping_.input_y_axis.c_str(),
            angle_mapping_.input_z_axis.c_str(),
            angle_mapping_.decomposition_order.c_str(),
            angle_mapping_.roll_sign, angle_mapping_.roll_source.c_str(),
            angle_mapping_.yaw_sign, angle_mapping_.yaw_source.c_str(),
            angle_mapping_.pitch_sign, angle_mapping_.pitch_source.c_str());

        subscription_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/quaternion", rclcpp::SensorDataQoS(),
            std::bind(&QuaternionTFPublisher::quaternion_callback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    struct RypAngles {
        double roll{0.0};
        double yaw{0.0};
        double pitch{0.0};
    };

    struct PoseOffset {
        tf2::Vector3 translation{0.0, 0.0, 0.0};
        tf2::Quaternion rotation{0.0, 0.0, 0.0, 1.0};
    };

    struct AngleMapping {
        std::string input_x_axis{"roll"};
        std::string input_y_axis{"pitch"};
        std::string input_z_axis{"yaw"};
        tf2::Matrix3x3 input_to_ros{1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0};
        std::string decomposition_order{"rpy"};
        std::string roll_source{"roll"};
        std::string yaw_source{"yaw"};
        std::string pitch_source{"pitch"};
        double roll_sign{1.0};
        double yaw_sign{1.0};
        double pitch_sign{1.0};
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

    static std::string normalize_source_name(std::string value) {
        for (auto& ch : value) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        return value;
    }

    static bool is_axis_source(const std::string& value) {
        return value == "roll" || value == "pitch" || value == "yaw" || value == "x" ||
               value == "y" || value == "z";
    }

    static bool is_valid_source(const std::string& value, bool allow_roll_special) {
        return is_axis_source(value) ||
               (allow_roll_special && (value == "none" || value == "auto"));
    }

    std::string read_source_param(
        const std::string& name, const std::string& fallback, bool allow_roll_special = false) {
        this->declare_parameter<std::string>(name, fallback);
        const auto value = normalize_source_name(this->get_parameter(name).as_string());
        if (is_valid_source(value, allow_roll_special)) {
            return value;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Parameter %s has invalid source '%s', use '%s'",
            name.c_str(), value.c_str(), fallback.c_str());
        return fallback;
    }

    static bool parse_axis_vector(const std::string& text, tf2::Vector3& axis) {
        auto value = normalize_source_name(text);
        double sign = 1.0;
        if (!value.empty() && (value.front() == '-' || value.front() == '+')) {
            sign = value.front() == '-' ? -1.0 : 1.0;
            value.erase(value.begin());
        }

        axis = tf2::Vector3(0.0, 0.0, 0.0);
        if (value == "x" || value == "roll") {
            axis.setX(sign);
            return true;
        }
        if (value == "y" || value == "pitch") {
            axis.setY(sign);
            return true;
        }
        if (value == "z" || value == "yaw") {
            axis.setZ(sign);
            return true;
        }
        return false;
    }

    std::string read_axis_param(const std::string& name, const std::string& fallback) {
        this->declare_parameter<std::string>(name, fallback);
        const auto value = normalize_source_name(this->get_parameter(name).as_string());
        tf2::Vector3 unused;
        if (parse_axis_vector(value, unused)) {
            return value;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Parameter %s has invalid axis '%s', use '%s'",
            name.c_str(), value.c_str(), fallback.c_str());
        return fallback;
    }

    tf2::Matrix3x3 read_input_to_ros_matrix(AngleMapping& mapping) {
        mapping.input_x_axis = read_axis_param("input_x_axis", mapping.input_x_axis);
        mapping.input_y_axis = read_axis_param("input_y_axis", mapping.input_y_axis);
        mapping.input_z_axis = read_axis_param("input_z_axis", mapping.input_z_axis);

        tf2::Vector3 x_axis;
        tf2::Vector3 y_axis;
        tf2::Vector3 z_axis;
        parse_axis_vector(mapping.input_x_axis, x_axis);
        parse_axis_vector(mapping.input_y_axis, y_axis);
        parse_axis_vector(mapping.input_z_axis, z_axis);

        const double determinant = x_axis.dot(y_axis.cross(z_axis));
        if (std::abs(std::abs(determinant) - 1.0) > 1e-6) {
            RCLCPP_WARN(
                this->get_logger(),
                "input_x_axis/input_y_axis/input_z_axis must describe 3 unique axes, use identity");
            mapping.input_x_axis = "roll";
            mapping.input_y_axis = "pitch";
            mapping.input_z_axis = "yaw";
            return tf2::Matrix3x3(1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0);
        }

        return tf2::Matrix3x3(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z());
    }

    double read_double_param(const std::string& name, double fallback) {
        this->declare_parameter<double>(name, fallback);
        return this->get_parameter(name).as_double();
    }

    std::string read_decomposition_order() {
        this->declare_parameter<std::string>("decomposition_order", "rpy");
        const auto order =
            normalize_source_name(this->get_parameter("decomposition_order").as_string());
        if (order == "rpy" || order == "zyx" || order == "xzy" || order == "zxy") {
            return order;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Parameter decomposition_order has invalid value '%s', use 'rpy'",
            order.c_str());
        return "rpy";
    }

    AngleMapping read_angle_mapping() {
        AngleMapping mapping;
        mapping.input_to_ros = read_input_to_ros_matrix(mapping);
        mapping.decomposition_order = read_decomposition_order();
        mapping.roll_source = read_source_param("roll_source", mapping.roll_source, true);
        mapping.yaw_source = read_source_param("yaw_source", mapping.yaw_source);
        mapping.pitch_source = read_source_param("pitch_source", mapping.pitch_source);
        mapping.roll_sign = read_double_param("roll_sign", mapping.roll_sign);
        mapping.yaw_sign = read_double_param("yaw_sign", mapping.yaw_sign);
        mapping.pitch_sign = read_double_param("pitch_sign", mapping.pitch_sign);
        return mapping;
    }

    static bool is_valid(const tf2::Quaternion& q) {
        const double norm2 = q.length2();
        return std::isfinite(norm2) && norm2 > 1e-12;
    }

    static double clamp_unit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    static double normalize_angle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    static RypAngles decompose_standard_rpy(const tf2::Matrix3x3& matrix) {
        RypAngles angles;
        matrix.getRPY(angles.roll, angles.pitch, angles.yaw);
        return angles;
    }

    static RypAngles decompose_xzy_with_yaw(const tf2::Matrix3x3& matrix, double yaw) {
        RypAngles angles;
        const double cos_yaw = std::cos(yaw);
        angles.yaw = normalize_angle(yaw);

        if (std::abs(cos_yaw) > 1e-6) {
            angles.roll = normalize_angle(
                std::atan2(matrix[2][1] / cos_yaw, matrix[1][1] / cos_yaw));
            angles.pitch = normalize_angle(
                std::atan2(matrix[0][2] / cos_yaw, matrix[0][0] / cos_yaw));
            return angles;
        }

        angles.roll = 0.0;
        angles.pitch = normalize_angle(std::atan2(-matrix[2][0], matrix[2][2]));
        return angles;
    }

    static double angular_distance(double lhs, double rhs) {
        return std::abs(normalize_angle(lhs - rhs));
    }

    static double decomposition_score(const RypAngles& angles, const RypAngles* reference) {
        if (reference != nullptr) {
            return angular_distance(angles.roll, reference->roll) +
                   angular_distance(angles.yaw, reference->yaw) +
                   angular_distance(angles.pitch, reference->pitch);
        }

        return std::abs(normalize_angle(angles.roll)) + std::abs(normalize_angle(angles.pitch));
    }

    static RypAngles decompose_xzy(const tf2::Matrix3x3& matrix, const RypAngles* reference) {
        static constexpr double kPi = 3.14159265358979323846;
        const double yaw_1 = std::asin(clamp_unit(-matrix[0][1]));
        const double yaw_2 = yaw_1 >= 0.0 ? kPi - yaw_1 : -kPi - yaw_1;

        const auto candidate_1 = decompose_xzy_with_yaw(matrix, yaw_1);
        const auto candidate_2 = decompose_xzy_with_yaw(matrix, yaw_2);
        return decomposition_score(candidate_2, reference) < decomposition_score(candidate_1, reference)
                   ? candidate_2
                   : candidate_1;
    }

    static RypAngles decompose_zxy_with_roll(const tf2::Matrix3x3& matrix, double roll) {
        RypAngles angles;
        const double cos_roll = std::cos(roll);
        angles.roll = normalize_angle(roll);

        if (std::abs(cos_roll) > 1e-6) {
            angles.yaw = normalize_angle(
                std::atan2(-matrix[0][1] / cos_roll, matrix[1][1] / cos_roll));
            angles.pitch = normalize_angle(
                std::atan2(-matrix[2][0] / cos_roll, matrix[2][2] / cos_roll));
            return angles;
        }

        angles.yaw = 0.0;
        angles.pitch = normalize_angle(std::atan2(matrix[0][2], matrix[0][0]));
        return angles;
    }

    static RypAngles decompose_zxy(const tf2::Matrix3x3& matrix, const RypAngles* reference) {
        static constexpr double kPi = 3.14159265358979323846;
        const double roll_1 = std::asin(clamp_unit(matrix[2][1]));
        const double roll_2 = roll_1 >= 0.0 ? kPi - roll_1 : -kPi - roll_1;

        const auto candidate_1 = decompose_zxy_with_roll(matrix, roll_1);
        const auto candidate_2 = decompose_zxy_with_roll(matrix, roll_2);
        return decomposition_score(candidate_2, reference) < decomposition_score(candidate_1, reference)
                   ? candidate_2
                   : candidate_1;
    }

    static RypAngles decompose_quaternion(
        const tf2::Quaternion& q, const AngleMapping& mapping,
        const RypAngles* reference) {
        const tf2::Matrix3x3 input_matrix(q);
        const tf2::Matrix3x3 ros_matrix =
            mapping.input_to_ros * input_matrix * mapping.input_to_ros.transpose();
        if (mapping.decomposition_order == "xzy") {
            return decompose_xzy(ros_matrix, reference);
        }
        if (mapping.decomposition_order == "zxy") {
            return decompose_zxy(ros_matrix, reference);
        }
        return decompose_standard_rpy(ros_matrix);
    }

    static double select_angle(const RypAngles& angles, const std::string& source) {
        if (source == "roll" || source == "x") {
            return angles.roll;
        }
        if (source == "pitch" || source == "y") {
            return angles.pitch;
        }
        if (source == "yaw" || source == "z") {
            return angles.yaw;
        }
        return 0.0;
    }

    static RypAngles apply_angle_mapping(const RypAngles& raw_angles, const AngleMapping& mapping) {
        RypAngles angles;
        angles.roll = mapping.roll_sign * select_angle(raw_angles, mapping.roll_source);
        angles.yaw = mapping.yaw_sign * select_angle(raw_angles, mapping.yaw_source);
        angles.pitch = mapping.pitch_sign * select_angle(raw_angles, mapping.pitch_source);
        return angles;
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

    void quaternion_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        const auto stamp = this->now();
        tf2::Quaternion q(msg->x, msg->y, msg->z, msg->w);
        if (!is_valid(q)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Received invalid /quaternion, skip TF publish");
            return;
        }

        q.normalize();
        const auto raw_angles = decompose_quaternion(
            q, angle_mapping_, has_last_raw_angles_ ? &last_raw_angles_ : nullptr);
        last_raw_angles_ = raw_angles;
        has_last_raw_angles_ = true;

        const auto angles = apply_angle_mapping(raw_angles, angle_mapping_);
        if (angle_mapping_.roll_source == "none") {
            publish_angles(stamp, 0.0, angles.yaw, angles.pitch);
        } else {
            publish_angles(stamp, angles.roll, angles.yaw, angles.pitch);
        }
    }

    void publish_angles(const rclcpp::Time& stamp, double roll, double yaw, double pitch) {
        tf2::Quaternion q_roll;
        q_roll.setRPY(roll, 0.0, 0.0);
        q_roll.normalize();

        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);
        q_yaw.normalize();

        tf2::Quaternion q_pitch;
        q_pitch.setRPY(0.0, pitch, 0.0);
        q_pitch.normalize();

        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.emplace_back(
            make_transform(stamp, "odom", "gimbal_yaw_link", q_yaw, yaw_offset_));
        transforms.emplace_back(
            make_transform(stamp, "gimbal_yaw_link", "gimbal_roll_link", q_roll, roll_offset_));
        transforms.emplace_back(
            make_transform(
                stamp, "gimbal_roll_link", "gimbal_pitch_link", q_pitch, pitch_offset_));

        tf_broadcaster_->sendTransform(transforms);
    }

    PoseOffset yaw_offset_;
    PoseOffset roll_offset_;
    PoseOffset pitch_offset_;
    AngleMapping angle_mapping_;
    RypAngles last_raw_angles_;
    bool has_last_raw_angles_{false};
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuaternionTFPublisher>());
    rclcpp::shutdown();
    return 0;
}

#include "buff_interfaces/msg/buff_world_model.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "gary_msgs/msg/auto_aim_debug.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double kSmallBuffAngularSpeed = M_PI / 3.0;
constexpr double kEps = 1e-6;

struct BallisticResult
{
    double pitch = 0.0;
    double t_fly = 0.0;
};

Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point& point)
{
    return {point.x, point.y, point.z};
}

Eigen::Vector3d vectorToEigen(const geometry_msgs::msg::Vector3& vector)
{
    return {vector.x, vector.y, vector.z};
}

Eigen::Vector3d rotateRodrigues(
    const Eigen::Vector3d& vector,
    const Eigen::Vector3d& axis,
    double angle)
{
    const Eigen::Vector3d k = axis.normalized();
    return vector * std::cos(angle)
         + k.cross(vector) * std::sin(angle)
         + k * k.dot(vector) * (1.0 - std::cos(angle));
}

double stampToSeconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

class RodriguesRotator
{
public:
    bool update(const buff_interfaces::msg::BuffWorldModel& model)
    {
        if (!model.valid)
        {
            valid_ = false;
            return false;
        }

        center_ = pointToEigen(model.circle_center_world);
        axis_ = vectorToEigen(model.circle_axis_world);
        current_point_ = pointToEigen(model.fitted_point_world);
        radius_ = model.circle_radius;
        current_angle_ = model.current_angle;
        angular_velocity_ = model.angular_velocity;
        mode_ = model.mode;
        spin_direction_ = model.spin_direction;
        if (spin_direction_ == 0 && std::abs(angular_velocity_) > kEps)
        {
            spin_direction_ = angular_velocity_ > 0.0 ? 1 : -1;
        }

        speed_a_ = model.speed_a;
        speed_omega_ = model.speed_omega;
        speed_phi_ = model.speed_phi;
        speed_b_ = model.speed_b;
        fit_start_time_sec_ = model.speed_fit_start_time_sec;
        stamp_sec_ = stampToSeconds(model.header.stamp);

        if (!center_.allFinite() || !axis_.allFinite() || !current_point_.allFinite() ||
            axis_.norm() < kEps || radius_ <= kEps)
        {
            valid_ = false;
            return false;
        }

        axis_.normalize();
        Eigen::Vector3d radius_vector = current_point_ - center_;
        radius_vector -= axis_ * radius_vector.dot(axis_);
        if (!radius_vector.allFinite() || radius_vector.norm() < kEps)
        {
            valid_ = false;
            return false;
        }

        current_point_ = center_ + radius_ * radius_vector.normalized();
        frame_id_ = model.header.frame_id;
        valid_ = true;
        return true;
    }

    bool valid() const
    {
        return valid_;
    }

    int8_t mode() const
    {
        return mode_;
    }

    const std::string& frameId() const
    {
        return frame_id_;
    }

    Eigen::Vector3d currentPoint() const
    {
        return current_point_;
    }

    Eigen::Vector3d predict(double delta_t) const
    {
        if (!valid_)
        {
            return current_point_;
        }

        const double delta_theta = integrateAngularSpeed(std::max(0.0, delta_t));
        const Eigen::Vector3d radius_vector = current_point_ - center_;
        return center_ + rotateRodrigues(radius_vector, axis_, delta_theta);
    }

private:
    double integrateAngularSpeed(double delta_t) const
    {
        if (spin_direction_ == 0)
        {
            return 0.0;
        }

        if (mode_ == 1)
        {
            if (std::abs(speed_omega_) > kEps && std::abs(speed_a_) > kEps)
            {
                const double t0 = stamp_sec_ - fit_start_time_sec_;
                const double t1 = t0 + delta_t;
                const double abs_speed_integral = speed_b_ * delta_t -
                    (speed_a_ / speed_omega_) *
                        (std::cos(speed_omega_ * t1 + speed_phi_) -
                         std::cos(speed_omega_ * t0 + speed_phi_));
                return static_cast<double>(spin_direction_) * abs_speed_integral;
            }

            return static_cast<double>(spin_direction_) * speed_b_ * delta_t;
        }

        if (mode_ == -1)
        {
            return static_cast<double>(spin_direction_) * kSmallBuffAngularSpeed * delta_t;
        }

        return 0.0;
    }

    bool valid_ = false;
    int8_t mode_ = 0;
    int8_t spin_direction_ = 0;
    std::string frame_id_ = "odom";
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_ = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d current_point_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;
    double current_angle_ = 0.0;
    double angular_velocity_ = 0.0;
    double speed_a_ = 0.0;
    double speed_omega_ = 0.0;
    double speed_phi_ = 0.0;
    double speed_b_ = 0.0;
    double fit_start_time_sec_ = 0.0;
    double stamp_sec_ = 0.0;
};

class BuffAimer : public rclcpp::Node
{
public:
    explicit BuffAimer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("buff_aimer_node", options)
    {
        declare_parameter<std::string>("frames.target", "odom");
        declare_parameter<double>("ballistic.bullet_speed", 23.8);
        declare_parameter<double>("ballistic.shoot_delay", 0.05);
        declare_parameter<double>("ballistic.gravity", 9.81);
        declare_parameter<double>("ballistic.air_k", 0.001);
        declare_parameter<double>("ballistic.converge_thresh", 0.002);
        declare_parameter<double>("ballistic.sim_dt", 0.005);
        declare_parameter<bool>("aiming.debug", true);
        declare_parameter<int>("max_iteration", 5);
        declare_parameter<double>("stop_error", 0.001);

        target_frame_ = get_parameter("frames.target").as_string();
        bullet_speed_ = get_parameter("ballistic.bullet_speed").as_double();
        shoot_delay_ = get_parameter("ballistic.shoot_delay").as_double();
        gravity_ = get_parameter("ballistic.gravity").as_double();
        air_k_ = get_parameter("ballistic.air_k").as_double();
        converge_thresh_ = get_parameter("ballistic.converge_thresh").as_double();
        sim_dt_ = get_parameter("ballistic.sim_dt").as_double();
        debug_ = get_parameter("aiming.debug").as_bool();
        max_iteration_ = get_parameter("max_iteration").as_int();
        stop_error_ = get_parameter("stop_error").as_double();

        RCLCPP_INFO(get_logger(), "BuffAimer started. bullet=%.2f m/s", bullet_speed_);

        world_model_sub_ = create_subscription<buff_interfaces::msg::BuffWorldModel>(
            "/buff/world_model", rclcpp::SensorDataQoS(),
            [this](const buff_interfaces::msg::BuffWorldModel::SharedPtr msg) {
                worldModelCallback(msg);
            });

        autoaim_pub_ = create_publisher<gary_msgs::msg::AutoAIM>(
            "/autoaim/target", rclcpp::SensorDataQoS());

        if (debug_)
        {
            debug_pub_ = create_publisher<gary_msgs::msg::AutoAimDebug>(
                "/autoaim/debug", rclcpp::SensorDataQoS());
        }
    }

private:
    void worldModelCallback(const buff_interfaces::msg::BuffWorldModel::SharedPtr msg)
    {
        if (!rotator_.update(*msg))
        {
            return;
        }

        double msg_latency = (now() - rclcpp::Time(msg->header.stamp)).seconds();
        msg_latency = std::max(0.0, msg_latency);

        const Eigen::Vector3d current_target = rotator_.currentPoint();
        double t_fly = std::max(
            std::hypot(current_target.x(), current_target.y()) / bullet_speed_,
            0.05);

        double final_yaw = 0.0;
        double final_pitch = 0.0;
        double distance_3d = 0.0;
        bool converged = false;

        for (int iter = 0; iter < max_iteration_; ++iter)
        {
            const double predict_delay = msg_latency + t_fly + shoot_delay_ + 0.04;
            const Eigen::Vector3d predicted_target = rotator_.predict(predict_delay);
            const double distance_xy = std::hypot(predicted_target.x(), predicted_target.y());
            const BallisticResult res = solveBallistic(distance_xy, predicted_target.z());

            final_pitch = -res.pitch;
            final_yaw = std::atan2(predicted_target.y(), predicted_target.x());
            distance_3d = std::hypot(distance_xy, predicted_target.z());

            if (std::abs(t_fly - res.t_fly) < converge_thresh_)
            {
                converged = true;
                break;
            }
            t_fly = res.t_fly;
        }

        if (!converged)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Ballistic iteration did not converge, using last result.");
        }

        auto autoaim_msg = std::make_unique<gary_msgs::msg::AutoAIM>();
        autoaim_msg->yaw = static_cast<float>(final_yaw);
        autoaim_msg->pitch = static_cast<float>(final_pitch);
        autoaim_msg->target_distance = static_cast<float>(distance_3d);
        autoaim_msg->header.stamp = now();
        autoaim_msg->header.frame_id = rotator_.frameId().empty() ? target_frame_ : rotator_.frameId();
        autoaim_msg->target_id = gary_msgs::msg::AutoAIM::TARGET_ID6_OUTPOST;
        autoaim_msg->vision_mode = rotator_.mode() == 1
            ? gary_msgs::msg::AutoAIM::VISION_MODE_BIG
            : gary_msgs::msg::AutoAIM::VISION_MODE_SMALL;
        autoaim_msg->shoot_command = gary_msgs::msg::AutoAIM::ALLOW_SHOOT;
        autoaim_msg->shoot_mode = gary_msgs::msg::AutoAIM::SHOOT_MODE_AUTO;
        autoaim_pub_->publish(std::move(autoaim_msg));

        if (debug_)
        {
            auto debug_msg = gary_msgs::msg::AutoAimDebug();
            debug_msg.plan_yaw = final_yaw;
            debug_msg.plan_pitch = final_pitch;
            debug_pub_->publish(debug_msg);
        }
    }

    BallisticResult solveBallistic(double distance_xy, double distance_z) const
    {
        const double v0 = bullet_speed_;
        const double g = gravity_;
        const double k = air_k_;
        const double dt = sim_dt_;

        const double v2 = v0 * v0;
        const double v4 = v2 * v2;
        const double root = std::sqrt(v4 - g * (g * distance_xy * distance_xy + 2 * distance_z * v2));
        double pitch = std::isnan(root)
            ? std::atan2(distance_z, distance_xy)
            : std::atan2(v2 - root, g * distance_xy);

        double y_aim = distance_z;
        double t_fly = 0.0;

        for (int iter = 0; iter < max_iteration_; ++iter)
        {
            double x = 0.0;
            double y = 0.0;
            double t = 0.0;
            double vx = v0 * std::cos(pitch);
            double vy = v0 * std::sin(pitch);

            while (x < distance_xy)
            {
                const double v = std::hypot(vx, vy);
                const double ax = -k * v * vx;
                const double ay = -g - k * v * vy;
                vx += ax * dt;
                vy += ay * dt;
                x += vx * dt;
                y += vy * dt;
                t += dt;
                if (t > 2.0)
                {
                    break;
                }
            }

            const double error = distance_z - y;
            t_fly = t;
            if (std::abs(error) < stop_error_)
            {
                break;
            }

            y_aim += error;
            pitch = std::atan2(y_aim, distance_xy);
        }

        return {pitch, t_fly};
    }

    rclcpp::Subscription<buff_interfaces::msg::BuffWorldModel>::SharedPtr world_model_sub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr autoaim_pub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAimDebug>::SharedPtr debug_pub_;

    RodriguesRotator rotator_;
    std::string target_frame_ = "odom";
    double bullet_speed_ = 23.8;
    double shoot_delay_ = 0.05;
    double gravity_ = 9.81;
    double air_k_ = 0.001;
    double converge_thresh_ = 0.002;
    double sim_dt_ = 0.005;
    double stop_error_ = 0.001;
    int max_iteration_ = 5;
    bool debug_ = true;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffAimer)

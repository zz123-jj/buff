
// ============================================================
// 文件名: solver.cpp
// 描述:   能量机关自瞄求解器，接收 BuffAimingData 和 joint_states，
//         计算相对 yaw/pitch 并发布 AutoAIM 消息
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <memory>
#include <string>

#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "gary_msgs/msg/auto_aim_debug.hpp"

// ==================== 弹道解算结果 ====================
struct BallisticResult {
    double pitch;   // 枪口仰角 (弧度)
    double t_fly;   // 子弹飞行时间 (秒)
};

// ==================== 主节点类 ====================
class BuffSolver : public rclcpp::Node {
public:
    explicit BuffSolver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("buff_solver", options)
    {
        // ----- 参数声明 -----
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("gimbal_frame", "gimbal_yaw_link");
        this->declare_parameter<double>("bullet_speed", 15.0);
        this->declare_parameter<double>("shoot_delay", 0.05);
        this->declare_parameter<double>("physical_radius", 1.5);
        this->declare_parameter<double>("gravity", 9.81);
        this->declare_parameter<double>("air_k", 0.001);
        this->declare_parameter<double>("converge_thresh", 0.002);
        this->declare_parameter<double>("sim_dt", 0.005);
        this->declare_parameter<bool>("debug", true);

        // 读取参数
        odom_frame_       = this->get_parameter("odom_frame").as_string();
        gimbal_frame_     = this->get_parameter("gimbal_frame").as_string();
        bullet_speed_     = this->get_parameter("bullet_speed").as_double();
        shoot_delay_      = this->get_parameter("shoot_delay").as_double();
        physical_radius_  = this->get_parameter("physical_radius").as_double();
        gravity_          = this->get_parameter("gravity").as_double();
        air_k_            = this->get_parameter("air_k").as_double();
        converge_thresh_  = this->get_parameter("converge_thresh").as_double();
        sim_dt_           = this->get_parameter("sim_dt").as_double();
        debug_            = this->get_parameter("debug").as_bool();

        RCLCPP_INFO(this->get_logger(),
            "BuffSolver started. Bullet:%.2fm/s, Radius:%.2fm",
            bullet_speed_, physical_radius_);

        // ----- TF 初始化 -----
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ----- 订阅者 -----
        aiming_sub_ = this->create_subscription<buff_interfaces::msg::BuffAimingData>(
            "/buff/aiming_data", rclcpp::SensorDataQoS(),
            std::bind(&BuffSolver::aimingCallback, this, std::placeholders::_1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&BuffSolver::jointCallback, this, std::placeholders::_1));

        // ----- 发布者 -----
        autoaim_pub_ = this->create_publisher<gary_msgs::msg::AutoAIM>(
            "/autoaim/target", rclcpp::SensorDataQoS());

        if (debug_) {
            debug_pub_ = this->create_publisher<gary_msgs::msg::AutoAimDebug>(
                "/autoaim/debug", rclcpp::SensorDataQoS());
        }
    }

private:
    // ---------- 回调 ----------
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        current_joint_msg_ = msg;
    }

    void aimingCallback(const buff_interfaces::msg::BuffAimingData::SharedPtr msg) {
        auto autoaim_msg = std::make_unique<gary_msgs::msg::AutoAIM>();

        // 无效情况：未跟踪或缺少关节状态
        if (!msg->is_tracking || !current_joint_msg_) {
            publishZeroCommand(std::move(autoaim_msg));
            return;
        }

        // 1. 计算消息延迟
        rclcpp::Time msg_stamp(msg->header.stamp);
        double msg_latency = (this->now() - msg_stamp).seconds();
        if (msg_latency < 0.0) msg_latency = 0.0;

        // 2. 初始飞行时间估计
        geometry_msgs::msg::Point r_center_in_gimbal;
        if (!transformPoint(
                msg->r_center_x_3d, msg->r_center_y_3d, msg->r_center_z_3d,
                msg_stamp, r_center_in_gimbal)) {
            publishZeroCommand(std::move(autoaim_msg));
            return;
        }
        double t_fly = std::max(
            std::hypot(r_center_in_gimbal.x, r_center_in_gimbal.y) / bullet_speed_,
            0.05);

        // 3. 迭代收敛
        BallisticResult final_result{0.0, t_fly};
        geometry_msgs::msg::Point target_in_gimbal;
        bool converged = false;

        for (int iter = 0; iter < 5; ++iter) {
            double total_delay = msg_latency + t_fly + shoot_delay_;

            // 预测目标位置 (odom系)
            geometry_msgs::msg::Point pred_odom = predictTargetPosition(*msg, total_delay);

            // 转换到枪口坐标系
            if (!transformPoint(pred_odom.x, pred_odom.y, pred_odom.z,
                                msg_stamp, target_in_gimbal)) {
                publishZeroCommand(std::move(autoaim_msg));
                return;
            }

            double dx = target_in_gimbal.x;
            double dy = target_in_gimbal.y;
            double dz = target_in_gimbal.z;
            double distance_xy = std::hypot(dx, dy);
            double distance_z  = dz;

            BallisticResult res = solveBallistic(distance_xy, distance_z);

            if (std::abs(t_fly - res.t_fly) < converge_thresh_) {
                final_result = res;
                converged = true;
                break;
            }
            t_fly = res.t_fly;
            final_result = res;
        }

        // 4. 计算绝对目标角度
        double plan_yaw   = std::atan2(target_in_gimbal.y, target_in_gimbal.x);
        double plan_pitch = final_result.pitch;

        // 5. 提取当前云台角度
        double current_yaw = 0.0, current_pitch = 0.0;
        for (size_t i = 0; i < current_joint_msg_->name.size(); ++i) {
            if (current_joint_msg_->name[i] == "yaw_joint") {
                current_yaw = current_joint_msg_->position[i];
            } else if (current_joint_msg_->name[i] == "pitch_joint") {
                current_pitch = current_joint_msg_->position[i];
            }
        }

        // 6. 相对角度差
        double yaw_diff   = plan_yaw - current_yaw;
        double pitch_diff = plan_pitch - current_pitch;

        // 7. 填充并发布
        autoaim_msg->yaw   = yaw_diff;
        autoaim_msg->pitch = pitch_diff;
        autoaim_msg->target_distance =
            std::hypot(target_in_gimbal.x, target_in_gimbal.y, target_in_gimbal.z);
        autoaim_msg->shoot_command = 1;  // 允许射击
        autoaim_msg->header.stamp = this->now();
        autoaim_msg->header.frame_id = gimbal_frame_;
        autoaim_pub_->publish(std::move(autoaim_msg));

        // 8. 调试信息
        if (debug_) {
            auto debug_msg = gary_msgs::msg::AutoAimDebug();
            debug_msg.plan_yaw   = plan_yaw;
            debug_msg.plan_pitch = plan_pitch;
            debug_msg.yaw_diff   = yaw_diff;
            debug_pub_->publish(debug_msg);
        }
    }

    // ---------- 辅助函数 ----------
    void publishZeroCommand(std::unique_ptr<gary_msgs::msg::AutoAIM> msg) {
        msg->yaw = 0.0;
        msg->pitch = 0.0;
        msg->target_distance = 0.0;
        msg->header.stamp = this->now();
        msg->header.frame_id = gimbal_frame_;
        autoaim_pub_->publish(std::move(msg));
        if (debug_) {
            auto debug_msg = gary_msgs::msg::AutoAimDebug();
            debug_pub_->publish(debug_msg);
        }
    }

    bool transformPoint(double x, double y, double z,
                        const rclcpp::Time& stamp,
                        geometry_msgs::msg::Point& out) {
        geometry_msgs::msg::PointStamped in, out_stamped;
        in.header.frame_id = odom_frame_;
        in.header.stamp = stamp;
        in.point.x = x;
        in.point.y = y;
        in.point.z = z;
        try {
            tf_buffer_->transform(in, out_stamped, gimbal_frame_,
                                  tf2::durationFromSec(0.02));
            out = out_stamped.point;
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF error: %s", e.what());
            return false;
        }
    }

    geometry_msgs::msg::Point predictTargetPosition(
        const buff_interfaces::msg::BuffAimingData& msg, double delta_t) {
        double theta0 = std::atan2(msg.target_y_3d - msg.r_center_y_3d,
                                   msg.target_x_3d - msg.r_center_x_3d);

        double a = msg.sin_a;
        double omega = msg.sin_omega;
        double phi   = msg.sin_phi;
        double b     = msg.sin_b;

        double t0 = rclcpp::Time(msg.header.stamp).seconds();
        double t_pred = t0 + delta_t;

        double cos_pred = std::cos(omega * t_pred + phi);
        double cos_t0   = std::cos(omega * t0 + phi);
        double delta_theta = b * delta_t - (a / omega) * (cos_pred - cos_t0);

        double theta_pred = theta0 + delta_theta;

        geometry_msgs::msg::Point p;
        p.x = msg.r_center_x_3d + physical_radius_ * std::cos(theta_pred);
        p.y = msg.r_center_y_3d + physical_radius_ * std::sin(theta_pred);
        p.z = msg.r_center_z_3d;
        return p;
    }

    BallisticResult solveBallistic(double distance_xy, double distance_z) {
        const double v0 = bullet_speed_;
        const double g = gravity_;
        const double k = air_k_;
        const double dt = sim_dt_;
        const double stop_err = 0.001;
        const int max_iter = 20;

        // 初值（理想抛物线）
        double v2 = v0 * v0;
        double r = std::sqrt(v2*v2 - g*(g*distance_xy*distance_xy + 2*distance_z*v2));
        double pitch = std::isnan(r) ? std::atan2(distance_z, distance_xy)
                                     : std::atan2(v2 - r, g * distance_xy);

        double y_aim = distance_z;
        double t_fly = 0.0;

        for (int iter = 0; iter < max_iter; ++iter) {
            double x = 0.0, y = 0.0, t = 0.0;
            double vx = v0 * std::cos(pitch);
            double vy = v0 * std::sin(pitch);

            while (x < distance_xy) {
                double v = std::sqrt(vx*vx + vy*vy);
                double accel = -k * v;
                vx += accel * vx * dt;
                vy += (-g + accel * vy) * dt;
                x += vx * dt;
                y += vy * dt;
                t += dt;
                if (t > 5.0) break;
            }

            double error = distance_z - y;
            t_fly = t;

            if (std::abs(error) < stop_err) break;

            y_aim += error;
            pitch = std::atan2(y_aim, distance_xy);
        }

        return {-pitch, t_fly};
    }

    // ---------- 成员变量 ----------
    rclcpp::Subscription<buff_interfaces::msg::BuffAimingData>::SharedPtr aiming_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr autoaim_pub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAimDebug>::SharedPtr debug_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::JointState::SharedPtr current_joint_msg_;

    std::string odom_frame_;
    std::string gimbal_frame_;
    double bullet_speed_;
    double shoot_delay_;
    double physical_radius_;
    double gravity_;
    double air_k_;
    double converge_thresh_;
    double sim_dt_;
    bool debug_;
};

// 主函数（独立可执行文件）
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuffSolver>());
    rclcpp::shutdown();
    return 0;
}

// 作为组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffSolver)
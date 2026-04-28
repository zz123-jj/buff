// ============================================================
// 文件名: buff_solver.cpp
// 描述:   能量机关自瞄解算器（支持大小符，含弹道迭代）
// ============================================================

#include <ctime>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "gary_msgs/msg/auto_aim_debug.hpp"

struct BallisticResult {
    double pitch;   // 枪口绝对仰角 (rad)
    double t_fly;   // 子弹飞行时间 (s)
};

class BuffSolver : public rclcpp::Node {
public:
    explicit BuffSolver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("buff_solver_node", options)
    {
        // ---------- 参数声明 ----------
        this->declare_parameter<std::string>("frames.target", "odom");
        this->declare_parameter<double>("ballistic.bullet_speed", 23.8);
        this->declare_parameter<double>("ballistic.shoot_delay", 0.05);
        this->declare_parameter<double>("physical_radius", 0.75);
        this->declare_parameter<double>("ballistic.gravity", 9.81);
        this->declare_parameter<double>("ballistic.air_k", 0.001);
        this->declare_parameter<double>("ballistic.converge_thresh", 0.002);
        this->declare_parameter<double>("ballistic.sim_dt", 0.005);
        this->declare_parameter<bool>("aiming.debug", true);
        this->declare_parameter<int>("max_iteration", 5);
        this->declare_parameter<double>("stop_error", 0.001);
        this->declare_parameter<double>("gimbal.command_limit", 0.1);
        this->declare_parameter<double>("gimbal.yaw_command_rate_limit", 0.8);
        this->declare_parameter<double>("gimbal.pitch_command_rate_limit", 0.8);
        this->declare_parameter<double>("gimbal.max_command_dt", 0.05);

// 读取参数也要对应加上前缀
        target_frame_    = this->get_parameter("frames.target").as_string();
        bullet_speed_    = this->get_parameter("ballistic.bullet_speed").as_double();
        shoot_delay_     = this->get_parameter("ballistic.shoot_delay").as_double();
        physical_radius_ = this->get_parameter("physical_radius").as_double();
        gravity_         = this->get_parameter("ballistic.gravity").as_double();
        air_k_           = this->get_parameter("ballistic.air_k").as_double();
        converge_thresh_ = this->get_parameter("ballistic.converge_thresh").as_double();
        sim_dt_          = this->get_parameter("ballistic.sim_dt").as_double();
        debug_           = this->get_parameter("aiming.debug").as_bool();
        max_iteration_   = this->get_parameter("max_iteration").as_int();
        stop_error_      = this->get_parameter("stop_error").as_double();
        command_limit_ = std::max(0.0, this->get_parameter("gimbal.command_limit").as_double());
        yaw_command_rate_limit_ =
            std::max(0.0, this->get_parameter("gimbal.yaw_command_rate_limit").as_double());
        pitch_command_rate_limit_ =
            std::max(0.0, this->get_parameter("gimbal.pitch_command_rate_limit").as_double());
        max_command_dt_ = std::max(0.0, this->get_parameter("gimbal.max_command_dt").as_double());

        RCLCPP_INFO(this->get_logger(),
            "BuffSolver started. bullet=%.2f m/s, radius=%.2f m", bullet_speed_, physical_radius_);

        // ---------- 订阅与发布 ----------
        aiming_sub_ = this->create_subscription<buff_interfaces::msg::BuffAimingData>(
            "/buff/aiming_data", rclcpp::SensorDataQoS(),
            [this](const buff_interfaces::msg::BuffAimingData::SharedPtr msg) {
                this->aimingCallback(msg);
            });
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->jointCallback(msg);
            });

        autoaim_pub_ = this->create_publisher<gary_msgs::msg::AutoAIM>(
            "/autoaim/target", rclcpp::SensorDataQoS());

        if (debug_) {
            debug_pub_ = this->create_publisher<gary_msgs::msg::AutoAimDebug>(
                "/autoaim/debug", rclcpp::SensorDataQoS());
        }
    }

private:
    // ---------- 回调：保存当前关节角度 ----------
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        bool has_yaw = false;
        bool has_pitch = false;
        double yaw = 0.0;
        double pitch = 0.0;

        if (msg->position.size() < msg->name.size()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "JointState position size (%zu) is smaller than name size (%zu).",
                msg->position.size(), msg->name.size());
        }

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i >= msg->position.size()) {
                break;
            }

            if (msg->name[i] == "yaw_joint") {
                yaw = msg->position[i];
                has_yaw = true;
            } else if (msg->name[i] == "pitch_joint") {
                pitch = msg->position[i];
                has_pitch = true;
            }
        }

        if (!has_yaw || !has_pitch) {
            current_joint_msg_.reset();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "JointState missing yaw_joint or pitch_joint.");
            return;
        }

        current_joint_msg_ = msg;
        current_yaw = yaw;
        current_pitch = pitch;
    }

    // ---------- 主回调 ----------
    void aimingCallback(const buff_interfaces::msg::BuffAimingData::SharedPtr msg) {
        // 若无关节状态数据或未追踪，不发布控制消息，并重置限幅状态。
        if (!current_joint_msg_||!msg->is_tracking) {
            resetCommandLimiter();
            return;
        }
        // 计算消息延迟
        rclcpp::Time msg_stamp = msg->header.stamp;
        double msg_latency = (this->now() - msg_stamp).seconds();
        if (msg_latency < 0.0) msg_latency = 0.0;

         // 初始飞行时间估计（基于当前目标位置和子弹速度）
        double t_fly = std::max(std::hypot( msg->target_x_3d, msg->target_y_3d ) / bullet_speed_, 0.05);

        // ---------- 弹道迭代 ----------
        BallisticResult final_result{0.0, t_fly};
        double final_yaw = 0.0;
        double final_pitch = 0.0;
        double distance_3d = 0.0;
        bool converged = false;

        for (int iter = 0; iter < max_iteration_; ++iter) {
        // 1. 利用当前的 t_fly 计算预测延迟
        double predict_delay = msg_latency + t_fly + shoot_delay_+0.04;

        // 2. 预测目标在那时在哪
        geometry_msgs::msg::Point pred_odom = predictTargetPosition(*msg, predict_delay);

        // 3. 计算弹道
        double distance_xy = std::hypot(pred_odom.x, pred_odom.y);
        BallisticResult res = solveBallistic(distance_xy, pred_odom.z);

        // 4. 更新最终结果
        final_result = res;
        final_pitch = -res.pitch;
        final_yaw = std::atan2(pred_odom.y, pred_odom.x);
        distance_3d = std::hypot(distance_xy, pred_odom.z);

        // 5. 检查是否收敛
        if (std::abs(t_fly - res.t_fly) < converge_thresh_) {
            converged = true;
            break;
        }

        // 6.把算出来的实际飞行时间传给下一轮，进行更精确的预测
        t_fly = res.t_fly;
    }
        if (!converged) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Ballistic iteration did not converge, using last result.");
        }

        // 计算相对角度差
        double pitch_diff = -(final_pitch - current_pitch);
        double yaw_diff   = final_yaw - current_yaw;
        const rclcpp::Time publish_time = this->now();
        double yaw_cmd = std::clamp(yaw_diff, -command_limit_, command_limit_);
        double pitch_cmd = std::clamp(pitch_diff, -command_limit_, command_limit_);

        if (has_last_command_) {
            double dt = (publish_time - last_command_time_).seconds();
            if (!std::isfinite(dt) || dt < 0.0) {
                dt = 0.0;
            }
            if (max_command_dt_ > 0.0) {
                dt = std::min(dt, max_command_dt_);
            }

            yaw_cmd = limitCommandRate(yaw_cmd, last_yaw_cmd_, yaw_command_rate_limit_, dt);
            pitch_cmd = limitCommandRate(pitch_cmd, last_pitch_cmd_, pitch_command_rate_limit_, dt);
        }

        // 发布自瞄指令
        auto autoaim_msg = std::make_unique<gary_msgs::msg::AutoAIM>();
        autoaim_msg->yaw   = yaw_cmd;
        autoaim_msg->pitch = pitch_cmd;
        autoaim_msg->target_distance = distance_3d;
        autoaim_msg->header.stamp = publish_time;
        autoaim_msg->header.frame_id = target_frame_;
        autoaim_msg->shoot_command = 1;
        autoaim_pub_->publish(std::move(autoaim_msg));

        last_yaw_cmd_ = yaw_cmd;
        last_pitch_cmd_ = pitch_cmd;
        last_command_time_ = publish_time;
        has_last_command_ = true;

        // 调试信息
        if (debug_) {
            auto debug_msg = gary_msgs::msg::AutoAimDebug();
            debug_msg.plan_yaw   = final_yaw;
            debug_msg.plan_pitch = final_pitch;
            debug_pub_->publish(debug_msg);
        }
    }


    double limitCommandRate(double target, double last, double rate_limit, double dt) const {
        const double max_step = rate_limit * dt;
        return std::clamp(target, last - max_step, last + max_step);
    }

    void resetCommandLimiter() {
        has_last_command_ = false;
        last_yaw_cmd_ = 0.0;
        last_pitch_cmd_ = 0.0;
    }


    geometry_msgs::msg::Point predictTargetPosition(
        const buff_interfaces::msg::BuffAimingData& msg, double delta_t) {

        double delta_theta = 0.0;
        const double eps = 1e-6;

        if (msg.is_bigbuff == 1) {
            // ---------- 大符：角速度 = A*sin(ωt + φ) + B ----------
            double A = msg.sin_a;
            double omega = msg.sin_omega;
            double phi = msg.sin_phi;
            double B = msg.sin_b;

            if (std::abs(omega) > eps && A > eps) {
                double t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - msg.fit_start_time_sec;
                double t_pred = delta_t + msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - msg.fit_start_time_sec;
                // 角度增量积分：∫[t0, t_pred] (A sin(ωτ+φ) + B) dτ
                delta_theta = msg.spin_direction * B * delta_t
                    - (A / omega) * (std::cos(omega * t_pred + phi) - std::cos(omega * t0 + phi));
            } else {
                // 参数无效时退化为匀速
                delta_theta = msg.spin_direction * B * delta_t;
            }
        } else if (msg.is_bigbuff == -1) {
            // ---------- 小符：匀速，pi/3 弧度每秒
            delta_theta = msg.spin_direction * M_PI / 3 * delta_t;
        } else {
            delta_theta = 0.0;
        }
             // 1. 构建 R 标中心点的 3D 向量
            Eigen::Vector3d r_center(msg.r_x_3d, msg.r_y_3d, msg.r_z_3d);

            // 2. 构建当前目标的 3D 向量
            Eigen::Vector3d current_target(msg.target_x_3d, msg.target_y_3d, msg.target_z_3d);

            // 3. 计算旋转轴（法向量）：target_frame下的“相机光心 -> R标”单位向量
            Eigen::Vector3d normal_axis(msg.axis_x_3d, msg.axis_y_3d, msg.axis_z_3d);
            if (normal_axis.norm() <= eps) {
                normal_axis = r_center;
            }
            if (normal_axis.norm() <= eps) {
                geometry_msgs::msg::Point p;
                p.x = current_target.x();
                p.y = current_target.y();
                p.z = current_target.z();
                return p;
            }
            normal_axis.normalize();

            // 4. 构建“半径向量”（从 R 标指向当前目标的向量）
            Eigen::Vector3d radius_vector = current_target - r_center;

            // 5. 构造 3D 旋转器 (基于右手螺旋定则)
            // 注意：如果实车测试发现预测点沿着反方向跑了，只需要把 delta_theta 改成 -delta_theta 即可
            Eigen::AngleAxisd rotation(delta_theta, normal_axis);

            // 6. 对半径向量进行 3D 旋转
            Eigen::Vector3d pred_radius_vector = rotation * radius_vector;

            // 7. 计算最终的 3D 预测坐标 = R心坐标 + 旋转后的半径向量
            Eigen::Vector3d pred_target = r_center + pred_radius_vector;

            geometry_msgs::msg::Point p;
            p.x = pred_target.x();
            p.y = pred_target.y();
            p.z = pred_target.z();
                    return p;
                }

    // ---------- 弹道解算（考虑重力与空气阻力） ----------
    BallisticResult solveBallistic(double distance_xy, double distance_z) {
        const double v0 = bullet_speed_;
        const double g  = gravity_;
        const double k  = air_k_;
        const double dt = sim_dt_;

        // 初值：理想抛物线
        double v2 = v0 * v0;
        double v4 = v2 * v2;
        double r = std::sqrt(v4 - g * (g * distance_xy * distance_xy + 2 * distance_z * v2));
        double pitch = std::isnan(r) ? std::atan2(distance_z, distance_xy)
                                     : std::atan2(v2 - r, g * distance_xy);

        double y_aim = distance_z;
        double t_fly = 0.0;

        for (int iter = 0; iter < max_iteration_; ++iter) {
            double x = 0.0, y = 0.0, t = 0.0;
            double vx = v0 * std::cos(pitch);
            double vy = v0 * std::sin(pitch);

            // 欧拉法模拟弹道
            while (x < distance_xy) {
                double v = std::sqrt(vx*vx + vy*vy);
                double ax = -k * v * vx;
                double ay = -g - k * v * vy;
                vx += ax * dt;
                vy += ay * dt;
                x  += vx * dt;
                y  += vy * dt;
                t  += dt;
                if (t > 2.0) break;  // 防止无限循环
            }

            double error = distance_z - y;
            t_fly = t;

            if (std::abs(error) < stop_error_)
                break;

            y_aim += error;
            pitch = std::atan2(y_aim, distance_xy);
        }

        return {pitch, t_fly};
    }

    // ---------- 成员变量 ----------


    rclcpp::Subscription<buff_interfaces::msg::BuffAimingData>::SharedPtr aiming_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr autoaim_pub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAimDebug>::SharedPtr debug_pub_;

    sensor_msgs::msg::JointState::SharedPtr current_joint_msg_;

    std::string target_frame_;
    double current_yaw = 0.0;
    double current_pitch = 0.0;
    double bullet_speed_, shoot_delay_, physical_radius_;
    double gravity_, air_k_, converge_thresh_, sim_dt_, stop_error_;
    double command_limit_ = 0.1;
    double yaw_command_rate_limit_ = 0.8;
    double pitch_command_rate_limit_ = 0.8;
    double max_command_dt_ = 0.05;
    double last_yaw_cmd_ = 0.0;
    double last_pitch_cmd_ = 0.0;
    rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
    int max_iteration_;
    bool debug_;
    bool has_last_command_ = false;
};


// 组件注册
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffSolver)

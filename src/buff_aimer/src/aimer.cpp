#include <ctime>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "buff_interfaces/msg/buff_aiming_data.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "gary_msgs/msg/auto_aim_debug.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

struct BallisticResult {
    double pitch;   // 枪口绝对仰角 (rad)
    double t_fly;   // 子弹飞行时间 (s)
};

class BuffAimer : public rclcpp::Node {
public:
    explicit BuffAimer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("buff_aimer_node", options)
    {
        this->declare_parameter<std::string>("frames.target", "odom");
        this->declare_parameter<double>("ballistic.bullet_speed", 23.8);
        this->declare_parameter<double>("ballistic.shoot_delay", 0.05);
        this->declare_parameter<double>("physical_radius", 0.75);
        this->declare_parameter<double>("ballistic.gravity", 9.81);
        this->declare_parameter<double>("ballistic.air_k", 0.001);
        this->declare_parameter<double>("ballistic.converge_thresh", 0.002);
        this->declare_parameter<double>("ballistic.sim_dt", 0.005);
        this->declare_parameter<bool>("aiming.debug", true);
        this->declare_parameter<double>("aiming.max_yaw_change_speed", 0.5);
        this->declare_parameter<double>("aiming.max_pitch_change_speed", 0.5);
        this->declare_parameter<int>("max_iteration", 5);
        this->declare_parameter<double>("stop_error", 0.001);

        target_frame_    = this->get_parameter("frames.target").as_string();
        bullet_speed_    = this->get_parameter("ballistic.bullet_speed").as_double();
        shoot_delay_     = this->get_parameter("ballistic.shoot_delay").as_double();
        physical_radius_ = this->get_parameter("physical_radius").as_double();
        gravity_         = this->get_parameter("ballistic.gravity").as_double();
        air_k_           = this->get_parameter("ballistic.air_k").as_double();
        converge_thresh_ = this->get_parameter("ballistic.converge_thresh").as_double();
        sim_dt_          = this->get_parameter("ballistic.sim_dt").as_double();
        debug_           = this->get_parameter("aiming.debug").as_bool();
        max_yaw_change_speed_ = this->get_parameter("aiming.max_yaw_change_speed").as_double();
        max_pitch_change_speed_ = this->get_parameter("aiming.max_pitch_change_speed").as_double();
        max_iteration_   = this->get_parameter("max_iteration").as_int();
        stop_error_      = this->get_parameter("stop_error").as_double();

        RCLCPP_INFO(this->get_logger(),
            "BuffAimer started. bullet=%.2f m/s, radius=%.2f m", bullet_speed_, physical_radius_);
        //订阅解算后信息
        aiming_sub_ = this->create_subscription<buff_interfaces::msg::BuffAimingData>(
            "/buff/aiming_data", rclcpp::SensorDataQoS(),
            [this](const buff_interfaces::msg::BuffAimingData::SharedPtr msg) 
            {
                this->aimingCallback(msg);
            });
        //发布自瞄结果
        autoaim_pub_ = this->create_publisher<gary_msgs::msg::AutoAIM>(
            "/autoaim/target", rclcpp::SensorDataQoS());
        //发布调试信息
        if (debug_) 
        {
            debug_pub_ = this->create_publisher<gary_msgs::msg::AutoAimDebug>(
                "/autoaim/debug", rclcpp::SensorDataQoS());
        }
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/buff/aiming_markers", rclcpp::SensorDataQoS());
    }

private:
    void aimingCallback(const buff_interfaces::msg::BuffAimingData::SharedPtr msg) {
        if (!msg->is_tracking) {
            publishDeleteTargetMarkers(this->now());
            return;
        }

        if (!isValidTargetPoint(msg->target_x_3d, msg->target_y_3d, msg->target_z_3d)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Invalid aiming target point, skip publish. target=(%.3f, %.3f, %.3f)",
                msg->target_x_3d, msg->target_y_3d, msg->target_z_3d);
            publishDeleteTargetMarkers(this->now());
            return;
        }

        // 计算消息延迟
        rclcpp::Time sensor_stamp = msg->header.stamp;
        double msg_latency = (this->now() - sensor_stamp).seconds();
        if (msg_latency < 0.0) {
            msg_latency = 0.0;
            RCLCPP_WARN(this->get_logger(), "Head timestamp error Sensor time: %f, Current time: %f",
                sensor_stamp.seconds(), this->now().seconds());
        }
        geometry_msgs::msg::Point current_point;
        current_point.x = msg->target_x_3d;
        current_point.y = msg->target_y_3d;
        current_point.z = msg->target_z_3d;

        const double current_distance_xy = std::hypot(current_point.x, current_point.y);
        double distance_3d = std::hypot(current_distance_xy, current_point.z);
        double t_fly = distance_3d / std::max(bullet_speed_, 1e-3);

        BallisticResult ballistic_res = solveBallistic(current_distance_xy, current_point.z);
        pred_point_ = current_point;
        bool converged = false;

        for (int iter = 0; iter < 4; ++iter) {
            // 1. 计算火控用的预测总时间（不加控制延迟）
            double predict_delay = t_fly + msg_latency;

            // 2. 预测目标在那时在哪
            geometry_msgs::msg::Point pred_odom = predictTargetPosition(*msg, predict_delay);
            pred_point_ = pred_odom; // 供外部查询使用
            
            // 3. 对着未来位置重新解算空气阻力弹道
            double distance_xy = std::hypot(pred_odom.x, pred_odom.y);
            ballistic_res = solveBallistic(distance_xy, pred_odom.z);
            distance_3d = std::hypot(distance_xy, pred_odom.z);

            // 4. 检查 t_fly 是否收敛
            if (std::abs(t_fly - ballistic_res.t_fly) < 0.002) {
                converged = true;
                break;
            }
            t_fly = ballistic_res.t_fly;
        }
        if (!converged) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Ballistic iteration did not converge, using last result.");
        }

        const rclcpp::Time publish_time = this->now();
        double publish_yaw = std::atan2(pred_point_.y, pred_point_.x);
        double publish_pitch = ballistic_res.pitch;
        limitOutputAngles(publish_yaw, publish_pitch, publish_time);

        // 发布自瞄指令
        auto autoaim_msg = std::make_unique<gary_msgs::msg::AutoAIM>();
        autoaim_msg->yaw   = static_cast<float>(publish_yaw);
        autoaim_msg->pitch = -static_cast<float>(publish_pitch);
        autoaim_msg->target_distance = static_cast<float>(distance_3d);
        autoaim_msg->header.stamp = publish_time;
        autoaim_msg->header.frame_id = target_frame_;
        autoaim_msg->shoot_command = gary_msgs::msg::AutoAIM::ALLOW_SHOOT;
        autoaim_msg->shoot_mode = gary_msgs::msg::AutoAIM::SHOOT_MODE_AUTO;
        autoaim_pub_->publish(std::move(autoaim_msg));
        publishTargetMarkers(*msg, pred_point_, publish_time);

        // 调试信息
        if (debug_) {
            auto debug_msg = gary_msgs::msg::AutoAimDebug();
            //只发预测点
            debug_msg.pred_point = pred_point_;
            debug_pub_->publish(debug_msg);
            }
        }

    visualization_msgs::msg::Marker makePointMarker(
        const std::string& ns,
        int id,
        const rclcpp::Time& stamp,
        const geometry_msgs::msg::Point& point,
        float r,
        float g,
        float b,
        double scale) const
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = target_frame_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.9f;
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 250000000;
        return marker;
    }

    void publishTargetMarkers(
        const buff_interfaces::msg::BuffAimingData& msg,
        const geometry_msgs::msg::Point& pred_point,
        const rclcpp::Time& stamp) const
    {
        geometry_msgs::msg::Point current_point;
        current_point.x = msg.target_x_3d;
        current_point.y = msg.target_y_3d;
        current_point.z = msg.target_z_3d;

        auto markers = visualization_msgs::msg::MarkerArray();
        markers.markers.emplace_back(makePointMarker(
            "buff_current_target", 0, stamp, current_point, 0.1f, 0.9f, 0.2f, 0.08));
        markers.markers.emplace_back(makePointMarker(
            "buff_predicted_target", 1, stamp, pred_point, 1.0f, 0.25f, 0.05f, 0.10));

        auto line = visualization_msgs::msg::Marker();
        line.header.frame_id = target_frame_;
        line.header.stamp = stamp;
        line.ns = "buff_prediction_delta";
        line.id = 2;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.02;
        line.color.r = 0.2f;
        line.color.g = 0.7f;
        line.color.b = 1.0f;
        line.color.a = 0.7f;
        line.points.push_back(current_point);
        line.points.push_back(pred_point);
        line.lifetime.sec = 0;
        line.lifetime.nanosec = 250000000;
        markers.markers.emplace_back(line);

        marker_pub_->publish(markers);
    }

    void publishDeleteTargetMarkers(const rclcpp::Time& stamp) const
    {
        if (!marker_pub_) {
            return;
        }

        auto markers = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = target_frame_;
        marker.header.stamp = stamp;
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.emplace_back(marker);
        marker_pub_->publish(markers);
    }

    static bool isValidTargetPoint(double x, double y, double z) {
        constexpr double kMinDistance = 1e-6;
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
            && std::hypot(x, y) > kMinDistance;
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    static double limitLinearStep(double current, double last, double max_speed, double dt) {
        if (max_speed <= 0.0 || dt <= 0.0) {
            return current;
        }
        const double max_step = max_speed * dt;
        const double step = current - last;
        if (step > max_step) {
            return last + max_step;
        }
        if (step < -max_step) {
            return last - max_step;
        }
        return current;
    }

    void limitOutputAngles(double& yaw, double& pitch, const rclcpp::Time& now) {
        if (has_last_output_) {
            const double dt = (now - last_output_time_).seconds();
            if (dt > 0.0 && max_yaw_change_speed_ > 0.0) {
                const double yaw_step = normalizeAngle(yaw - last_output_yaw_);
                const double max_yaw_step = max_yaw_change_speed_ * dt;
                if (yaw_step > max_yaw_step) {
                    yaw = normalizeAngle(last_output_yaw_ + max_yaw_step);
                } else if (yaw_step < -max_yaw_step) {
                    yaw = normalizeAngle(last_output_yaw_ - max_yaw_step);
                }
            }

            pitch = limitLinearStep(pitch, last_output_pitch_, max_pitch_change_speed_, dt);
        }

        last_output_time_ = now;
        last_output_yaw_ = yaw;
        last_output_pitch_ = pitch;
        has_last_output_ = true;
    }

    //计算目标未来位置
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
                // 角速度拟合的是幅值，方向由 spin_direction 统一决定。
                //计算角度差
                const double speed_integral = B * delta_t
                    - (A / omega) * (std::cos(omega * t_pred + phi) - std::cos(omega * t0 + phi));
                delta_theta = msg.spin_direction * speed_integral;
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
            Eigen::Vector3d normal_axis(msg.r_x_3d, msg.r_y_3d, 0);
            //Eigen::Vector3d normal_axis(msg.axis_x_3d, msg.axis_y_3d, msg.axis_z_3d);
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
            radius_vector.normalized()*physical_radius_;
            // 5. 构造 3D 旋转器 (基于右手螺旋定则)
            // delta_theta 已按 spin_direction 带符号，旋转方向遵循右手定则（顺时针+逆时针-）
            Eigen::AngleAxisd rotation(delta_theta, normal_axis);

            // 6. 对半径向量进行 3D 旋转
            Eigen::Vector3d pred_radius_vector = rotation * radius_vector;

            // 7. 计算最终的 3D 预测坐标 = R心坐标 + 旋转后的半径向量
            Eigen::Vector3d pred_target = r_center + pred_radius_vector;

            geometry_msgs::msg::Point predicted_point;
            predicted_point.x = pred_target.x();
            predicted_point.y = pred_target.y();
            predicted_point.z = pred_target.z();
                
            return predicted_point;
    }

    // ---------- 弹道解算（考虑重力与空气阻力） ----------
    BallisticResult solveBallistic(double distance_xy, double distance_z) {
        // 第一步：理想抛物线模型热启动
        double v2 = bullet_speed_ * bullet_speed_;
        double v4 = v2 * v2;
        double r = std::sqrt(v4 - gravity_ * (gravity_ * distance_xy * distance_xy + 2 * distance_z * v2));

        double pitch;
        if (std::isnan(r)) {
            pitch = std::atan2(distance_z, distance_xy);
        } else {
            pitch = std::atan2(v2 - r, gravity_ * distance_xy);
        }

        double y_aim = distance_z;
        double actual_t_fly = 0.0;

        // 第二步：空气阻力迭代补偿
        for (int iter = 0; iter < max_iteration_; ++iter) {
            double x = 0.0, y = 0.0, t = 0.0;
            double vx = bullet_speed_ * std::cos(pitch);
            double vy = bullet_speed_ * std::sin(pitch);
            double dt = 0.005;

            while (x < distance_xy) {
                double v = std::sqrt(vx*vx + vy*vy);
                vx += (-air_k_ * v * vx) * dt;
                vy += (-gravity_ - air_k_ * v * vy) * dt;
                x  += vx * dt;
                y  += vy * dt;
                t  += dt;
                if (t > 2.0) break;
            }

            double error = distance_z - y;
            actual_t_fly = t;

            if (std::abs(error) < stop_error_)
                break;

            y_aim += error;
            pitch = std::atan2(y_aim, distance_xy);
        }

        return {-pitch, actual_t_fly};
    }

    rclcpp::Subscription<buff_interfaces::msg::BuffAimingData>::SharedPtr aiming_sub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAIM>::SharedPtr autoaim_pub_;
    rclcpp::Publisher<gary_msgs::msg::AutoAimDebug>::SharedPtr debug_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    //debug用
    geometry_msgs::msg::Point pred_point_;
    
    std::string target_frame_;
    double bullet_speed_, shoot_delay_, physical_radius_;
    double gravity_, air_k_, converge_thresh_, sim_dt_, stop_error_;
    double max_yaw_change_speed_, max_pitch_change_speed_;
    rclcpp::Time last_output_time_;
    double last_output_yaw_ = 0.0;
    double last_output_pitch_ = 0.0;
    int max_iteration_;
    bool debug_;
    bool has_last_output_ = false;
};

// 组件注册
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BuffAimer)

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

struct pid_config {
    double PID_KP = 0.8f;
    double PID_KD = 0.0f;
    double PID_KI = 0.1f;
    double I_max = 100.0f;
    double I_min = -100.0f;
};

// PID 控制器类
class PIDController {
   public:
    pid_config config{};

    // 存储当前的误差、误差的累计值和误差的变化率
    double prevError = 0.0;
    double integral = 0.0;
    double target_angle = 0.0;

    // 构造函数，初始化增益系数
    PIDController(pid_config&& conf) : config(conf) {}
    PIDController() = default;

    // 计算PID输出
    double compute(double setpoint, double currentValue) {
        // 计算误差
        double error = setpoint - currentValue;

        // 积分项：累积误差
        integral += error;
        const double lower_bound = std::min(config.I_min, config.I_max);
        const double upper_bound = std::max(config.I_min, config.I_max);
        integral = std::clamp(integral, lower_bound, upper_bound);

        // 微分项：误差的变化率
        double derivative = error - prevError;

        // PID控制公式
        double output =
            config.PID_KP * error + config.PID_KI * integral + config.PID_KD * derivative;
        output = std::max(-180.0, std::min(180.0, output));  // 限制输出范围

        // 更新前一个误差
        prevError = error;

        return output;
    }

    // 重置PID控制器的状态（误差、积分项等）
    void reset() {
        prevError = 0.0;
        integral = 0.0;
    }

    void update_config(const pid_config&& new_config) { config = new_config; }
};

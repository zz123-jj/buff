#include "predictor.hpp"
#include <ceres/ceres.h>

namespace {

struct VelocitySineResidual {
    VelocitySineResidual(double t, double y, double fit_offset_sum)
        : t_(t), y_(y), fit_offset_sum_(fit_offset_sum) {}

    template <typename T>
    bool operator()(const T* const A, const T* const omega, const T* const phi, T* residual) const {
        const T prediction =
            A[0] * ceres::sin(omega[0] * T(t_) + phi[0]) + (T(fit_offset_sum_) - A[0]);
        residual[0] = prediction - T(y_);
        return true;
    }

private:
    double t_;
    double y_;
    double fit_offset_sum_;
};

}  // namespace

Big_Buff_Predictor::Big_Buff_Predictor() {}

void Big_Buff_Predictor::reset() {
    time_w_pairs_.clear();
    fit_attempted_ = false;
    fit_ready_ = false;
    velocity_fit_params_.setZero();
}

bool Big_Buff_Predictor::try_fit_once_at_1p5s() {
    if (fit_attempted_) {
        return fit_ready_;
    }

    if (time_w_pairs_.size() < static_cast<size_t>(fit_min_samples_)) {
        return false;
    }

    if (time_w_pairs_.back().first < fit_window_sec_-0.3) {
        return false;
    }
 
    
    fit_attempted_ = true;


    double A = 0.5 * (a_lower_ + a_upper_);
    double omega = 0.5 * (omega_lower_ + omega_upper_);
    double phi = 0.0;

    ceres::Problem problem;
    for (const auto& sample : time_w_pairs_) {
        ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<VelocitySineResidual, 1, 1, 1, 1>(
                new VelocitySineResidual(sample.first, static_cast<double>(sample.second), fit_offset_sum_));
        problem.AddResidualBlock(cost, nullptr, &A, &omega, &phi);
    }

    problem.SetParameterLowerBound(&A, 0, a_lower_);
    problem.SetParameterUpperBound(&A, 0, a_upper_);
    problem.SetParameterLowerBound(&omega, 0, omega_lower_);
    problem.SetParameterUpperBound(&omega, 0, omega_upper_);
    problem.SetParameterLowerBound(&phi, 0, phi_lower_);
    problem.SetParameterUpperBound(&phi, 0, phi_upper_);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 200;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    const bool success = summary.IsSolutionUsable() &&
                         std::isfinite(A) && std::isfinite(omega) && std::isfinite(phi);

    if (!success) {
        fit_ready_ = false;
        return false;
    }

    velocity_fit_params_[0] = static_cast<float>(A);
    velocity_fit_params_[1] = static_cast<float>(omega);
    velocity_fit_params_[2] = static_cast<float>(phi);
    velocity_fit_params_[3] = static_cast<float>(fit_offset_sum_ - A);
    fit_ready_ = true;

    return true;
}

//两点之间的欧氏距离
float euclidean_distance(cv::Point2f p1, cv::Point2f p2){
    float dx = p1.x - p2.x;  
    float dy = p1.y - p2.y;  
    return std::sqrt(dx * dx + dy * dy);  
}

//将笛卡尔坐标(x,y)转换为角度（弧度）
//返回范围：[0, 2π]，其中0°指向x轴正方向，逆时针增加
float trans(float x, float y) {
    // 特殊情况：x=0时，点在y轴上
    if (x == 0.0f) {
        if (y > 0) {
            return M_PI / 2.0f;  // 90° (y轴正方向)
        } else if (y < 0) {
            return 3.0f * M_PI / 2.0f;  // 270° (y轴负方向)
        } else {
            return 0.0f;  // 原点
        }
    }
    // 使用atan计算基础角度（返回范围：-π/2 到 π/2）
    float angle = std::atan(y / x);
    // 根据点所在的象限调整角度
    if (x > 0 && y > 0) {
        // 第一象限：角度已经正确，直接返回
        return angle;
    } else if (x < 0) {
        // 第二、三象限：需要加π
        // 因为atan只能区分上下，不能区分左右
        return angle + M_PI;
    } else {
        // 第四象限和x轴正方向 (x > 0 && y <= 0)
        if (y == 0.0f) {
            return 0.0f;  // x轴正方向，角度为0
        }
        // 第四象限：需要加2π，使角度为正
        return angle + 2.0f * M_PI;
    }
}

//跟踪旋转目标的角度，解决角度跳变问题
// 1. 角度连续性：从359°跳到0°时，需要识别为360°而不是跳回0
// 2. 装甲板跳变：能量机关有5个装甲板，视觉可能突然跳到另一个板
angleObserver::angleObserver(clockMode mode) 
    : mode_(mode),                     // 旋转方向（顺时针/逆时针）
      last_point_(-1.0f, -1.0f),       // 上一帧的坐标
      blade_jump_count_(0),            // 累计的装甲板跳变次数
    is_first_angle_(true),           // 是否是第一次计算角度
    last_angle_(0.0f) {}             // 上一帧的角度

//向量旋转函数
//正值theta逆时针旋转 负值则顺时针旋转
Vector2f angleObserver::Rotation(float theta, Vector2f vector) {
    Rotation2Df rotator(theta);  // 创建旋转矩阵
    return rotator * vector;  
}

//角度连续化处理
//处理从175°到-175°的类似情况
float angleObserver::AngleTransformer(float x, float y) {
    float current_angle = std::atan2(y, x);  // 返回 [-π, π]
    // 第一次调用，直接保存并返回
    if (is_first_angle_) {
        last_angle_ = current_angle;
        is_first_angle_ = false;
        return current_angle;
    }
    // 计算角度差
    float angle_diff = current_angle - last_angle_;
    // 将角度差归一化到 [-π, π] 范围
    // 这样可以选择最短的旋转路径
    while (angle_diff > M_PI) {
        angle_diff -= 2.0f * M_PI;
    }
    while (angle_diff < -M_PI) {
        angle_diff += 2.0f * M_PI;
    }
    // 累加角度差，保持连续性
    float continuous_angle = last_angle_ + angle_diff;
    // 更新记录
    last_angle_ = continuous_angle;
    return continuous_angle;
}

clockMode angleObserver::getClockMode() const {
    return mode_;
}

void angleObserver::setClockMode(clockMode mode) {
    if (mode_ != mode) {
        blade_jump_count_ = 0;
    }
    mode_ = mode;
}

void angleObserver::reset() {
    mode_ = clockMode::unknown;
    last_point_ = cv::Point2f(-1.0f, -1.0f);
    blade_jump_count_ = 0;
    is_first_angle_ = true;
    last_angle_ = 0.0f;
}

//x, y: 目标的当前坐标（相对于旋转中心）raduis: 旋转半径
//检查是否发生72度倍数的跳变 如果跳变则反向旋转坐标进行校正
//返回连续化后的角度（弧度）
float angleObserver::update(float target_x, float target_y, float raduis) {
    const int NUM_BLADE_PLATES = 5;                                         //扇叶个数
    const float BLADE_ANGLE_INTERVAL = 2.0f * M_PI / NUM_BLADE_PLATES;  //扇叶间距72° = 2π/5
    
    // 1. 初始化第一帧
    bool first_call = (last_point_.x == -1.0f && last_point_.y == -1.0f);
    if (first_call) {
        last_point_.x = target_x;
        last_point_.y = target_y;
        return AngleTransformer(target_x, target_y);
    }
    
    // 2. 方向未知时仅做角度连续化，不做依赖方向符号的跳变修正
    if (mode_ == clockMode::unknown) {
        float current_raw_angle = AngleTransformer(target_x, target_y);
        last_point_.x = target_x;
        last_point_.y = target_y;
        return current_raw_angle;
    }

    // 根据旋转方向确定旋转角度的符号
    float rotation_sign = (mode_ == anticlockwise) ? -1.0f : 1.0f;

    // 步骤2：如果之前发生过装甲板跳变，需要先对当前坐标进行校正
    if (blade_jump_count_ != 0) {
        Vector2f rotated = Rotation(-rotation_sign * BLADE_ANGLE_INTERVAL * blade_jump_count_, Vector2f(target_x, target_y));
        target_x = rotated[0];
        target_y = rotated[1];
    }
    // 步骤3：检测是否发生了装甲板跳变
    // 判断依据：如果当前位置和上一帧位置的距离大于半径，可能跳板了（实际计算最小跳变的间距约为1.176R）
    float distance_threshold = raduis;
    if (euclidean_distance(last_point_, cv::Point2f(target_x, target_y)) > distance_threshold) {
        // 生成5个可能的装甲板位置
        // 方法：根据旋转方向，将上一帧位置旋转0°, ±72°, ±144°, ±216°, ±288°
        std::vector<Vector2f> candidate_positions; //候选点
        for (int i = 0; i < NUM_BLADE_PLATES; ++i) {
            Vector2f rotated_pos = Rotation(rotation_sign * BLADE_ANGLE_INTERVAL * i, Vector2f(last_point_.x, last_point_.y));
            candidate_positions.push_back(rotated_pos);
        }
        // 找到距离当前点最近的装甲板（说明跳到了这个板）
        float min_distance = std::numeric_limits<float>::max(); //获取float类型可表示的最大值
        int closest_armor_index = 0;
        for (int i = 0; i < NUM_BLADE_PLATES; ++i) {
            float dist = euclidean_distance(cv::Point2f(target_x, target_y), cv::Point2f(candidate_positions[i][0], candidate_positions[i][1]));
            if (dist < min_distance) {
                min_distance = dist;
                closest_armor_index = i;
            }
        }
        // 将当前坐标旋转回跳变之后的位置
        Vector2f corrected_pos = Rotation(-rotation_sign * BLADE_ANGLE_INTERVAL * (closest_armor_index), Vector2f(target_x, target_y));
        target_x = corrected_pos[0];
        target_y = corrected_pos[1];
        blade_jump_count_ += closest_armor_index;  // 记录累计跳变次数
    }
    // 步骤4：计算角度（已经保证连续性）
    float continuous_angle = AngleTransformer(target_x, target_y);
    // 步骤5：更新上一帧位置，供下次调用使用
    last_point_.x = target_x;
    last_point_.y = target_y;
    return continuous_angle;
}



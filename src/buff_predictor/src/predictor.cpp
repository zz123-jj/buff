#include "predictor.hpp"

Big_Buff_Predictor::Big_Buff_Predictor():
    Avg_filter_(5),                                    // 创建平滑滤波器
    frame_counter_(0),                                 // 当前帧计数
    start_detector_(1.5),                              // 使用1.5秒作为等待时间阈值
    canStart_(false),                                  // 是否开始标识位
    is_get_para_(false),                               // 是否已有拟合参数
    sin_para_(5)                                       // 准备存储5个拟合参数: A, ω, φ, B, C
    {}

//正弦拟合函数
VectorXf Big_Buff_Predictor::sinFit(const VectorXf& x, const VectorXf& y) {
    if (x.size() < 4) return VectorXf::Zero(5); // 保护：数据点太少无法拟合

    //1.差分（使用时间间隔归一化：dy/dt）
    VectorXf dy(x.size() - 1);
    for (int i = 0; i < x.size() - 1; ++i) {
        const float dt_i = x[i + 1] - x[i];
        if (dt_i > 1e-6f) {
            dy[i] = (y[i + 1] - y[i]) / dt_i;
        } else {
            dy[i] = 0.0f;
        }
    }
    diff_datas_ = dy;  // 保存原始差分数据
   
    //2.中值滤波
    MedianFilter median_filter(5);
    VectorXf dy_filtered(dy.size());
    for (int i = 0; i < (int)dy.size(); ++i) {
        dy_filtered[i] = median_filter.update(dy[i]);
    }
    diff_smoothed_datas_ = dy_filtered;
    
    //3.用FFT估算周期参数
    // 跳过前5个点，避免边界效应
    int skip_points = std::min(5, (int)dy_filtered.size() / 4);
    VectorXf dy_for_fft = dy_filtered.tail(dy_filtered.size() - skip_points);
    FFT<float> fft;
    std::vector<std::complex<float>> freq_domain;
    std::vector<float> time_domain(dy_for_fft.size());
    VectorXf::Map(&time_domain[0], dy_for_fft.size()) = dy_for_fft;
    fft.fwd(freq_domain, time_domain);
    // 在频域中找到幅值最大的频率（主频率）
    float max_mag = -1.0f;
    int max_index = -1;
    for (int i = 1; i < (int)dy_for_fft.size() / 2; ++i) {
        float mag = std::abs(freq_domain[i]);
        if (mag > max_mag) {
            max_mag = mag;
            max_index = i;
        }
    }
    // 根据找到的主频率索引，计算实际的频率值
    float dt_sum = 0.0f;
    int dt_count = 0;
    for (int i = 1; i < (int)x.size(); ++i) {
        float dti = x[i] - x[i - 1];
        if (dti > 1e-6f) {
            dt_sum += dti;
            dt_count++;
        }
    }
    float dt = (dt_count > 0) ? (dt_sum / dt_count) : 1.0f;
    float freq_hz = (float)max_index / (dy_for_fft.size() * dt);
    float w_init = 2.0f * M_PI * freq_hz;  // 角频率初始估计
    
    // 4.对差分数据拟合正弦函数 dy = a*sin(ωx+φ) + b
    // 构造对应的x坐标（使用中点时间）
    VectorXf x_dy(x.size() - 1);
    for (int i = 0; i < (int)x.size() - 1; ++i) {
        x_dy[i] = 0.5f * (x[i] + x[i + 1]);
    }
    // 估算线性趋势 b（差分数据的平均值）
    float sum_dy = 0.0f;
    for (int i = 0; i < (int)dy_filtered.size(); ++i) {
        sum_dy += dy_filtered[i];
    }
    float b_init = sum_dy / dy_filtered.size();
    // 去除线性趋势
    VectorXf dy_detrend(dy_filtered.size());
    for (int i = 0; i < (int)dy_filtered.size(); ++i) {
        dy_detrend[i] = dy_filtered[i] - b_init;
    }
    // 估算振幅 a
    float a_init = (dy_detrend.maxCoeff() - dy_detrend.minCoeff()) / 2.0f;
    
    //5.定义拟合函数 dy = a*sin(ωx+φ) + b 
    struct DiffSinusoidalFunctor {
        typedef double Scalar;
        typedef VectorXd InputType;
        typedef VectorXd ValueType;
        typedef Matrix<Scalar, Dynamic, Dynamic> JacobianType;
        enum {
            InputsAtCompileTime = Dynamic,
            ValuesAtCompileTime = Dynamic
        };
        VectorXd m_x;
        VectorXd m_y;
        int m_inputs, m_values;
        DiffSinusoidalFunctor(const VectorXf& x, const VectorXf& y) 
            : m_x(x.cast<double>()), 
              m_y(y.cast<double>()), 
              m_inputs(4),  // 4个参数：a, ω, φ, b
              m_values(x.size()) {}
        int inputs() const { return m_inputs; }
        int values() const { return m_values; }
        int operator()(const VectorXd &p, VectorXd &fvec) const {
            double a = p[0];      // 振幅
            double omega = p[1];  // 角频率
            double phi = p[2];    // 相位
            double b = p[3];      // 线性系数
            for (int i = 0; i < values(); ++i) {
                // dy = a*sin(ωx+φ) + b
                double dy_pred = a * std::sin(omega * m_x[i] + phi) + b;
                fvec[i] = dy_pred - m_y[i];
            }
            return 0;
        }
    };
    // 初始化4个参数 [a, ω, φ, b]
    VectorXd lm_para(4);
    lm_para[0] = static_cast<double>(a_init);       // a: 振幅
    lm_para[1] = static_cast<double>(w_init);       // ω: 角频率
    lm_para[2] = 0.0;                               // φ: 相位初始为0
    lm_para[3] = static_cast<double>(b_init);       // b: 线性系数
    
    //6.用LM算法拟合
    DiffSinusoidalFunctor diff_functor(x_dy, dy_filtered);
    NumericalDiff<DiffSinusoidalFunctor> diff_numDiff(diff_functor);
    LevenbergMarquardt<NumericalDiff<DiffSinusoidalFunctor>> lm_diff(diff_numDiff);
    lm_diff.parameters.maxfev = 12000;
    lm_diff.parameters.xtol = 1e-7;
    lm_diff.minimize(lm_para);
    sin_para_ = VectorXf(4);
    sin_para_[0] = static_cast<float>(lm_para[0]);  // a: 振幅
    sin_para_[1] = static_cast<float>(lm_para[1]);  // ω: 角频率
    sin_para_[2] = static_cast<float>(lm_para[2]);  // φ: 相位
    sin_para_[3] = static_cast<float>(lm_para[3]);  // b: 线性系数
    
    // 7.通过不定积分转换为位移函数参数
    // dy/dx = a*sin(ωx+φ) + b
    // 即： y = A*cos(ωx+φ) + Bx + C
    // 其中： A = -a/ω, B = b, ω不变, φ不变
    double a = lm_para[0];
    double omega = lm_para[1];
    double phi = lm_para[2];
    double b = lm_para[3];
    double A = -a / omega;  // 积分后的余弦振幅
    double B = b;           // 线性系数不变
    // 用第一个数据点确定常数项 C
    // y[0] = A*cos(ω*x[0]+φ) + B*x[0] + C
    // C = y[0] - A*cos(ω*x[0]+φ) - B*x[0]
    double C = y[0] - A * std::cos(omega * x[0] + phi) - B * x[0];
    
    // 8.组装最终的5个参数 [A, ω, φ, B, C]
    cos_para_ = VectorXf(5);
    cos_para_[0] = static_cast<float>(A);
    cos_para_[1] = static_cast<float>(omega);
    cos_para_[2] = static_cast<float>(phi);
    cos_para_[3] = static_cast<float>(B);
    cos_para_[4] = static_cast<float>(C);
    
    VectorXf result(5);
    result[0] = static_cast<float>(A);
    result[1] = static_cast<float>(omega);
    result[2] = static_cast<float>(phi);
    result[3] = static_cast<float>(B);
    result[4] = static_cast<float>(C);
    
    return result;
}

//返回值：pair<是否成功预测, 角度增量（不再进行角度预测，此处恒返回0.0f）>
std::pair<bool, float> Big_Buff_Predictor::update(float continues_angle, double stamp_sec) { 
    //1.对输入数据进行平滑处理
    float smoothed_angle = Avg_filter_.update(continues_angle);
    //2.保存平滑后的数据
    smoothed_angles_.push_back(smoothed_angle);
    frame_counter_++;  // 帧计数器加1

    if (start_time_sec_ < 0.0) {
        start_time_sec_ = stamp_sec;
    }
    double t = stamp_sec - start_time_sec_;

    // Protect against non-increasing timestamps to avoid zero/negative dt.
    if (!time_stamps_.empty() && t <= time_stamps_.back() + 1e-6) {
        smoothed_angles_.pop_back();
        frame_counter_--;
        return {false, 0.0f};
    }

    last_time_sec_ = stamp_sec;
    time_stamps_.push_back(t);
    
    //3.判断是否可以开始拟合
    if (!canStart_) {
        canStart_ = start_detector_.update(stamp_sec, true);  // 传入时间戳和tracking状态
    }
    
    //4.如果已经可以开始拟合，进行预测
    if (canStart_) {
        // 构造时间序列（秒）
        VectorXf time_seq(smoothed_angles_.size());
        for (size_t i = 0; i < smoothed_angles_.size(); ++i) {
            time_seq[i] = static_cast<float>(time_stamps_[i]);
        }
        // 将角度数据转换为Eigen向量
        VectorXf smoothed_vector_datas = VectorXf::Map(smoothed_angles_.data(), smoothed_angles_.size());
        

        sin_para_ = sinFit(time_seq, smoothed_vector_datas);  // 拟合积分形式正弦函数
        is_get_para_ = true;
        
        // 拟合成功后返回参数状态，不再计算未来角度
        if (is_get_para_) {
            return {true, 0.0f};
        }
    }
    
    // 如果还没收集够数据或还没开始拟合，返回失败
    return {false, 0.0f};
}

smallPredictor::smallPredictor()
    : window_time_(1.5f),           // 窗口时间：1.5秒用于计算角速度
      angular_velocity_(0.0f) {
    // 基于时间的实现，不需要帧数
}

void smallPredictor::reset() {
    angles_.clear();
    timestamps_.clear();
    angular_velocity_ = 0.0f;
}

// std::pair<bool, float> smallPredictor::update(float data) {

//     // 添加新数据到序列
//     y_.push_back(data);

//     if (y_.size() > static_cast<size_t>(win_size_)) {
//         y_.erase(y_.begin());
//     }

//     // 3. 数据不足时，无法预测，返回失败
//     if (y_.size() < static_cast<size_t>(win_size_)) {
//         return {false, 0.0f};
//     }

//     // 4. 计算平均角速度 (rad/frame)
//     // 逻辑：(最新角度 - 最旧角度) / 间隔帧数
//     // 这比累加每一帧的 diff 更快且误差更小
//     float total_diff = y_.back() - y_.front();
    
//     // 更新成员变量
//     pred_ = total_diff / (y_.size() - 1);

//     // 5. 计算预测量
//     // 预测增量 = 平均速度 * 预测帧数 (即 win_size_)

//     if (std::abs(pred_ * win_size_) < 0.5f && std::abs(pred_) > 0.0001f) {
//         return {true, pred_ * win_size_};
//     }

//     return {false,0.0f};
// }

std::pair<bool, float> smallPredictor::update(float angle, double timestamp) {
    // 如果已经锁定预测，直接返回状态
    if (std::abs(angular_velocity_) > 0.0001f) {
        return {true, 0.0f};
    }
    
    // 数据入队
    angles_.push_back(angle);
    timestamps_.push_back(timestamp);

    // 维持时间窗口：移除超过 window_time_ 的旧数据
    while (angles_.size() > 1 && (timestamps_.back() - timestamps_.front()) > window_time_) {
        angles_.erase(angles_.begin());
        timestamps_.erase(timestamps_.begin());
    }

    // 数据不足（至少需要2个点）
    if (angles_.size() < 2) {
        return {false, 0.0f};
    }

    // 计算时间跨度
    double time_span = timestamps_.back() - timestamps_.front();
    if (time_span < 0.1) {  // 时间跨度太小
        return {false, 0.0f};
    }

    // 计算当前窗口内的角速度（rad/s）
    float angle_diff = angles_.back() - angles_.front();
    float current_velocity = angle_diff / static_cast<float>(time_span);
    
    // 判断角速度是否在一个合理范围内
    if (std::abs(current_velocity) > 0.1f && std::abs(current_velocity) < 10.0f) {
        angular_velocity_ = current_velocity;  // 锁定角速度
        return {true, 0.0f};
    }
    
    // 预测量过大，重置数据
    if (std::abs(current_velocity) >= 10.0f) {
        angles_.clear();
        timestamps_.clear();
        angles_.push_back(angle);
        timestamps_.push_back(timestamp);
    }

    // 没锁定时，返回 0
    return {false, 0.0f};
}


//收集足够的数据点后开始拟合
// fps: 相机帧率(fps)
// required_time: 用户设定的时间阈值（秒），默认1.5秒
FitStartDetect::FitStartDetect(double required_time):
    required_time_(required_time),
    first_timestamp_(-1.0),
    has_first_stamp_(false)
{
}

// 更新时间戳，判断是否可以开始拟合
// current_timestamp: 当前时间戳（秒）
// is_tracking: 是否处于tracking状态
// 返回true表示已经达到时间阈值，可以开始拟合
bool FitStartDetect::update(double current_timestamp, bool is_tracking) {
    // 如果不在tracking，重置状态
    if (!is_tracking) {
        reset();
        return false;
    }
    
    // 如果是第一次tracking，记录时间戳
    if (!has_first_stamp_) {
        first_timestamp_ = current_timestamp;
        has_first_stamp_ = true;
        return false;
    }
    
    // 计算时间差
    double elapsed_time = current_timestamp - first_timestamp_;
    
    // 判断是否达到时间阈值
    if (elapsed_time >= required_time_) {
        return true;
    }
    
    return false;
}

// 重置检测器
void FitStartDetect::reset() {
    first_timestamp_ = -1.0;
    has_first_stamp_ = false;
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
      last_angle_(0.0f),               // 上一帧的角度
      blade_jump_count_(0),            // 累计的装甲板跳变次数
      is_first_angle_(true) {}         // 是否是第一次计算角度

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
    
    // 2. 如果方向是未知的，尝试进行方向判定
    if (mode_ == clockMode::unknown) {
        // 在没有跳变时（两帧之间距离在合理范围内），采集角度增量
        float dist = euclidean_distance(last_point_, cv::Point2f(target_x, target_y));
        if (dist < raduis * 0.8f) { // 小于半径认为没有发生跳变
            float current_raw_angle = AngleTransformer(target_x, target_y); // 获取连续化角度
            float angle_diff = current_raw_angle - last_angle_;
            total_angle_diff_ += angle_diff;
            direction_detect_count_++;
            
            // 累计采集到 5~10 帧数据后，再做出判断
            if (direction_detect_count_ > 5) { 
                if (total_angle_diff_ > 0) {
                    mode_ = clockMode::anticlockwise; // 角度变大，逆时针
                } else {
                    mode_ = clockMode::clockwise;     // 角度变小，顺时针
                }
            }
            
            last_point_.x = target_x;
            last_point_.y = target_y;
            return current_raw_angle;
        } else {
            // 如果在方向未确定的时候就发生跳变，可以丢弃此帧或者重置判定
            return AngleTransformer(target_x, target_y); 
        }
    }

    // 根据旋转方向确定旋转角度的符号
    // 逆时针：正角度；顺时针：负角度
    float rotation_sign = (mode_ == anticlockwise) ? 1.0f : -1.0f;

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

// ==================== Predictor3D类实现 ====================

/**
 * 构造函数
 */
Predictor3D::Predictor3D(const CameraParams& cam, float radius_world)
    : cam_(cam), radius_world_(radius_world) {
    std::cout << "[Predictor3D] 初始化完成" << std::endl;
    std::cout << "  相机参数: fx=" << cam_.fx << ", fy=" << cam_.fy 
              << ", cx=" << cam_.cx << ", cy=" << cam_.cy << std::endl;
    std::cout << "  目标半径: " << radius_world_ << "m" << std::endl;
}

/**
 * 通过已知半径计算深度
 * 原理：radius_world / Z = radius_pixel / fx
 * 解出：Z = (radius_world × fx) / radius_pixel
 */
float Predictor3D::compute_depth(float radius_pixel) const {
    if (radius_pixel <= 0) {
        // std::cerr << "[Predictor3D警告] radius_pixel <= 0, 返回默认深度1.0m" << std::endl;
        return 1.0f;
    }
    return (radius_world_ * cam_.fx) / radius_pixel;
}

/**
 * 像素坐标转相机坐标系3D点
 * 使用针孔相机模型的反投影公式
 */
Point3D Predictor3D::pixel_to_3d(float u, float v, float depth) const {
    Point3D p;
    p.z = depth;
    // 针孔相机模型反投影：
    // X = (u - cx) * Z / fx
    // Y = (v - cy) * Z / fy
    p.x = (u - cam_.cx) * depth / cam_.fx;
    p.y = (v - cam_.cy) * depth / cam_.fy;
    return p;
}

/**
 * 预测未来3D位置
 */
Point3D Predictor3D::predict_3d_position(float current_angle, float angle_increment,
                                         float center_u, float center_v,
                                         float radius_pixel) const {
    // 1. 计算预测角度（当前角度 + 增量）
    float predicted_angle = current_angle + angle_increment;
    
    // 2. 根据预测角度计算目标的像素坐标
    // 假设旋转中心为(center_u, center_v)，半径为radius_pixel
    float pred_u = center_u + radius_pixel * std::cos(predicted_angle);
    float pred_v = center_v + radius_pixel * std::sin(predicted_angle);
    
    // 3. 通过已知半径计算深度
    float depth = compute_depth(radius_pixel);
    
    // 4. 将像素坐标和深度转换为相机坐标系3D点
    return pixel_to_3d(pred_u, pred_v, depth);
}
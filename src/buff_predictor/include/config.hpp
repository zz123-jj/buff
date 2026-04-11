#pragma once

// 预测参数
static constexpr double MIN_OMEGA = 1.884;          // 最小角速度
static constexpr double FIT_START_TIME = 1.5;      // 开始拟合等待时间（秒）

// 物理参数
static constexpr double PHYSICAL_ARM_LENGTH = 1.5;  // 目标到中心的臂长（米）
static constexpr double PHYSICAL_R_RADIUS = 0.1;    // R标半径（米）

// 调试参数
static constexpr bool DEBUG_MODE = true;           // 是否启用debug模式（CSV记录等）

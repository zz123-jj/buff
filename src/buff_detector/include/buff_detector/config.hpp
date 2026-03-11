#pragma once

#include <opencv2/opencv.hpp>

// YOLO
inline const std::string MODEL_PATH = "src/buff_detector/model/Fan.onnx"; // YOLO模型路径
inline constexpr float CONFIDENCE_THRESHOLD = 0.5f;                        // 置信度阈值
inline constexpr float IOU_THRESHOLD = 0.5f;                               // NMS阈值

// 调试参数
inline constexpr bool IS_DEBUG = true; // 是否为调试模式

// 图像预处理参数
inline constexpr float INSIDE_SHADE_RATE = 0.7f;    // 内部遮罩半径与能量机关半径之比
inline constexpr float OUTSIDE_SHADE_RATE = 1.39f;  // 外部遮罩半径与能量机关半径之比
inline const cv::Scalar LOWER_HSV(0, 40, 220);       // 二值化HSV下限
inline const cv::Scalar UPPER_HSV(70, 255, 255);     // 二值化HSV上限
inline constexpr int DILATE_KERNEL_SIZE = 7;         // 膨胀核大小

// 其他参数
inline constexpr int MAX_LOST_FRAME = 5; // 最大丢失帧数
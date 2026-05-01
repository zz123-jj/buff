#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct BuffDetectorConfig
{
    std::string model_path = "src/buff_detector/model/yolo11_buff_int8.xml";
    std::string openvino_device = "GPU";
    float confidence_threshold = 0.5f;
    bool debug_mode = true;
    std::string buff_mode = "auto";
    float mode_judge_far_distance_px = 80.0f;
};

enum class BuffType
{
    small_buff,
    big_buff
};

class BBox
{
public:
    BBox() = default;
    BBox(float x_min, float y_min, float x_max, float y_max)
        : point_min_(x_min, y_min), point_max_(x_max, y_max)
    {
    }

    cv::Point2f get_point_min() const { return point_min_; }
    cv::Point2f get_point_max() const { return point_max_; }
    float get_width() const { return point_max_.x - point_min_.x; }
    float get_height() const { return point_max_.y - point_min_.y; }

private:
    cv::Point2f point_min_{0.0f, 0.0f};
    cv::Point2f point_max_{0.0f, 0.0f};
};

struct FanBlade
{
    BBox box;
};

class BuffDetector
{
public:
    BuffDetector() = default;

    void set_config(const BuffDetectorConfig& config) { config_ = config; }
    bool update(cv::Mat& frame, double stamp_sec);

    float get_pose_confidence() const { return pose_confidence_; }
    int get_yolo_target_count() const { return yolo_target_count_; }
    const std::vector<cv::Point2f>& get_current_target_keypoints() const
    {
        return current_target_keypoints_;
    }
    BuffType get_buff_type() const { return buff_type_; }

    cv::Mat debug_frame_;

private:
    bool detect_by_yolo(cv::Mat& frame, double stamp_sec);

    std::vector<FanBlade> target_blades_;
    float pose_confidence_ = 0.0f;
    int yolo_target_count_ = 0;
    std::vector<cv::Point2f> current_target_keypoints_;
    BuffType buff_type_ = BuffType::small_buff;
    BuffDetectorConfig config_;

    bool buff_type_locked_ = false;
    bool has_last_secondary_target_box_ = false;
    BBox last_secondary_target_box_;
};

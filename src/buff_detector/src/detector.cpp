#include "buff_detector/detector.hpp"
#include "buff_detector/yolo_inference.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
cv::Point2f rectCenter(const cv::Rect& rect)
{
    return {
        rect.x + rect.width * 0.5f,
        rect.y + rect.height * 0.5f
    };
}

BBox rectToBBox(const cv::Rect& rect)
{
    return BBox(
        static_cast<float>(rect.x),
        static_cast<float>(rect.y),
        static_cast<float>(rect.x + rect.width),
        static_cast<float>(rect.y + rect.height));
}

float candidateDistance(const BuffPoseCandidate& lhs, const BuffPoseCandidate& rhs)
{
    return static_cast<float>(cv::norm(rectCenter(lhs.fan_box) - rectCenter(rhs.fan_box)));
}

bool hasFarCandidate(const BuffPoseResult& output, float far_distance_px)
{
    if (output.candidates.size() < 2 || output.selected_index >= output.candidates.size()) {
        return false;
    }

    const auto& selected = output.candidates[output.selected_index];
    for (std::size_t i = 0; i < output.candidates.size(); ++i) {
        if (i == output.selected_index) {
            continue;
        }
        if (candidateDistance(selected, output.candidates[i]) >= far_distance_px) {
            return true;
        }
    }
    return false;
}

int findSecondaryCandidateIndex(
    const BuffPoseResult& output,
    float far_distance_px,
    bool prefer_far)
{
    if (output.candidates.size() < 2 || output.selected_index >= output.candidates.size()) {
        return -1;
    }

    const auto& selected = output.candidates[output.selected_index];
    int best_index = -1;
    float best_distance = -1.0f;
    for (std::size_t i = 0; i < output.candidates.size(); ++i) {
        if (i == output.selected_index) {
            continue;
        }

        const float distance = candidateDistance(selected, output.candidates[i]);
        if (prefer_far && distance < far_distance_px) {
            continue;
        }
        if (distance > best_distance) {
            best_distance = distance;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}
}  // namespace

bool BuffDetector::update(cv::Mat& frame, double stamp_sec)
{
    return detect_by_yolo(frame, stamp_sec);
}

bool BuffDetector::detect_by_yolo(cv::Mat& frame, double /*stamp_sec*/)
{
    static BuffYoloPoseOpenVINO yolo_detector;
    static bool loaded = false;
    if (!loaded) {
        loaded = yolo_detector.create_session(
            config_.model_path,
            config_.openvino_device,
            config_.confidence_threshold);
        if (!loaded && config_.openvino_device != "CPU") {
            RCLCPP_WARN(
                rclcpp::get_logger("BuffDetector"),
                "YOLOPose OpenVINO %s session init failed: %s. Fallback to CPU.",
                config_.openvino_device.c_str(),
                yolo_detector.last_error().c_str());
            loaded = yolo_detector.create_session(
                config_.model_path, "CPU", config_.confidence_threshold);
        }

        if (!loaded) {
            RCLCPP_ERROR(
                rclcpp::get_logger("BuffDetector"),
                "YOLOPose OpenVINO session init failed: %s",
                yolo_detector.last_error().c_str());
            return false;
        }
    }

    BuffPoseResult output;
    if (!yolo_detector.run(frame, output)) {
        if (config_.debug_mode) {
            RCLCPP_DEBUG(
                rclcpp::get_logger("BuffDetector"),
                "YOLOPose failed: %s",
                yolo_detector.last_error().c_str());
        }
        return false;
    }

    target_blades_.clear();

    std::string mode = config_.buff_mode;
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    if (!buff_type_locked_) {
        if (mode == "big" || mode == "big_buff") {
            buff_type_ = BuffType::big_buff;
        } else if (mode == "small" || mode == "small_buff") {
            buff_type_ = BuffType::small_buff;
        } else {
            buff_type_ = hasFarCandidate(output, config_.mode_judge_far_distance_px)
                ? BuffType::big_buff
                : BuffType::small_buff;
        }
        buff_type_locked_ = true;
        RCLCPP_INFO(
            rclcpp::get_logger("BuffDetector"),
            "Buff mode locked as %s by first YOLO frame: candidates=%d far_distance_px=%.1f",
            buff_type_ == BuffType::big_buff ? "big" : "small",
            output.target_count,
            config_.mode_judge_far_distance_px);
    }

    target_blades_.push_back(FanBlade{rectToBBox(output.fan_box)});

    if (buff_type_ == BuffType::big_buff) {
        const int secondary_index =
            findSecondaryCandidateIndex(output, config_.mode_judge_far_distance_px, true);
        const int fallback_secondary_index = secondary_index >= 0
            ? secondary_index
            : findSecondaryCandidateIndex(output, config_.mode_judge_far_distance_px, false);

        if (fallback_secondary_index >= 0) {
            last_secondary_target_box_ =
                rectToBBox(output.candidates[static_cast<std::size_t>(fallback_secondary_index)].fan_box);
            has_last_secondary_target_box_ = true;
        }

        if (has_last_secondary_target_box_) {
            target_blades_.push_back(FanBlade{last_secondary_target_box_});
        }
    } else {
        has_last_secondary_target_box_ = false;
    }

    pose_confidence_ = output.confidence;
    yolo_target_count_ = buff_type_ == BuffType::big_buff
        ? std::max(output.target_count, 2)
        : output.target_count;
    current_target_keypoints_.assign(
        output.keypoints.begin(),
        output.keypoints.begin() + std::min<std::size_t>(4, output.keypoints.size()));

    if (config_.debug_mode) {
        debug_frame_ = frame.clone();
        for (std::size_t i = 0; i < target_blades_.size(); ++i) {
            const auto& box = target_blades_[i].box;
            const cv::Scalar color = i == 0 ? cv::Scalar(0, 255, 0)
                                            : cv::Scalar(0, 180, 255);
            cv::rectangle(
                debug_frame_,
                cv::Rect(
                    static_cast<int>(box.get_point_min().x),
                    static_cast<int>(box.get_point_min().y),
                    static_cast<int>(box.get_width()),
                    static_cast<int>(box.get_height())),
                color,
                2);
        }
        cv::rectangle(debug_frame_, output.r_box, cv::Scalar(255, 255, 0), 2);
        for (const auto& point : current_target_keypoints_) {
            cv::circle(debug_frame_, point, 4, cv::Scalar(0, 0, 255), -1);
        }
    }

    RCLCPP_DEBUG(
        rclcpp::get_logger("BuffDetector"),
        "YOLOPose target detected: confidence=%.3f, candidates=%d, mode=%s, fan=(%d,%d,%d,%d), R=(%d,%d,%d,%d)",
        output.confidence,
        output.target_count,
        buff_type_ == BuffType::big_buff ? "big" : "small",
        output.fan_box.x,
        output.fan_box.y,
        output.fan_box.width,
        output.fan_box.height,
        output.r_box.x,
        output.r_box.y,
        output.r_box.width,
        output.r_box.height);

    return true;
}

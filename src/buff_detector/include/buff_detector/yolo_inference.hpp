#pragma once

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>

#include <cstddef>
#include <string>
#include <vector>

struct BuffPoseResult
{
    float confidence = 0.0f;
    std::vector<cv::Point2f> keypoints;
    cv::Rect fan_box;
    cv::Rect r_box;
};

class BuffYoloPoseOpenVINO
{
public:
    bool create_session(
        const std::string& model_path,
        const std::string& device_name,
        float confidence_threshold,
        std::size_t keypoint_count = 6,
        int input_size = 640
    );

    bool run(const cv::Mat& bgr_image, BuffPoseResult& result);
    const std::string& last_error() const { return last_error_; }

private:
    float fill_tensor(const cv::Mat& bgr_image);
    bool parse_output(const ov::Tensor& output_tensor, float scale_factor, BuffPoseResult& result);
    cv::Rect keypoints_to_rect(
        const std::vector<cv::Point2f>& keypoints,
        std::size_t begin,
        std::size_t count,
        const cv::Size& image_size
    ) const;
    cv::Rect detect_r_box(const std::vector<cv::Point2f>& keypoints, const cv::Mat& bgr_image) const;
    cv::Rect clamp_rect(const cv::Rect2f& rect, const cv::Size& image_size) const;
    void warm_up();

    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    ov::Tensor input_tensor_;

    std::string last_error_;
    std::string device_name_ = "GPU";
    float confidence_threshold_ = 0.7f;
    std::size_t keypoint_count_ = 6;
    int input_size_ = 640;
};

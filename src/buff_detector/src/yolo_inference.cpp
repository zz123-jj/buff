#include "buff_detector/yolo_inference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace
{
constexpr float kMinRBoxSize = 8.0f;
constexpr int kRThreshold = 100;
constexpr float kNmsThreshold = 0.45f;

struct PoseCandidate
{
    float confidence = 0.0f;
    std::vector<cv::Point2f> keypoints;
    cv::Rect fan_box;
};

float rect_iou(const cv::Rect& a, const cv::Rect& b)
{
    const int intersection = (a & b).area();
    const int union_area = a.area() + b.area() - intersection;
    if (union_area <= 0)
    {
        return 0.0f;
    }
    return static_cast<float>(intersection) / static_cast<float>(union_area);
}

template<typename T>
std::vector<float> tensor_to_float_vector(const ov::Tensor& tensor)
{
    const auto* data = tensor.data<const T>();
    std::vector<float> values(tensor.get_size());
    std::transform(data, data + tensor.get_size(), values.begin(), [](T value) {
        return static_cast<float>(value);
    });
    return values;
}
}  // namespace

bool BuffYoloPoseOpenVINO::create_session(
    const std::string& model_path,
    const std::string& device_name,
    float confidence_threshold,
    std::size_t keypoint_count,
    int input_size
)
{
    try
    {
        device_name_ = device_name;
        confidence_threshold_ = confidence_threshold;
        keypoint_count_ = keypoint_count;
        input_size_ = input_size;

        model_ = core_.read_model(model_path);
        compiled_model_ = core_.compile_model(model_, device_name_);
        infer_request_ = compiled_model_.create_infer_request();
        input_tensor_ = infer_request_.get_input_tensor();
        input_tensor_.set_shape({1, 3, static_cast<std::size_t>(input_size_), static_cast<std::size_t>(input_size_)});
        warm_up();
        last_error_.clear();
        return true;
    }
    catch (const std::exception& e)
    {
        last_error_ = e.what();
        return false;
    }
}

bool BuffYoloPoseOpenVINO::run(const cv::Mat& bgr_image, BuffPoseResult& result)
{
    if (bgr_image.empty())
    {
        last_error_ = "empty input image";
        return false;
    }

    try
    {
        const float scale_factor = fill_tensor(bgr_image);
        infer_request_.infer();

        BuffPoseResult parsed;
        if (!parse_output(infer_request_.get_output_tensor(), scale_factor, bgr_image.size(), parsed))
        {
            return false;
        }

        parsed.r_box = detect_r_box(parsed.keypoints, bgr_image);
        if (parsed.fan_box.area() <= 0 || parsed.r_box.area() <= 0)
        {
            last_error_ = "invalid fan or R box from pose keypoints";
            return false;
        }

        result = std::move(parsed);
        last_error_.clear();
        return true;
    }
    catch (const std::exception& e)
    {
        last_error_ = e.what();
        return false;
    }
}

float BuffYoloPoseOpenVINO::fill_tensor(const cv::Mat& bgr_image)
{
    const float scale = std::min(
        input_size_ / static_cast<float>(bgr_image.rows),
        input_size_ / static_cast<float>(bgr_image.cols)
    );

    cv::Mat blob_image;
    const cv::Matx23f matrix{scale, 0.0f, 0.0f, 0.0f, scale, 0.0f};
    cv::warpAffine(bgr_image, blob_image, matrix, cv::Size(input_size_, input_size_));
    blob_image.convertTo(blob_image, CV_32F, 1.0 / 255.0);
    cv::cvtColor(blob_image, blob_image, cv::COLOR_BGR2RGB);

    auto* data = input_tensor_.data<float>();
    for (int c = 0; c < 3; ++c)
    {
        for (int h = 0; h < input_size_; ++h)
        {
            for (int w = 0; w < input_size_; ++w)
            {
                data[c * input_size_ * input_size_ + h * input_size_ + w] =
                    blob_image.at<cv::Vec3f>(h, w)[c];
            }
        }
    }

    return 1.0f / scale;
}

bool BuffYoloPoseOpenVINO::parse_output(
    const ov::Tensor& output_tensor,
    float scale_factor,
    const cv::Size& image_size,
    BuffPoseResult& result
)
{
    const auto element_type = output_tensor.get_element_type();
    std::vector<float> values;
    if (element_type == ov::element::f32)
    {
        values = tensor_to_float_vector<float>(output_tensor);
    }
    else if (element_type == ov::element::f16)
    {
        values = tensor_to_float_vector<ov::float16>(output_tensor);
    }
    else
    {
        std::ostringstream oss;
        oss << "unsupported output tensor type: " << element_type;
        last_error_ = oss.str();
        return false;
    }

    const auto shape = output_tensor.get_shape();
    const std::size_t min_feature_count = 5 + keypoint_count_ * 2;
    std::size_t feature_count = 0;
    std::size_t candidate_count = 0;
    bool feature_major = true;

    if (shape.size() == 3)
    {
        const std::size_t dim1 = shape[1];
        const std::size_t dim2 = shape[2];
        feature_major = dim1 <= dim2;
        feature_count = feature_major ? dim1 : dim2;
        candidate_count = feature_major ? dim2 : dim1;
    }
    else if (shape.size() == 2)
    {
        const std::size_t dim0 = shape[0];
        const std::size_t dim1 = shape[1];
        feature_major = dim0 <= dim1;
        feature_count = feature_major ? dim0 : dim1;
        candidate_count = feature_major ? dim1 : dim0;
    }
    else
    {
        last_error_ = "unsupported output tensor rank";
        return false;
    }

    if (feature_count < min_feature_count || candidate_count == 0)
    {
        std::ostringstream oss;
        oss << "unexpected output shape, features=" << feature_count
            << " candidates=" << candidate_count;
        last_error_ = oss.str();
        return false;
    }

    auto at = [&](std::size_t feature, std::size_t candidate) -> float {
        if (shape.size() == 3)
        {
            if (feature_major)
            {
                return values[feature * candidate_count + candidate];
            }
            return values[candidate * feature_count + feature];
        }

        if (feature_major)
        {
            return values[feature * candidate_count + candidate];
        }
        return values[candidate * feature_count + feature];
    };

    std::vector<PoseCandidate> candidates;
    for (std::size_t candidate = 0; candidate < candidate_count; ++candidate)
    {
        const float confidence = at(4, candidate);
        if (confidence < confidence_threshold_)
        {
            continue;
        }

        PoseCandidate pose;
        pose.confidence = confidence;
        pose.keypoints.reserve(keypoint_count_);
        for (std::size_t i = 0; i < keypoint_count_; ++i)
        {
            pose.keypoints.emplace_back(
                at(5 + i * 2, candidate) * scale_factor,
                at(5 + i * 2 + 1, candidate) * scale_factor
            );
        }

        pose.fan_box = keypoints_to_rect(pose.keypoints, 0, 4, image_size);
        if (pose.fan_box.area() <= 0)
        {
            continue;
        }
        candidates.push_back(std::move(pose));
    }

    if (candidates.empty())
    {
        std::ostringstream oss;
        oss << "no pose candidate above threshold " << confidence_threshold_;
        last_error_ = oss.str();
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.confidence > rhs.confidence;
    });

    std::vector<PoseCandidate> kept;
    for (auto& candidate : candidates)
    {
        bool suppressed = false;
        for (const auto& accepted : kept)
        {
            if (rect_iou(candidate.fan_box, accepted.fan_box) > kNmsThreshold)
            {
                suppressed = true;
                break;
            }
        }
        if (!suppressed)
        {
            kept.push_back(std::move(candidate));
        }
    }

    result.confidence = kept.front().confidence;
    result.keypoints = kept.front().keypoints;
    result.fan_box = kept.front().fan_box;
    result.target_count = static_cast<int>(kept.size());
    return true;
}

cv::Rect BuffYoloPoseOpenVINO::keypoints_to_rect(
    const std::vector<cv::Point2f>& keypoints,
    std::size_t begin,
    std::size_t count,
    const cv::Size& image_size
) const
{
    if (keypoints.size() < begin + count || count == 0)
    {
        return {};
    }

    float x_min = std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_max = std::numeric_limits<float>::lowest();
    for (std::size_t i = begin; i < begin + count; ++i)
    {
        x_min = std::min(x_min, keypoints[i].x);
        y_min = std::min(y_min, keypoints[i].y);
        x_max = std::max(x_max, keypoints[i].x);
        y_max = std::max(y_max, keypoints[i].y);
    }

    return clamp_rect(cv::Rect2f(x_min, y_min, x_max - x_min, y_max - y_min), image_size);
}

cv::Rect BuffYoloPoseOpenVINO::detect_r_box(
    const std::vector<cv::Point2f>& keypoints,
    const cv::Mat& bgr_image
) const
{
    if (keypoints.size() < 6)
    {
        return {};
    }

    const cv::Point2f rough_center = (keypoints[5] - keypoints[4]) * 1.4f + keypoints[4];
    const float radius = std::max(
        static_cast<float>(cv::norm(keypoints[2] - keypoints[4]) * 0.8),
        kMinRBoxSize
    );

    cv::Mat gray;
    cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, kRThreshold, 255, cv::THRESH_BINARY);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(binary, binary, kernel);

    cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8U);
    cv::circle(mask, rough_center, static_cast<int>(radius), cv::Scalar(255), -1);
    cv::bitwise_and(binary, mask, binary);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    cv::Rect best_rect;
    double best_score = std::numeric_limits<double>::max();
    for (const auto& contour : contours)
    {
        if (contour.size() < 5)
        {
            continue;
        }

        const cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
        const float width = std::max(rotated_rect.size.width, 1.0f);
        const float height = std::max(rotated_rect.size.height, 1.0f);
        const float ratio = std::max(width / height, height / width);
        const double score =
            ratio + cv::norm(rotated_rect.center - rough_center) / std::max(radius / 3.0f, 1.0f);

        if (score < best_score)
        {
            best_score = score;
            best_rect = cv::boundingRect(contour);
        }
    }

    if (best_rect.area() > 0)
    {
        return clamp_rect(best_rect, bgr_image.size());
    }

    const float fallback_size = std::max(
        static_cast<float>(cv::norm(keypoints[5] - keypoints[4]) * 0.7),
        kMinRBoxSize
    );
    return clamp_rect(
        cv::Rect2f(
            rough_center.x - fallback_size * 0.5f,
            rough_center.y - fallback_size * 0.5f,
            fallback_size,
            fallback_size
        ),
        bgr_image.size()
    );
}

cv::Rect BuffYoloPoseOpenVINO::clamp_rect(const cv::Rect2f& rect, const cv::Size& image_size) const
{
    const float x_min = std::clamp(rect.x, 0.0f, static_cast<float>(std::max(image_size.width - 1, 0)));
    const float y_min = std::clamp(rect.y, 0.0f, static_cast<float>(std::max(image_size.height - 1, 0)));
    const float x_max = std::clamp(
        rect.x + rect.width,
        0.0f,
        static_cast<float>(std::max(image_size.width, 0))
    );
    const float y_max = std::clamp(
        rect.y + rect.height,
        0.0f,
        static_cast<float>(std::max(image_size.height, 0))
    );

    const int left = static_cast<int>(std::floor(x_min));
    const int top = static_cast<int>(std::floor(y_min));
    const int right = static_cast<int>(std::ceil(x_max));
    const int bottom = static_cast<int>(std::ceil(y_max));
    if (right <= left || bottom <= top)
    {
        return {};
    }
    return cv::Rect(left, top, right - left, bottom - top);
}

void BuffYoloPoseOpenVINO::warm_up()
{
    cv::Mat image(input_size_, input_size_, CV_8UC3, cv::Scalar(0, 0, 0));
    fill_tensor(image);
    infer_request_.infer();
}

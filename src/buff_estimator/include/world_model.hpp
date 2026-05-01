#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace buff_estimator
{

struct WorldPointSample
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double t = 0.0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double confidence = 0.0;
};

using WorldPointDeque = std::deque<WorldPointSample, Eigen::aligned_allocator<WorldPointSample>>;
using WorldPointVector = std::vector<WorldPointSample, Eigen::aligned_allocator<WorldPointSample>>;

struct CircleModel
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d basis_u = Eigen::Vector3d::UnitX();
    Eigen::Vector3d basis_v = Eigen::Vector3d::UnitY();
    double radius = 0.0;
    double plane_rms = 0.0;
    double circle_rms = 0.0;
    double stamp = 0.0;
    int sample_count = 0;
};

class PointBuffer
{
public:
    explicit PointBuffer(double window_sec = 5.0)
        : window_sec_(window_sec)
    {
    }

    void setWindow(double window_sec)
    {
        window_sec_ = std::max(0.1, window_sec);
    }

    void clear()
    {
        samples_.clear();
    }

    void push(double t, const Eigen::Vector3d& p, double confidence)
    {
        samples_.push_back(WorldPointSample{t, p, confidence});
        removeOld(t);
    }

    void removeOld(double now)
    {
        while (!samples_.empty() && now - samples_.front().t > window_sec_)
        {
            samples_.pop_front();
        }
    }

    double span() const
    {
        if (samples_.size() < 2)
        {
            return 0.0;
        }
        return samples_.back().t - samples_.front().t;
    }

    std::size_t size() const
    {
        return samples_.size();
    }

    const WorldPointDeque& samples() const
    {
        return samples_;
    }

private:
    double window_sec_ = 5.0;
    WorldPointDeque samples_;
};

class WorldCircleFitter
{
public:
    static bool fit(
        const WorldPointDeque& samples,
        const CircleModel& previous,
        CircleModel& output)
    {
        if (samples.size() < 6)
        {
            return false;
        }

        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto& sample : samples)
        {
            mean += sample.p;
        }
        mean /= static_cast<double>(samples.size());

        Eigen::MatrixXd centered(samples.size(), 3);
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            centered.row(static_cast<Eigen::Index>(i)) = (samples[i].p - mean).transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeFullV);
        Eigen::Vector3d normal = svd.matrixV().col(2).normalized();
        if (previous.valid && normal.dot(previous.normal) < 0.0)
        {
            normal = -normal;
        }

        Eigen::Vector3d basis_u;
        if (previous.valid)
        {
            basis_u = previous.basis_u - normal * previous.basis_u.dot(normal);
            if (basis_u.norm() < 1e-6)
            {
                basis_u = normal.unitOrthogonal();
            }
        }
        else
        {
            basis_u = normal.unitOrthogonal();
        }
        basis_u.normalize();
        Eigen::Vector3d basis_v = normal.cross(basis_u).normalized();

        Eigen::MatrixXd lhs(samples.size(), 3);
        Eigen::VectorXd rhs(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const Eigen::Vector3d d = samples[i].p - mean;
            const double x = d.dot(basis_u);
            const double y = d.dot(basis_v);
            lhs(static_cast<Eigen::Index>(i), 0) = x;
            lhs(static_cast<Eigen::Index>(i), 1) = y;
            lhs(static_cast<Eigen::Index>(i), 2) = 1.0;
            rhs(static_cast<Eigen::Index>(i)) = -(x * x + y * y);
        }

        const Eigen::Vector3d coeff = lhs.colPivHouseholderQr().solve(rhs);
        const double cx = -0.5 * coeff.x();
        const double cy = -0.5 * coeff.y();
        const Eigen::Vector3d center = mean + cx * basis_u + cy * basis_v;

        double radius_sum = 0.0;
        for (const auto& sample : samples)
        {
            const Eigen::Vector3d r = sample.p - center;
            const Eigen::Vector3d projected = r - normal * r.dot(normal);
            radius_sum += projected.norm();
        }
        const double radius = radius_sum / static_cast<double>(samples.size());
        if (!std::isfinite(radius) || radius < 1e-4)
        {
            return false;
        }

        double plane_error_sq = 0.0;
        double circle_error_sq = 0.0;
        for (const auto& sample : samples)
        {
            const Eigen::Vector3d r = sample.p - center;
            const double plane_error = r.dot(normal);
            const double circle_error = (r - normal * plane_error).norm() - radius;
            plane_error_sq += plane_error * plane_error;
            circle_error_sq += circle_error * circle_error;
        }

        output.valid = true;
        output.center = center;
        output.normal = normal;
        output.basis_u = basis_u;
        output.basis_v = basis_v;
        output.radius = radius;
        output.plane_rms = std::sqrt(plane_error_sq / static_cast<double>(samples.size()));
        output.circle_rms = std::sqrt(circle_error_sq / static_cast<double>(samples.size()));
        output.stamp = samples.back().t;
        output.sample_count = static_cast<int>(samples.size());
        return std::isfinite(output.plane_rms) && std::isfinite(output.circle_rms);
    }

    static double angleOf(const CircleModel& model, const Eigen::Vector3d& p)
    {
        const Eigen::Vector3d r = p - model.center;
        return std::atan2(r.dot(model.basis_v), r.dot(model.basis_u));
    }

    static double radialResidual(const CircleModel& model, const Eigen::Vector3d& p)
    {
        const Eigen::Vector3d r = p - model.center;
        const double plane_error = r.dot(model.normal);
        const double radius = (r - model.normal * plane_error).norm();
        return std::hypot(plane_error, radius - model.radius);
    }
};

inline double unwrapAngle(double angle, double last_angle)
{
    double diff = angle - last_angle;
    while (diff > M_PI)
    {
        diff -= 2.0 * M_PI;
    }
    while (diff < -M_PI)
    {
        diff += 2.0 * M_PI;
    }
    return last_angle + diff;
}

}  // namespace buff_estimator

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
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double confidence = 0.0;
    bool has_normal = false;
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
    double angle_span = 0.0;
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

    void push(
        double t,
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& normal,
        double confidence)
    {
        WorldPointSample sample;
        sample.t = t;
        sample.p = p;
        sample.confidence = confidence;
        sample.has_normal = normal.allFinite() && normal.norm() > 1e-6;
        if (sample.has_normal)
        {
            sample.normal = normal.normalized();
        }
        samples_.push_back(sample);
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
        double normal_weight,
        double fixed_radius,
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
        Eigen::Vector3d pca_normal = svd.matrixV().col(2).normalized();
        Eigen::Vector3d normal = pca_normal;

        Eigen::Vector3d pnp_normal = Eigen::Vector3d::Zero();
        double normal_weight_sum = 0.0;
        for (const auto& sample : samples)
        {
            if (!sample.has_normal)
            {
                continue;
            }

            Eigen::Vector3d sample_normal = sample.normal.normalized();
            if (previous.valid && sample_normal.dot(previous.normal) < 0.0)
            {
                sample_normal = -sample_normal;
            }
            else if (!previous.valid && normal_weight_sum > 0.0 &&
                     sample_normal.dot(pnp_normal) < 0.0)
            {
                sample_normal = -sample_normal;
            }

            const double weight = std::max(sample.confidence, 1e-3);
            pnp_normal += weight * sample_normal;
            normal_weight_sum += weight;
        }

        if (normal_weight_sum > 1e-6 && pnp_normal.norm() > 1e-6)
        {
            pnp_normal.normalize();
            if (pnp_normal.dot(pca_normal) < 0.0)
            {
                pnp_normal = -pnp_normal;
            }

            const double clamped_weight = std::clamp(normal_weight, 0.0, 1.0);
            normal = ((1.0 - clamped_weight) * pca_normal + clamped_weight * pnp_normal);
            if (normal.norm() < 1e-6)
            {
                normal = pnp_normal;
            }
            else
            {
                normal.normalize();
            }
        }

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

        std::vector<Eigen::Vector2d> plane_points;
        plane_points.reserve(samples.size());

        Eigen::MatrixXd lhs(samples.size(), 3);
        Eigen::VectorXd rhs(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            const Eigen::Vector3d d = samples[i].p - mean;
            const double x = d.dot(basis_u);
            const double y = d.dot(basis_v);
            plane_points.emplace_back(x, y);
            lhs(static_cast<Eigen::Index>(i), 0) = x;
            lhs(static_cast<Eigen::Index>(i), 1) = y;
            lhs(static_cast<Eigen::Index>(i), 2) = 1.0;
            rhs(static_cast<Eigen::Index>(i)) = -(x * x + y * y);
        }

        const Eigen::Vector3d coeff = lhs.colPivHouseholderQr().solve(rhs);
        double cx = -0.5 * coeff.x();
        double cy = -0.5 * coeff.y();

        double radius = 0.0;
        if (std::isfinite(fixed_radius) && fixed_radius > 1e-4)
        {
            radius = fixed_radius;
            seedFixedRadiusCenter(plane_points, previous, mean, basis_u, basis_v, radius, cx, cy);
            refineFixedRadiusCenter(plane_points, radius, cx, cy);
        }
        else
        {
            double radius_sum = 0.0;
            const Eigen::Vector3d free_center = mean + cx * basis_u + cy * basis_v;
            for (const auto& sample : samples)
            {
                const Eigen::Vector3d r = sample.p - free_center;
                const Eigen::Vector3d projected = r - normal * r.dot(normal);
                radius_sum += projected.norm();
            }
            radius = radius_sum / static_cast<double>(samples.size());
        }

        if (!std::isfinite(radius) || radius < 1e-4)
        {
            return false;
        }

        const Eigen::Vector3d center = mean + cx * basis_u + cy * basis_v;

        double plane_error_sq = 0.0;
        double circle_error_sq = 0.0;
        std::vector<double> angles;
        angles.reserve(samples.size());
        for (const auto& sample : samples)
        {
            const Eigen::Vector3d r = sample.p - center;
            const double plane_error = r.dot(normal);
            const double circle_error = (r - normal * plane_error).norm() - radius;
            plane_error_sq += plane_error * plane_error;
            circle_error_sq += circle_error * circle_error;
            angles.push_back(std::atan2(r.dot(basis_v), r.dot(basis_u)));
        }

        std::sort(angles.begin(), angles.end());
        double largest_gap = 0.0;
        for (std::size_t i = 1; i < angles.size(); ++i)
        {
            largest_gap = std::max(largest_gap, angles[i] - angles[i - 1]);
        }
        if (!angles.empty())
        {
            largest_gap = std::max(largest_gap, angles.front() + 2.0 * M_PI - angles.back());
        }

        output.valid = true;
        output.center = center;
        output.normal = normal;
        output.basis_u = basis_u;
        output.basis_v = basis_v;
        output.radius = radius;
        output.plane_rms = std::sqrt(plane_error_sq / static_cast<double>(samples.size()));
        output.circle_rms = std::sqrt(circle_error_sq / static_cast<double>(samples.size()));
        output.angle_span = 2.0 * M_PI - largest_gap;
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

private:
    static double fixedRadiusScore(
        const std::vector<Eigen::Vector2d>& points,
        double radius,
        double cx,
        double cy)
    {
        double score = 0.0;
        for (const auto& point : points)
        {
            const double residual = (point - Eigen::Vector2d(cx, cy)).norm() - radius;
            score += residual * residual;
        }
        return score;
    }

    static void seedFixedRadiusCenter(
        const std::vector<Eigen::Vector2d>& points,
        const CircleModel& previous,
        const Eigen::Vector3d& mean,
        const Eigen::Vector3d& basis_u,
        const Eigen::Vector3d& basis_v,
        double radius,
        double& cx,
        double& cy)
    {
        if (points.size() < 2)
        {
            return;
        }

        double best_chord = 0.0;
        std::size_t best_i = 0;
        std::size_t best_j = 1;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            for (std::size_t j = i + 1; j < points.size(); ++j)
            {
                const double chord = (points[i] - points[j]).norm();
                if (chord > best_chord)
                {
                    best_chord = chord;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        if (best_chord < 1e-6 || best_chord >= 2.0 * radius)
        {
            return;
        }

        const Eigen::Vector2d p0 = points[best_i];
        const Eigen::Vector2d p1 = points[best_j];
        const Eigen::Vector2d midpoint = 0.5 * (p0 + p1);
        const Eigen::Vector2d chord_dir = (p1 - p0).normalized();
        const Eigen::Vector2d perp(-chord_dir.y(), chord_dir.x());
        const double h = std::sqrt(std::max(0.0, radius * radius - 0.25 * best_chord * best_chord));
        const Eigen::Vector2d c0 = midpoint + h * perp;
        const Eigen::Vector2d c1 = midpoint - h * perp;

        if (previous.valid)
        {
            const Eigen::Vector3d previous_delta = previous.center - mean;
            const Eigen::Vector2d previous_center(
                previous_delta.dot(basis_u),
                previous_delta.dot(basis_v));
            const double score0 = (c0 - previous_center).squaredNorm();
            const double score1 = (c1 - previous_center).squaredNorm();
            cx = score0 <= score1 ? c0.x() : c1.x();
            cy = score0 <= score1 ? c0.y() : c1.y();
            return;
        }

        const double free_score = fixedRadiusScore(points, radius, cx, cy);
        const double score0 = fixedRadiusScore(points, radius, c0.x(), c0.y());
        const double score1 = fixedRadiusScore(points, radius, c1.x(), c1.y());
        if (score0 < free_score || score1 < free_score)
        {
            const Eigen::Vector2d chosen = score0 <= score1 ? c0 : c1;
            cx = chosen.x();
            cy = chosen.y();
        }
    }

    static void refineFixedRadiusCenter(
        const std::vector<Eigen::Vector2d>& points,
        double radius,
        double& cx,
        double& cy)
    {
        for (int iter = 0; iter < 12; ++iter)
        {
            Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
            Eigen::Vector2d gradient = Eigen::Vector2d::Zero();

            for (const auto& point : points)
            {
                const Eigen::Vector2d delta(cx - point.x(), cy - point.y());
                const double distance = delta.norm();
                if (distance < 1e-6)
                {
                    continue;
                }

                const double residual = distance - radius;
                const Eigen::Vector2d jacobian = delta / distance;
                hessian += jacobian * jacobian.transpose();
                gradient += jacobian * residual;
            }

            hessian += 1e-9 * Eigen::Matrix2d::Identity();
            const Eigen::Vector2d step = hessian.ldlt().solve(-gradient);
            if (!step.allFinite())
            {
                break;
            }

            cx += step.x();
            cy += step.y();
            if (step.norm() < 1e-6)
            {
                break;
            }
        }
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

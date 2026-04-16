#ifndef COORDINATE_SOLVER_HPP_
#define COORDINATE_SOLVER_HPP_
#include <cmath>
#include <geometry_msgs/msg/point.hpp>

class CoordinateSolver {
public:
    struct CameraIntrinsics {
        double fx, fy, cx, cy;
    };

    CoordinateSolver(CameraIntrinsics intri, double radius_world)
        : cam_(intri), radius_world_(radius_world) {}

    double computeDepthFromRadius(double radius_pixel) const {
        if (radius_pixel <= 1e-6) {
            return 0.0;
        }
        return (radius_world_ * cam_.fx) / radius_pixel;
    }

    // 像素坐标 -> 相机坐标系 (Z轴向前)
    geometry_msgs::msg::Point pixelToCamera(double u, double v, double depth) {
        geometry_msgs::msg::Point p;
        p.z = depth;
        p.x = (u - cam_.cx) * depth / cam_.fx;
        p.y = (v - cam_.cy) * depth / cam_.fy;
        return p;
    }

private:
    CameraIntrinsics cam_;
    double radius_world_;
};

#endif
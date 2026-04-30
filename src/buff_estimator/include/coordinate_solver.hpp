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
        : cam_intri_(intri), radius_world_(radius_world) {}


    double computeDepthFromRadius(double radius_pixel) const {
        if (radius_pixel <= 1e-6) {
            return 0.0;
        }
        return (radius_world_ * cam_intri_.fx) / radius_pixel;
    }

    // 像素坐标 -> 相机坐标系 (Z轴向前)
    geometry_msgs::msg::Point pixelToCamera(double u, double v, double depth) {
        geometry_msgs::msg::Point p;
        p.z = depth;
        p.x = (u - cam_intri_.cx) * depth / cam_intri_.fx;
        p.y = (v - cam_intri_.cy) * depth / cam_intri_.fy;
        return p;
    }

private:
    CameraIntrinsics cam_intri_;
    double radius_world_;
};

#endif
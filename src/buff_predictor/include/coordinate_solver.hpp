#ifndef COORDINATE_SOLVER_HPP_
#define COORDINATE_SOLVER_HPP_

#include <cmath>
#include <geometry_msgs/msg/point.hpp>

class CoordinateSolver {
public:
    struct CameraIntrinsics {
        double fx, fy, cx, cy;
    };

    CoordinateSolver(CameraIntrinsics intri) : cam_(intri) {}

    // 像素坐标 -> 相机坐标系 (Z轴向前)
    geometry_msgs::msg::Point pixelToCamera(double u, double v, double depth) {
        geometry_msgs::msg::Point p;
        p.z = depth;
        p.x = (u - cam_.cx) * depth / cam_.fx;
        p.y = (v - cam_.cy) * depth / cam_.fy;
        return p;
    }

    // 相机坐标系 -> 世界坐标系
    
    // geometry_msgs::msg::Point cameraToWorld(const geometry_msgs::msg::Point& p_cam, double pitch, double yaw) {
    //     // 1. 坐标轴修正 (Camera -> Body/Gimbal中心)
    //     // Camera: Z前, X右, Y下
    //     // Body:   X前, Y左, Z上
    //     double x_b = p_cam.z;
    //     double y_b = -p_cam.x;
    //     double z_b = -p_cam.y;

        
        
    //     //旋转矩阵 
        
    //     //Pitch
    //     // x' = x*cos(P) + z*sin(P)
    //     // z' = -x*sin(P) + z*cos(P)
    //     double x_p = x_b * std::cos(pitch) + z_b * std::sin(pitch);
    //     double y_p = y_b;
    //     double z_p = -x_b * std::sin(pitch) + z_b * std::cos(pitch);

    //     // Yaw
    //     // x'' = x'*cos(Y) - y'*sin(Y)
    //     // y'' = x'*sin(Y) + y'*cos(Y)
    //     double x_w = x_p * std::cos(yaw) - y_p * std::sin(yaw);
    //     double y_w = x_p * std::sin(yaw) + y_p * std::cos(yaw);
    //     double z_w = z_p;

    //     geometry_msgs::msg::Point p_world;
    //     p_world.x = x_w;
    //     p_world.y = y_w;
    //     p_world.z = z_w;
    //     return p_world;
    // }

private:
    CameraIntrinsics cam_;
};

#endif
#ifndef RM_AUTO_BUFF_ROS__TRAJECTORY_HPP_
#define RM_AUTO_BUFF_ROS__TRAJECTORY_HPP_

namespace rm_buff_common
{

struct Trajectory
{
  bool solvable = false;
  double fly_time = 0.0;
  double pitch = 0.0;
};

Trajectory solve_trajectory(double bullet_speed, double horizontal_distance, double height);

}  // namespace rm_buff_common

#endif  // RM_AUTO_BUFF_ROS__TRAJECTORY_HPP_

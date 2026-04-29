#include "rm_buff_common/trajectory.hpp"

#include <cmath>

namespace rm_buff_common
{

Trajectory solve_trajectory(double bullet_speed, double horizontal_distance, double height)
{
  constexpr double gravity = 9.794;
  Trajectory trajectory;
  if (bullet_speed <= 1e-3 || horizontal_distance <= 1e-3) return trajectory;

  auto v2 = bullet_speed * bullet_speed;
  auto discriminant =
    v2 * v2 - gravity * (gravity * horizontal_distance * horizontal_distance + 2.0 * height * v2);
  if (discriminant < 0.0) return trajectory;

  auto tan_pitch = (v2 - std::sqrt(discriminant)) / (gravity * horizontal_distance);
  trajectory.pitch = std::atan(tan_pitch);
  auto cos_pitch = std::cos(trajectory.pitch);
  if (std::abs(cos_pitch) < 1e-6) return trajectory;

  trajectory.fly_time = horizontal_distance / (bullet_speed * cos_pitch);
  trajectory.solvable = std::isfinite(trajectory.fly_time) && trajectory.fly_time > 0.0;
  return trajectory;
}

}  // namespace rm_buff_common

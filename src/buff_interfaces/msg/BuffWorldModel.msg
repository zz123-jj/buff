std_msgs/Header header

# World-frame buff rotation model published by buff_estimator.
bool valid
int8 mode                    # 0=unknown, -1=small buff, 1=big buff
int8 spin_direction          # 0=unknown, -1=negative angle, 1=positive angle

bool has_observation             # true only when this message was corrected by a YOLO observation
float32 seconds_since_observation
geometry_msgs/Point observed_point_world # last/current YOLO observation in world frame
geometry_msgs/Point fitted_point_world   # model point on the circle at header.stamp
geometry_msgs/Point observed_point_camera
float32 depth

geometry_msgs/Point circle_center_world
geometry_msgs/Vector3 circle_axis_world
float32 circle_radius
float32 current_angle
float32 angular_velocity

# Big buff angular speed model:
# abs(w(t)) = speed_a * sin(speed_omega * t + speed_phi) + speed_b
float32 speed_a
float32 speed_omega
float32 speed_phi
float32 speed_b
float64 speed_fit_start_time_sec
float32 speed_fit_buffer_duration_sec
int32 speed_fit_sample_count

float32 plane_rms
float32 circle_rms
float32 angle_span
float32 observation_confidence
int32 yolo_target_count

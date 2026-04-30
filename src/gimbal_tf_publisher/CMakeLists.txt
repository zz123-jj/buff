cmake_minimum_required(VERSION 3.8)
project(gimbal_tf_publisher)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(joint_tf_publisher src/joint_tf_publisher.cpp)
ament_target_dependencies(joint_tf_publisher rclcpp sensor_msgs geometry_msgs tf2_ros tf2)

add_executable(quaternion_tf_publisher src/quaternion_tf_publisher.cpp)
ament_target_dependencies(quaternion_tf_publisher rclcpp geometry_msgs tf2_ros tf2)

install(TARGETS
  joint_tf_publisher
  quaternion_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

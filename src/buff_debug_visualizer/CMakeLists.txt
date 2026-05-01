cmake_minimum_required(VERSION 3.8)
project(buff_debug_visualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(buff_interfaces REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/debug_visualizer.cpp
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  buff_interfaces
)

target_link_libraries(${PROJECT_NAME}_component
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "BuffDebugVisualizer"
  EXECUTABLE buff_debug_visualizer_node
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.16)
project(rm_buff_aimer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_buff_common REQUIRED)
find_package(rm_vision_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/aimer_node.cpp)
target_link_libraries(${PROJECT_NAME}_component Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_component
  gary_msgs
  geometry_msgs
  rclcpp
  rclcpp_components
  rm_buff_common
  rm_vision_interfaces
  std_msgs
  tf2
  tf2_ros
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "rm_buff_aimer::BuffAimerNode")

install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(gary_msgs geometry_msgs rclcpp rclcpp_components rm_buff_common rm_vision_interfaces std_msgs tf2 tf2_ros)
ament_package()

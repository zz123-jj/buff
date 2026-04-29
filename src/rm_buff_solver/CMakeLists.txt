cmake_minimum_required(VERSION 3.16)
project(rm_buff_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_buff_common REQUIRED)
find_package(rm_vision_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/solver_node.cpp)
target_include_directories(${PROJECT_NAME}_component PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_component Eigen3::Eigen ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_component
  geometry_msgs
  rclcpp
  rclcpp_components
  rm_buff_common
  rm_vision_interfaces
  sensor_msgs
  tf2
  tf2_ros
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "rm_buff_solver::BuffSolverNode")

install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(geometry_msgs rclcpp rclcpp_components rm_buff_common rm_vision_interfaces sensor_msgs tf2 tf2_ros)
ament_package()

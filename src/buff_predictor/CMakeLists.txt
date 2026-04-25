cmake_minimum_required(VERSION 3.8)
project(buff_predictor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)   # 组件支持
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)       
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(buff_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###########
## Build ##
###########

# 创建共享库（供组件使用）
add_library(${PROJECT_NAME}_component SHARED
  src/predictor.cpp
  src/dataProcessor.cpp
  src/predictor_node.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  buff_interfaces
)

target_link_libraries(${PROJECT_NAME}_component
  ${OpenCV_LIBS}
  Ceres::ceres
)

# 注册组件（自动生成可执行文件 buff_predictor_node）
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "BuffPredictorNode"
  EXECUTABLE buff_predictor_node
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  Eigen3
  Ceres
)

ament_package()
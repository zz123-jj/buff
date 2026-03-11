cmake_minimum_required(VERSION 3.8)
project(buff_predictor)

# Enforce C++17
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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)       
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(buff_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###########
## Build ##
###########


add_library(${PROJECT_NAME} SHARED
  src/predictor.cpp
  src/dataProcessor.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)


add_executable(buff_predictor_node
  src/predictor_node.cpp 
)

target_link_libraries(buff_predictor_node
  ${PROJECT_NAME}
)

ament_target_dependencies(buff_predictor_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs         
  buff_interfaces
  tf2_ros 
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  buff_predictor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs         
  Eigen3
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(contact)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(spdlog REQUIRED)

include_directories(
  include
)

add_subdirectory(serial/lib)

add_executable(
  contact_node
  src/ContactNode.cpp
  src/CommPort.cpp
  src/Checksum.cpp
)

target_link_libraries(
  contact_node
  serial
  spdlog::spdlog
  -lpthread
)

ament_target_dependencies(
  contact_node
  rclcpp
  gary_msgs
  tf2_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
)

install(
  TARGETS
  contact_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

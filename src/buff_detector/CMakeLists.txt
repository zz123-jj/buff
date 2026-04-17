cmake_minimum_required(VERSION 3.8)
project(buff_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(buff_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

message(STATUS "Jazzy Build: cv_bridge include: ${cv_bridge_INCLUDE_DIRS}")

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}       
  ${image_transport_INCLUDE_DIRS} 
  ${sensor_msgs_INCLUDE_DIRS}     
 
  /home/caleb/onnxruntime/include
)

link_directories(

  /home/caleb/onnxruntime/lib
)

add_executable(buff_detector
  src/detector_node.cpp
  src/detector.cpp
  src/yolo_inference.cpp
)

ament_target_dependencies(buff_detector
  rclcpp
  buff_interfaces
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

target_link_libraries(buff_detector

  /home/caleb/onnxruntime/lib/libonnxruntime.so
)

set_target_properties(buff_detector PROPERTIES
    BUILD_RPATH "/home/caleb/onnxruntime/lib"
    INSTALL_RPATH "/home/caleb/onnxruntime/lib"
)

install(TARGETS buff_detector
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

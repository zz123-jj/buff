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
find_package(rclcpp_components REQUIRED)
find_package(buff_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ------------------------------------------------------------
# 强制使用环境变量 ONNXRUNTIME_DIR
# ------------------------------------------------------------
set(ONNXRUNTIME_DIR "$ENV{ONNXRUNTIME_DIR}" CACHE PATH "ONNX Runtime root directory")
if(NOT ONNXRUNTIME_DIR)
  message(FATAL_ERROR "ONNXRUNTIME_DIR environment variable not set. Please set it to the ONNX Runtime install path.")
endif()

find_path(ONNX_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS ${ONNXRUNTIME_DIR}/include
  NO_DEFAULT_PATH
)

find_library(ONNX_LIB
  NAMES onnxruntime libonnxruntime
  PATHS ${ONNXRUNTIME_DIR}/lib
  NO_DEFAULT_PATH
)

if(NOT ONNX_INCLUDE_DIR OR NOT ONNX_LIB)
  message(FATAL_ERROR "ONNX Runtime not found at ${ONNXRUNTIME_DIR}")
endif()

message(STATUS "Found ONNX Runtime: include=${ONNX_INCLUDE_DIR}, lib=${ONNX_LIB}")

# 创建共享库
add_library(${PROJECT_NAME}_component SHARED
  src/detector_node.cpp
  src/detector.cpp
  src/yolo_inference.cpp
)

target_include_directories(${PROJECT_NAME}_component PRIVATE
  include
  ${ONNX_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  ament_index_cpp
  buff_interfaces
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  OpenCV
)

target_link_libraries(${PROJECT_NAME}_component
  ${ONNX_LIB}
)

# 组件注册
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "BuffDetectorNode"
  EXECUTABLE buff_detector_node
)

# 安装
install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
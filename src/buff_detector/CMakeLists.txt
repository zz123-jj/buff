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

set(OPENVINO_CMAKE_CANDIDATES
  "/usr/lib/cmake/openvino2024.6.0"
  "/opt/intel/openvino_2024.6.0/runtime/cmake"
  "$ENV{HOME}/.local/lib/python3.12/site-packages/openvino/cmake"
  "$ENV{HOME}/.local/lib/python3.10/site-packages/openvino/cmake"
)

foreach(OPENVINO_CMAKE_DIR IN LISTS OPENVINO_CMAKE_CANDIDATES)
  if(NOT OpenVINO_DIR AND EXISTS "${OPENVINO_CMAKE_DIR}/OpenVINOConfig.cmake")
    set(OpenVINO_DIR "${OPENVINO_CMAKE_DIR}" CACHE PATH "OpenVINO CMake directory")
  endif()
endforeach()

find_package(OpenVINO QUIET)

if(NOT OpenVINO_FOUND AND NOT OpenVINO_DIR)
  find_package(Python3 COMPONENTS Interpreter QUIET)
endif()
if(NOT OpenVINO_FOUND AND NOT OpenVINO_DIR AND Python3_Interpreter_FOUND)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import pathlib, openvino; print(pathlib.Path(openvino.__file__).resolve().parent / 'cmake')"
    OUTPUT_VARIABLE PYTHON_OPENVINO_CMAKE
    RESULT_VARIABLE PYTHON_OPENVINO_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    TIMEOUT 5
  )
  if(PYTHON_OPENVINO_RESULT EQUAL 0 AND EXISTS "${PYTHON_OPENVINO_CMAKE}/OpenVINOConfig.cmake")
    set(OpenVINO_DIR "${PYTHON_OPENVINO_CMAKE}" CACHE PATH "OpenVINO CMake directory")
    find_package(OpenVINO QUIET)
  endif()
endif()

if(NOT OpenVINO_FOUND)
  find_package(OpenVINO REQUIRED)
endif()

# 创建共享库
add_library(${PROJECT_NAME}_component SHARED
  src/detector_node.cpp
  src/detector.cpp
  src/yolo_inference.cpp
)

target_include_directories(${PROJECT_NAME}_component PRIVATE
  include
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
  openvino::runtime
)

if(OpenVINO_DIR MATCHES "site-packages/openvino/cmake$")
  get_filename_component(PYTHON_OPENVINO_ROOT "${OpenVINO_DIR}" DIRECTORY)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES
    BUILD_RPATH "${PYTHON_OPENVINO_ROOT}/libs"
    INSTALL_RPATH "${PYTHON_OPENVINO_ROOT}/libs"
  )
endif()

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

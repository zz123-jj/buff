cmake_minimum_required(VERSION 3.16)
project(rm_buff_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
if(NOT OpenVINO_DIR AND EXISTS "/opt/intel/openvino_2024.6.0/runtime/cmake/OpenVINOConfig.cmake")
  set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/" CACHE PATH "OpenVINO CMake directory")
endif()
if(NOT OpenVINO_DIR)
  execute_process(
    COMMAND python3 -c "import pathlib, openvino; print(pathlib.Path(openvino.__file__).resolve().parent / 'cmake')"
    OUTPUT_VARIABLE PYTHON_OPENVINO_CMAKE
    RESULT_VARIABLE PYTHON_OPENVINO_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(PYTHON_OPENVINO_RESULT EQUAL 0 AND EXISTS "${PYTHON_OPENVINO_CMAKE}/OpenVINOConfig.cmake")
    set(OpenVINO_DIR "${PYTHON_OPENVINO_CMAKE}" CACHE PATH "OpenVINO CMake directory")
  endif()
endif()
if(NOT OpenVINO_DIR AND EXISTS "$ENV{HOME}/.local/lib/python3.10/site-packages/openvino/cmake/OpenVINOConfig.cmake")
  set(OpenVINO_DIR "$ENV{HOME}/.local/lib/python3.10/site-packages/openvino/cmake" CACHE PATH "OpenVINO CMake directory")
endif()
find_package(OpenVINO REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_vision_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/detector_node.cpp)
target_include_directories(${PROJECT_NAME}_component PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_component openvino::runtime ${OpenCV_LIBS} image_transport::image_transport)
if(OpenVINO_DIR MATCHES "site-packages/openvino/cmake$")
  get_filename_component(PYTHON_OPENVINO_ROOT "${OpenVINO_DIR}" DIRECTORY)
  set_target_properties(${PROJECT_NAME}_component PROPERTIES
    BUILD_RPATH "${PYTHON_OPENVINO_ROOT}/libs"
    INSTALL_RPATH "${PYTHON_OPENVINO_ROOT}/libs"
  )
endif()
ament_target_dependencies(${PROJECT_NAME}_component
  cv_bridge
  geometry_msgs
  rclcpp
  rclcpp_components
  image_transport
  rm_vision_interfaces
  sensor_msgs
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "rm_buff_detector::BuffDetectorNode")

install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  cv_bridge
  geometry_msgs
  image_transport
  rclcpp
  rclcpp_components
  rm_vision_interfaces
  sensor_msgs
)
ament_package()

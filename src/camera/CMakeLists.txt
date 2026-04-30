cmake_minimum_required(VERSION 3.14)
project(camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

find_library(GXIAPI_LIBRARY
  NAMES gxiapi
  PATHS
    /usr/lib
    /usr/local/lib
    /opt/DahengImaging/GalaxySDK/lib
    /opt/DahengImaging/GalaxySDK/lib/x86_64
)
find_library(GLOG_LIBRARY NAMES glog)

if(GXIAPI_LIBRARY)
  set(CAMERA_SOURCES
    src/CamNode.cpp
    src/CamWrapper.cpp
  )
else()
  message(WARNING "libgxiapi not found. Building a disabled CameraPublisher component; install Daheng Galaxy SDK for real camera capture.")
  set(CAMERA_SOURCES
    src/disabled_camera_node.cpp
  )
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${CAMERA_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
if(GXIAPI_LIBRARY)
  target_link_libraries(${PROJECT_NAME} ${GXIAPI_LIBRARY})
  if(GLOG_LIBRARY)
    target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARY})
  else()
    target_link_libraries(${PROJECT_NAME} glog)
  endif()
endif()

# Register the component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN CameraPublisher
  EXECUTABLE camera_node
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch config
)

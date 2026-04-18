cmake_minimum_required(VERSION 3.8)
project(buff_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(buff_interfaces REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 创建共享库（组件必须为 SHARED）
add_library(${PROJECT_NAME} SHARED
  src/solver.cpp
)

# 设置包含目录（如果将来有头文件可启用）
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  buff_interfaces
  gary_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# 注册组件
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "BuffSolver"
  EXECUTABLE buff_solver_node
)

# 安装库和可执行文件
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 头文件安装暂缺，注释掉
# install(DIRECTORY include/
#   DESTINATION include
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
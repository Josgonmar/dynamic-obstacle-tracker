cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacle_tracker LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree search segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Polynomial3.msg"
  "msg/PiecewisePolynomial3.msg"
  "msg/QuinticPolynomial3.msg"
  "msg/QuinticPiecewisePolynomial3.msg"
  "msg/DynamicObstacleTrajectory.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(dynamic_obstacle_tracker_node
  src/ros2/dynamic_obstacle_tracker_node.cpp
  src/tracker/obstacle_tracker.cpp
  src/common/config_loader.cpp
  utils/main.cpp
)

target_include_directories(dynamic_obstacle_tracker_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(dynamic_obstacle_tracker_node
  ament_index_cpp
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(dynamic_obstacle_tracker_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dynamic_obstacle_tracker_node "${cpp_typesupport_target}")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dynamic_obstacle_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

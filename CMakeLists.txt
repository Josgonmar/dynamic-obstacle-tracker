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
find_package(PCL REQUIRED COMPONENTS common)
find_package(TBB REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# The KD-tree and clustering modules are header-only. Avoid cilantro's top-level
# CMake project, which also builds optional visualization sources and sets global flags.
set(CILANTRO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/cilantro/include")
if(NOT EXISTS "${CILANTRO_INCLUDE_DIR}/cilantro/core/kd_tree.hpp")
  message(FATAL_ERROR "The cilantro submodule is missing; run 'git submodule update --init --recursive'")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Polynomial3.msg"
  "msg/PiecewisePolynomial3.msg"
  "msg/QuinticPolynomial3.msg"
  "msg/QuinticPiecewisePolynomial3.msg"
  "msg/DynamicObstacleTrajectory.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(dynamic_obstacle_tracker_component SHARED
  src/ros2/dynamic_obstacle_tracker_node.cpp
  src/tracker/obstacle_tracker.cpp
)

add_library(dynamic_point_detector_component SHARED
  src/detector/dynamic_point_detector.cpp
  src/detector/temporal_voxel_map.cpp
  src/ros2/dynamic_point_detector_node.cpp
)

target_include_directories(dynamic_obstacle_tracker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(dynamic_point_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(dynamic_obstacle_tracker_component SYSTEM PRIVATE
  ${CILANTRO_INCLUDE_DIR}
)

target_include_directories(dynamic_point_detector_component SYSTEM PRIVATE
  ${CILANTRO_INCLUDE_DIR}
)

ament_target_dependencies(dynamic_obstacle_tracker_component
  ament_index_cpp
  geometry_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  visualization_msgs
)

ament_target_dependencies(dynamic_point_detector_component
  ament_index_cpp
  geometry_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
)

target_link_libraries(dynamic_obstacle_tracker_component
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

target_link_libraries(dynamic_point_detector_component
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  TBB::tbb
  yaml-cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dynamic_obstacle_tracker_component "${cpp_typesupport_target}")
target_link_libraries(dynamic_point_detector_component "${cpp_typesupport_target}")

rclcpp_components_register_nodes(dynamic_obstacle_tracker_component
  "dynamic_obstacle_tracker::DynamicObstacleTrackerNode"
)
rclcpp_components_register_nodes(dynamic_point_detector_component
  "dynamic_obstacle_tracker::DynamicPointDetectorNode"
)

add_executable(dynamic_obstacle_tracker_node utils/tracker_main.cpp)
add_executable(dynamic_point_detector_node utils/detector_main.cpp)

ament_target_dependencies(dynamic_obstacle_tracker_node rclcpp)
ament_target_dependencies(dynamic_point_detector_node rclcpp)

target_link_libraries(dynamic_obstacle_tracker_node
  dynamic_obstacle_tracker_component
)
target_link_libraries(dynamic_point_detector_node
  dynamic_point_detector_component
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    dynamic_obstacle_tracker_component
    dynamic_point_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    dynamic_obstacle_tracker_node
    dynamic_point_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES external/cilantro/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME cilantro-LICENSE
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

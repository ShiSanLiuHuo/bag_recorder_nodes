cmake_minimum_required(VERSION 3.10)
project(bag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # 修正拼写

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

set(BAG_RECORDER_NODE_SOURCE src/bag_recorder_node.cpp)

add_library(bag_recorder_component SHARED ${BAG_RECORDER_NODE_SOURCE})
target_include_directories(bag_recorder_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bag_recorder_component
  rclcpp
  rclcpp_components
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  cv_bridge
  OpenCV
)

rclcpp_components_register_node(bag_recorder_component
  PLUGIN "BagRecorderNode"
  EXECUTABLE bag_recorder_node
)

install(TARGETS bag_recorder_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  rosbag2_cpp
  rosbag2_storage
)
ament_export_include_directories(include)
ament_export_libraries(bag_recorder_component)
ament_package()
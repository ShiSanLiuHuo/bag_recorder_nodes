cmake_minimum_required(VERSION 3.8)
project(bag_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMENDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# 组件必须是 SHARED library
add_library(bag_recorder_component SHARED
  src/bag_recorder_node.cpp
)

ament_target_dependencies(bag_recorder_component
  rclcpp
  rclcpp_components
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
)

# 注册 component
rclcpp_components_register_nodes(bag_recorder_component
  PLUGIN "BagRecorderNode"
)

# 安装库
install(TARGETS bag_recorder_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

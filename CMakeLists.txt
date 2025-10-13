cmake_minimum_required(VERSION 3.8)
project(bag_recorder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_executable(bag_recorder_node src/bag_recorder_node.cpp)
ament_target_dependencies(bag_recorder_node
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
)

install(TARGETS
  bag_recorder_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

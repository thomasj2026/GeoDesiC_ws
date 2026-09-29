cmake_minimum_required(VERSION 3.8)
project(state_estimation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  imu_subscriber
  src/imu_subscriber.cpp
)

ament_target_dependencies(
  imu_subscriber
  rclcpp
  sensor_msgs
)

install(
  TARGETS imu_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/imu_covariance_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
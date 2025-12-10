cmake_minimum_required(VERSION 3.10)
project(gait_estimator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gait_estimator src/gait_estimator.cpp)
ament_target_dependencies(gait_estimator rclcpp sensor_msgs std_msgs)

install(TARGETS gait_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

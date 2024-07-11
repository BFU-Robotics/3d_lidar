cmake_minimum_required(VERSION 3.8)
project(lidar_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories("include/io")
include_directories("include/protocol")

add_executable(${PROJECT_NAME} src/main.cpp include/io/SerialConnector.hpp src/io/SerialConnector.cpp include/protocol/ModbusMaster.hpp src/protocol/ModbusMaster.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()

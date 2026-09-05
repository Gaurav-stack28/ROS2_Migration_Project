cmake_minimum_required(VERSION 3.8)

project(velodyne_laserscan)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(
  include
)


# Main library
add_library(${PROJECT_NAME}
  src/velodyne_laserscan.cpp
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)


# ROS2 executable node
add_executable(${PROJECT_NAME}_node
  src/node.cpp
)


target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)


ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
)


# Install
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()
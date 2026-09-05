cmake_minimum_required(VERSION 3.5)
project(catvehicle)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# ROS 2 dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ============================================================
# ros2_control dependencies
# ============================================================

find_package(controller_manager REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(position_controllers REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

# ============================================================
# Other dependencies
# ============================================================

find_package(Boost REQUIRED COMPONENTS system)

# ============================================================
# Message generation
# ============================================================

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointColorData.msg"
  "msg/PointColorDataArray.msg"
  DEPENDENCIES std_msgs
)

# ============================================================
# Gazebo
# ============================================================

include(FindPkgConfig)

if(PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
  include/catvehicle
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

# ============================================================
# Gazebo plugin
# ============================================================

add_library(catvehiclegazebo SHARED
  src/cont.cc
  include/catvehicle/cont.hh
)

ament_target_dependencies(catvehiclegazebo
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

target_link_libraries(
  catvehiclegazebo
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

# ============================================================
# distanceEstimator
# ============================================================

add_executable(
  distanceEstimator
  src/distanceEstimator.cpp
)

ament_target_dependencies(
  distanceEstimator
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ============================================================
# distanceEstimatorSteeringBased
# ============================================================

add_executable(
  distanceEstimatorSteeringBased
  src/distanceEstimatorSteeringBased.cpp
)

ament_target_dependencies(
  distanceEstimatorSteeringBased
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ============================================================
# Install package resources
# ============================================================

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Controller configuration
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Install executables and Gazebo plugin
# ============================================================

install(
  TARGETS
    catvehiclegazebo
    distanceEstimator
    distanceEstimatorSteeringBased
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Python executables
# ============================================================

install(
  PROGRAMS
    src/joy2cmdvel.py
    src/cmdvel2gazebo.py
    src/distanceEstimatorOdom.py
    src/drive_lead.py
    src/odom2path.py
    src/predict.py
    src/primitiveCmdVel.py
    src/velinjector.py
    script/fusion2pcl2.py
    script/test_multiplexer_v_2.py
    script/right_lane_follower.py
    script/test_injector_v_2.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Testing
# ============================================================

ament_package()
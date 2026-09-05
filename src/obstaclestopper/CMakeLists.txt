cmake_minimum_required(VERSION 3.5)

project(obstaclestopper)

set(CMAKE_VERBOSE_MAKEFILE ON)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ROS2 dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Threads REQUIRED)


add_definitions(
-DMODEL=obstacleStopper
-DNUMST=1
-DNCSTATES=0
-DHAVESTDIO
-DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
-DUNIX
-DCLASSIC_INTERFACE=0
-DALLOCATIONFCN=0
-DTID01EQ=0
-DTERMFCN=1
-DONESTEPFCN=1
-DMAT_FILE=0
-DMULTI_INSTANCE_CODE=0
-DINTEGER_CODE=0
-DMT=0
-DROS_PROJECT
-DMW_SCHED_OTHER
-DSTACK_SIZE=64
-D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
-DRT=RT
)


include_directories(
  ${PROJECT_SOURCE_DIR}
)


add_executable(obstacleStopper_node

  ert_main.cpp

  obstacleStopper.cpp
  obstacleStopper_data.cpp

  rtGetInf.cpp
  rtGetNaN.cpp
  rt_nonfinite.cpp

  linuxinitialize.cpp

  slros_busmsg_conversion.cpp
  slros_initialize.cpp
  slros_generic_param.cpp

)


ament_target_dependencies(
  obstacleStopper_node

  rclcpp

  geometry_msgs

  std_msgs
)


target_link_libraries(
  obstacleStopper_node

  rt

  Threads::Threads
)


install(
  TARGETS
    obstacleStopper_node

  DESTINATION
    lib/${PROJECT_NAME}
)


ament_package()
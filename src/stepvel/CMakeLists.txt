cmake_minimum_required(VERSION 3.8)

project(stepvel)

set(CMAKE_VERBOSE_MAKEFILE ON)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ROS2 dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_definitions(
-DMODEL=stepvel
-DNUMST=1
-DNCSTATES=0
-DHAVESTDIO
-DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
-DUNIX
-DCLASSIC_INTERFACE=0
-DALLOCATIONFCN=0
-DTID01EQ=0
-DONESTEPFCN=1
-DTERMFCN=1
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


add_executable(${PROJECT_NAME}_node

  ert_main.cpp

  stepvel.cpp
  stepvel_data.cpp

  linuxinitialize.cpp

  slros_busmsg_conversion.cpp
  slros_initialize.cpp
  slros_generic_param.cpp

)


ament_target_dependencies(${PROJECT_NAME}_node

  rclcpp
  geometry_msgs
  std_msgs

)


target_link_libraries(${PROJECT_NAME}_node

  rt

)


target_compile_options(${PROJECT_NAME}_node PRIVATE
  -fpermissive
)


install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
cmake_minimum_required(VERSION 3.5)

project(sicktoolbox_wrapper)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Threads REQUIRED)


###########
## Include ##
###########

include_directories(
  /home/student/ROS2/src/sicktoolbox/include
)


set(SICKTOOLBOX_LIB_DIR
  /home/student/ROS2/src/sicktoolbox/build/sicktoolbox
)


###########
## Build ##
###########

# sickld node

add_executable(sickld
  ros/sickld/sickld.cpp
)

ament_target_dependencies(
  sickld
  rclcpp
  sensor_msgs
  diagnostic_updater
)

target_link_libraries(
  sickld
  ${SICKTOOLBOX_LIB_DIR}/libSickLD.a
  ${SICKTOOLBOX_LIB_DIR}/libSickLMS1xx.a
  ${SICKTOOLBOX_LIB_DIR}/libSickLMS2xx.a
  Threads::Threads
)


# sicklms node

add_executable(sicklms
  ros/sicklms/sicklms.cpp
)

ament_target_dependencies(
  sicklms
  rclcpp
  sensor_msgs
  diagnostic_updater
)

target_link_libraries(
  sicklms
  ${SICKTOOLBOX_LIB_DIR}/libSickLMS2xx.a
  Threads::Threads
)



# standalone tools

add_executable(print_scans
  standalone/print_scans.cpp
)

target_link_libraries(
  print_scans
  ${SICKTOOLBOX_LIB_DIR}/libSickLMS2xx.a
  Threads::Threads
)



add_executable(time_scans
  standalone/time_scans.cpp
)

target_link_libraries(
  time_scans
  ${SICKTOOLBOX_LIB_DIR}/libSickLMS2xx.a
  Threads::Threads
)



add_executable(log_scans
  standalone/log_scans.cpp
)

target_link_libraries(
  log_scans
  ${SICKTOOLBOX_LIB_DIR}/libSickLMS2xx.a
  Threads::Threads
)



#############
## Install ##
#############

install(
  TARGETS
    sickld
    sicklms
    print_scans
    time_scans
    log_scans
  DESTINATION lib/${PROJECT_NAME}
)


install(
  FILES
    sick_test.vcg
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
cmake_minimum_required(VERSION 3.5)
project(control_toolbox)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(tinyxml_vendor REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Generate ROS2 services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPidGains.srv"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
)

# Library
add_library(control_toolbox_lib
  src/dither.cpp
  src/limited_proxy.cpp
  src/pid.cpp
  src/pid_gains_setter.cpp
  src/sine_sweep.cpp
  src/sinusoid.cpp
)

ament_target_dependencies(control_toolbox_lib
  rclcpp
  control_msgs
  realtime_tools
)

rosidl_target_interfaces(control_toolbox_lib
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(control_toolbox_lib
  ${Boost_LIBRARIES}
  tinyxml
)


# Tests
if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(pid_tests
    test/pid_tests.cpp
  )

  target_link_libraries(pid_tests
    control_toolbox_lib
    tinyxml
  )

endif()


# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


# Install library
install(
  TARGETS control_toolbox_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install scripts
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  control_msgs
  realtime_tools
  rosidl_default_runtime
  tinyxml_vendor
)

ament_package()
cmake_minimum_required(VERSION 3.8)

project(velodyne_driver)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(rclcpp_components REQUIRED)

find_package(diagnostic_updater REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(velodyne_msgs REQUIRED)

find_package(pluginlib REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)


# libpcap

set(libpcap_LIBRARIES
    -lpcap
)


include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)


# libraries

add_subdirectory(src/lib)

add_subdirectory(src/driver)



# install headers

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)



# install ROS2 launch files

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)



# install ROS2 component description

install(
    FILES velodyne_driver_component.xml
    DESTINATION share/${PROJECT_NAME}
)



# install executable scripts

install(
    PROGRAMS src/vdump
    DESTINATION lib/${PROJECT_NAME}
)



ament_export_dependencies(
    rclcpp
    rclcpp_components
    diagnostic_updater
    sensor_msgs
    velodyne_msgs
    pluginlib
)


ament_export_include_directories(include)


ament_package()
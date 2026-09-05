cmake_minimum_required(VERSION 3.5)

project(cmdvel2gazebo)


# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ROS2 dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_definitions(
-DMODEL=cmdvel2gazebo
-DNUMST=2
-DNCSTATES=2
-DHAVESTDIO
-DUNIX
-DTERMFCN=1
-DONESTEPFCN=1
-DMAT_FILE=0
-DMULTI_INSTANCE_CODE=0
-DINTEGER_CODE=0
-DMT=0
-DCLASSIC_INTERFACE=0
-DALLOCATIONFCN=0
-DTID01EQ=1
-DROS_PROJECT
-DMW_SCHED_OTHER
-DSTACK_SIZE=64
-D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
-DRT=RT
)


include_directories(
    .
    include
)


add_executable(${PROJECT_NAME}_node

    ert_main.cpp

    cmdvel2gazebo.cpp
    cmdvel2gazebo_data.cpp

    rtGetInf.cpp
    rtGetNaN.cpp
    rt_nonfinite.cpp

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

    pthread
    rt

)


install(
    TARGETS
        ${PROJECT_NAME}_node

    DESTINATION
        lib/${PROJECT_NAME}

)


ament_package()
#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include "cmdvel2gazebo_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(
    geometry_msgs::msg::Twist* msgPtr,
    SL_Bus_cmdvel2gazebo_geometry_msgs_Twist const* busPtr);

void convertToBus(
    SL_Bus_cmdvel2gazebo_geometry_msgs_Twist* busPtr,
    geometry_msgs::msg::Twist const* msgPtr);


void convertFromBus(
    geometry_msgs::msg::Vector3* msgPtr,
    SL_Bus_cmdvel2gazebo_geometry_msgs_Vector3 const* busPtr);

void convertToBus(
    SL_Bus_cmdvel2gazebo_geometry_msgs_Vector3* busPtr,
    geometry_msgs::msg::Vector3 const* msgPtr);


void convertFromBus(
    std_msgs::msg::Float64* msgPtr,
    SL_Bus_cmdvel2gazebo_std_msgs_Float64 const* busPtr);

void convertToBus(
    SL_Bus_cmdvel2gazebo_std_msgs_Float64* busPtr,
    std_msgs::msg::Float64 const* msgPtr);


#endif
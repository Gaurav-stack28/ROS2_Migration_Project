#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include "obstacleStopper_types.h"
#include "slros_msgconvert_utils.h"

void convertFromBus(
    geometry_msgs::msg::Twist* msgPtr,
    const SL_Bus_obstacleStopper_geometry_msgs_Twist* busPtr);
void convertToBus(
    SL_Bus_obstacleStopper_geometry_msgs_Twist* busPtr,
    const geometry_msgs::msg::Twist* msgPtr);

void convertFromBus(
    geometry_msgs::msg::Vector3* msgPtr,
    const SL_Bus_obstacleStopper_geometry_msgs_Vector3* busPtr);
void convertToBus(
    SL_Bus_obstacleStopper_geometry_msgs_Vector3* busPtr,
    const geometry_msgs::msg::Vector3* msgPtr);

void convertFromBus(
    std_msgs::msg::Float64* msgPtr,
    const SL_Bus_obstacleStopper_std_msgs_Float64* busPtr);
void convertToBus(
    SL_Bus_obstacleStopper_std_msgs_Float64* busPtr,
    const std_msgs::msg::Float64* msgPtr);

#endif
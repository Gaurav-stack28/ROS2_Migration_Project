#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "stepvel_types.h"
#include "slros_msgconvert_utils.h"

void convertFromBus(geometry_msgs::msg::Twist* msgPtr,
                    const SL_Bus_stepvel_geometry_msgs_Twist* busPtr);
void convertToBus(SL_Bus_stepvel_geometry_msgs_Twist* busPtr,
                  const geometry_msgs::msg::Twist* msgPtr);

void convertFromBus(geometry_msgs::msg::Vector3* msgPtr,
                    const SL_Bus_stepvel_geometry_msgs_Vector3* busPtr);
void convertToBus(SL_Bus_stepvel_geometry_msgs_Vector3* busPtr,
                  const geometry_msgs::msg::Vector3* msgPtr);

#endif

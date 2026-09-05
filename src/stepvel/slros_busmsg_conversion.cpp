#include "slros_busmsg_conversion.h"


// Conversion between SL_Bus_stepvel_geometry_msgs_Twist and geometry_msgs::msg::Twist

void convertFromBus(
    geometry_msgs::msg::Twist* msgPtr,
    const SL_Bus_stepvel_geometry_msgs_Twist* busPtr)
{
    convertFromBus(&msgPtr->angular,
                   &busPtr->Angular);

    convertFromBus(&msgPtr->linear,
                   &busPtr->Linear);
}


void convertToBus(
    SL_Bus_stepvel_geometry_msgs_Twist* busPtr,
    const geometry_msgs::msg::Twist* msgPtr)
{
    convertToBus(&busPtr->Angular,
                 &msgPtr->angular);

    convertToBus(&busPtr->Linear,
                 &msgPtr->linear);
}



// Conversion between SL_Bus_stepvel_geometry_msgs_Vector3 and geometry_msgs::msg::Vector3

void convertFromBus(
    geometry_msgs::msg::Vector3* msgPtr,
    const SL_Bus_stepvel_geometry_msgs_Vector3* busPtr)
{
    msgPtr->x = busPtr->X;
    msgPtr->y = busPtr->Y;
    msgPtr->z = busPtr->Z;
}


void convertToBus(
    SL_Bus_stepvel_geometry_msgs_Vector3* busPtr,
    const geometry_msgs::msg::Vector3* msgPtr)
{
    busPtr->X = msgPtr->x;
    busPtr->Y = msgPtr->y;
    busPtr->Z = msgPtr->z;
}
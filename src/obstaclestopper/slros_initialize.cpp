#include "slros_initialize.h"

#include<iostream>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>


std::shared_ptr<rclcpp::Node> SLROSNodePtr;
const std::string SLROSNodeName = "obstacleStopper";


// For Block obstacleStopper/Subscribe
SimulinkSubscriber<std_msgs::msg::Float64, SL_Bus_obstacleStopper_std_msgs_Float64> Sub_obstacleStopper_12;

// For Block obstacleStopper/Subscribe1
SimulinkSubscriber<geometry_msgs::msg::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Sub_obstacleStopper_13;

// For Block obstacleStopper/Subscribe2
SimulinkSubscriber<geometry_msgs::msg::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Sub_obstacleStopper_39;

// For Block obstacleStopper/Publish
SimulinkPublisher<geometry_msgs::msg::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Pub_obstacleStopper_17;


// ROS2 executor thread
void ros_spin_thread()
{
    rclcpp::spin(SLROSNodePtr);
}


void slros_node_init(int argc, char **argv)
{
    std::cout << "Starting ROS2 initialization" << std::endl;

    rclcpp::init(argc, argv);

    SLROSNodePtr = std::make_shared<rclcpp::Node>(SLROSNodeName);

    std::cout << "ROS2 node created: "
              << SLROSNodePtr->get_name()
              << std::endl;

    std::thread ros_thread(ros_spin_thread);
    ros_thread.detach();

    std::cout << "ROS2 spin thread started" << std::endl;
}
#include "slros_initialize.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>


std::shared_ptr<rclcpp::Node> SLROSNodePtr;

const std::string SLROSNodeName = "cmdvel2gazebo";


// For Block cmdvel2gazebo/Subscribe
SimulinkSubscriber<
    geometry_msgs::msg::Twist,
    SL_Bus_cmdvel2gazebo_geometry_msgs_Twist>
Sub_cmdvel2gazebo_3;


// For Block cmdvel2gazebo/Publish
SimulinkPublisher<
    std_msgs::msg::Float64,
    SL_Bus_cmdvel2gazebo_std_msgs_Float64>
Pub_cmdvel2gazebo_9;


// For Block cmdvel2gazebo/Publish1
SimulinkPublisher<
    std_msgs::msg::Float64,
    SL_Bus_cmdvel2gazebo_std_msgs_Float64>
Pub_cmdvel2gazebo_18;


// For Block cmdvel2gazebo/Publish2
SimulinkPublisher<
    std_msgs::msg::Float64,
    SL_Bus_cmdvel2gazebo_std_msgs_Float64>
Pub_cmdvel2gazebo_24;


// For Block cmdvel2gazebo/Publish3
SimulinkPublisher<
    std_msgs::msg::Float64,
    SL_Bus_cmdvel2gazebo_std_msgs_Float64>
Pub_cmdvel2gazebo_25;



void slros_node_init(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    SLROSNodePtr = std::make_shared<rclcpp::Node>(SLROSNodeName);
}
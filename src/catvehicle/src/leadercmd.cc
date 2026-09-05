#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>

// this global var holds the velocity
geometry_msgs::msg::Twist leader_vel;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("leadercmd");

    std::string leader_InputVel_topic = "/catvehicle/cmd_vel";

    auto leader_vel_pub =
        node->create_publisher<geometry_msgs::msg::Twist>(
            leader_InputVel_topic, 1);

    rclcpp::Rate loop_rate(100);

    double bias = 3.0;
    double sinecomp = 0.0;
    double t = 0.0;
    double pi = 3.14159265359;

    while (rclcpp::ok())
    {
        if (t > 100)
        {
            t = 0.0;
        }

        sinecomp = std::sin(t * 5 / (2 * pi));
        t += 0.005;

        leader_vel.linear.x = bias + sinecomp;

        leader_vel_pub->publish(leader_vel);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
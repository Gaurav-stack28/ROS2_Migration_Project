#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <stdlib.h>


namespace gazebo
{
class CatGPS : public ModelPlugin
{
public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        void OnUpdate(const common::UpdateInfo & /*_info*/);
private:
        void CatVehicleSimROSThread();
        void Callback(const nav_msgs::msg::Odometry::SharedPtr msg);


        //ROS
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        boost::thread ros_spinner_thread_;
        rclcpp::Node::SharedPtr rosnode_;

	//gazebo
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;

	//variables
	int k;
	double x , y , z , x_ang , y_ang , z_ang , omega , x_new , y_new , offset_x , offset_y , car_height;
	ignition::math::Vector3d vector;
	ignition::math::Pose3d _pose;
	ignition::math::Quaterniond quaternion;
};
}

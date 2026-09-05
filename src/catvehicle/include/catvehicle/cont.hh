#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include <gazebo/physics/Joint.hh>
#include <cstdlib>
//#include <unistd.h>
#include <gazebo_msgs/msg/model_states.hpp>
#include <ignition/math/Vector3.hh>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
namespace gazebo
{
    class CatSteering : public ModelPlugin
    {
        public:
            CatSteering();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            //void OnUpdate(const common::UpdateInfo & _info);

        private:
            void CatVehicleSimROSThread();
            void modelRead(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

            physics::PhysicsEnginePtr physicsEngine;
            //to read the name space from urdf file
            std::string robotNamespace;
            //to read the name space from urdf file
            std::string tfScope;
            //Name of the speed topic being published
            std::string speedTopic;
    	    // Name of the tire angle topic being published
	        std::string tireTopic;
            // Name of the odometry topic being published
            std::string odomTopic;
            //ROS
            rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros_pub;
            rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr steering_pub;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
            boost::thread ros_spinner_thread_;
            rclcpp::Node::SharedPtr rosnode_;
            
            //velocity vector to fetch velocity from model entity
	    ignition::math::Vector3<double> linear_vel;
	    ignition::math::Vector3<double> angular_vel;
            //Gazebo
            physics::JointPtr steering_joints[2];
            physics::JointController *j_cont;
            event::ConnectionPtr updateConnection;


            //Pointer to the model entity
            physics::ModelPtr model;
            //Pointer to the world in which the model exists
            physics::WorldPtr world;


            //rate at which to update the catsteering
            double updateRate;
            //Previous time when the catsteering was updated.
            rclcpp::Time prevUpdateTime;
    };
}

/*
Author: Rahul Kumar Bhadani, Jonathan Sprinkle
Copyright (c) 2018 Arizona Board of Regents
All rights reserved.

Permission is hereby granted, without written agreement and without 
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the 
above copyright notice and the following two paragraphs appear in 
all copies of this software.

IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

*/
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
// JMS added Wrench for steering info
#include <geometry_msgs/msg/wrench.hpp>
#include <gazebo/physics/Joint.hh>
#include <cstdlib>
#include <stdlib.h>
#include <pwd.h>
#include <stdio.h>
#include <sys/types.h>
#include<iostream>
#include<string>
#include<vector>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include "catvehicle/cont.hh"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std;

//string logFile = "//home//"+std::string(getpwuid (getuid())->pw_name)+"//azcar_speed//vData.mat";
//fstream logOut(logFile.c_str(),ios_base::out | ios_base::app);

namespace gazebo
{
    CatSteering::CatSteering()
    {
        /*Rahul added default values of update rate of odometry data*/
        this->updateRate = 100.0;
        this->prevUpdateTime = rclcpp::Clock().now();
    }

    void CatSteering::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model

        this->model = _parent;
        this->world = _parent->GetWorld();
        this->robotNamespace = "";
       
        /* Rahul Added updateRate tag in urdf file which can be used to control the publishing rate of
            odometry data. Following code reads the update rate value from urdf file and check if they
            are factor of real time update rate specified in the gazebo. If they are not just print
            warning message and proceed gracefully
        */
        //physicsEngine = (this->world)->GetPhysicsEngine();
        physicsEngine = (this->world)->Physics();
        //get the update rate from sdf
        if (_sdf->HasElement("updateRate"))
        {
            this->updateRate = _sdf->GetElement("updateRate")->Get<double>();
            //TO DO: Check if gazebo's update rate is multiple of update rate specified in the urdf file.
            if( fmod(physicsEngine->GetRealTimeUpdateRate(), this->updateRate) != 0)
            {
                RCLCPP_WARN_STREAM(rosnode_->get_logger(), "updateRate in urdf file is not a factor of real time update rate specified in gazebo");
                RCLCPP_WARN(rosnode_->get_logger(), "update rate = %f", this->updateRate);
            }
            else
            {
                RCLCPP_INFO(rosnode_->get_logger(), "update rate = %f", this->updateRate);
            }
        }  
        // get namespace from urdf .gazebo file which is actually passed to .gazebo file from .launch file
        if (_sdf->HasElement("robotNamespace"))
        {
            this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
        }  
        this->speedTopic = this->robotNamespace + "/vel";

        this->tireTopic = this->robotNamespace + "/steering";

        this->odomTopic = this->robotNamespace + "/odom";

        this->tfScope = this->robotNamespace.substr(1,this->robotNamespace.size()-1);


        //Start up ros_node
        int argc = 0;
        char** argv = NULL;
        rclcpp::init(argc, argv);
        rosnode_ = std::make_shared<rclcpp::Node>("cat_sim");

        ros_pub = rosnode_->create_publisher<geometry_msgs::msg::Twist>(  speedTopic, 1);
        steering_pub = rosnode_->create_publisher<geometry_msgs::msg::Wrench>(tireTopic, 1);
        odom_pub = rosnode_->create_publisher<nav_msgs::msg::Odometry>(odomTopic, 1);


        //rosnode_ = new ros::NodeHandle(robot_namespace);
        this->ros_spinner_thread_ = boost::thread( boost::bind( &CatSteering::CatVehicleSimROSThread, this ) );
    } //end Load

    void CatSteering::CatVehicleSimROSThread()
    {
        RCLCPP_INFO_STREAM(rosnode_->get_logger(),"$ Callback thread id=" << boost::this_thread::get_id());

        //Added by Rahul
        
        //modelRead calback function is necessary in order to get instantaneous velocity vector
        sub_ = this->rosnode_->create_subscription<gazebo_msgs::msg::ModelStates>("gazebo/model_states",1,std::bind(&CatSteering::modelRead, this, std::placeholders::_1));

        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok())
        {
            rclcpp::spin_some(rosnode_);
            loop_rate.sleep( );       
        }
    } //end CatVehicleSimROSThread

    //Added by Rahul
    void CatSteering::modelRead(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        
           /* Rahul added code to control the update rate of odometry data*/
           rclcpp::Duration duration = rosnode_->get_clock()->now() - this->prevUpdateTime;
           if (duration.seconds() < 1.0/(this->updateRate))
           {
                return;
           }
         

       // gazebo::common::Time::MSleep(10);


        rclcpp::Time current_time = rosnode_->get_clock()->now();
        vector<string> modelNames = msg->name;
        int index=-1;

        // figure out which index we are in the msg list of model states
        for( int i=0; i<modelNames.size() && index<0; i++ )
        {
            if( this->robotNamespace == std::string("/"+modelNames[i]) )
            {
                //                ROS_INFO_STREAM(this->robotNamespace << " comparing to " << std::string("/"+modelNames[i]) << "[" << i << "]");
                index = i;
            }
        }

        double Vx, Vy, Vz, V;
        //Variable to story velocity vector that will be used  for publishing on speed topic
        geometry_msgs::msg::Twist out_vel;
        // JMS: output to give steering information
        geometry_msgs::msg::Wrench steering_msg;

        linear_vel = model->RelativeLinearVel();
        angular_vel = model->RelativeAngularVel();
        Vx = out_vel.linear.x = linear_vel.X();
        Vy = out_vel.linear.y = linear_vel.Y();
        Vz = out_vel.linear.z = linear_vel.Z();
        out_vel.angular.x = angular_vel.X();
        out_vel.angular.y = angular_vel.Y();
        out_vel.angular.z = angular_vel.Z();
        V = sqrt(Vx*Vx + Vy*Vy + Vz*Vz);


        //Publish the velocity as string to speed topic
        ros_pub->publish(out_vel);

        // JMS: get information about the steering joints
        physics::JointPtr steering_joints[2];
        steering_joints[0] = model->GetJoint("front_left_steering_joint");
        steering_joints[1] = model->GetJoint("front_right_steering_joint");
        //physics::JointState j_state0 = new physics::JointState(steering_joints[0]);
        //physics::JointState j_state1 = new physics::JointState(steering_joints[1]);

        double a0,a1;
        a0 = steering_joints[0]->Position(0);
        //a1 = steering_joints[1]->GetAngle(0).Radian();
        a1 = steering_joints[1]->Position(0);
        // average these values, though in most modes they will be equal

        steering_msg.torque.z = (a0+a1)/2.0;
        steering_pub->publish(steering_msg);

        if( index == -1 )
        {
            RCLCPP_ERROR_STREAM(rosnode_->get_logger(),"Unable to find odometry for model name "<< this->robotNamespace << "=" << index);
        }
        else
        {
            tf2_ros::TransformBroadcaster br(rosnode_);
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = current_time; 
            transform.header.frame_id = this->tfScope + "/odom"; 
            transform.child_frame_id = this->tfScope + "/base_link";
            transform.transform.translation.x = msg->pose[index].position.x;
            transform.transform.translation.y = msg->pose[index].position.y;
            transform.transform.translation.z = msg->pose[index].position.z;
            transform.transform.rotation.x = msg->pose[index].orientation.x;
            transform.transform.rotation.y = msg->pose[index].orientation.y;
            transform.transform.rotation.z = msg->pose[index].orientation.z;
            transform.transform.rotation.w = msg->pose[index].orientation.w; 
            br.sendTransform(transform);
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = this->tfScope + "/odom";
            odom.child_frame_id = this->tfScope + "/base_link";
            odom.pose.pose = msg->pose[index];
            odom.twist.twist = msg->twist[index];
            odom_pub->publish(odom);
            this->prevUpdateTime = rosnode_->get_clock()->now();
        }
    }
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(CatSteering)
}

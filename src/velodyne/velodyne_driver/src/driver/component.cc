// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
// ROS2 migration

/** \file
 *
 *  ROS2 component wrapper for the Velodyne 3D LIDAR driver
 */

#include <memory>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "velodyne_driver/driver.h"


namespace velodyne_driver
{


class DriverComponent : public rclcpp::Node
{

public:

  explicit DriverComponent(
      const rclcpp::NodeOptions & options)
  : Node(
      "velodyne_driver",
      options),
    running_(false)
  {

    RCLCPP_INFO(
        this->get_logger(),
        "Starting Velodyne driver component");


    // initialize driver after node is created
    driver_ =
        std::make_shared<VelodyneDriver>(
            this->shared_from_this());


    running_ = true;


    device_thread_ =
        std::make_shared<std::thread>(
            &DriverComponent::devicePoll,
            this);

  }



  ~DriverComponent()
  {

    RCLCPP_INFO(
        this->get_logger(),
        "Stopping Velodyne driver component");


    running_ = false;


    if(device_thread_ &&
       device_thread_->joinable())
    {
      device_thread_->join();
    }


    RCLCPP_INFO(
        this->get_logger(),
        "Velodyne driver stopped");

  }



private:


  void devicePoll()
  {

    while(rclcpp::ok() && running_)
    {

      bool result =
          driver_->poll();


      if(!result)
      {

        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Driver polling failed");

      }


      // avoid 100% CPU usage
      std::this_thread::sleep_for(
          std::chrono::milliseconds(1));

    }


    running_ = false;

  }



  std::shared_ptr<VelodyneDriver> driver_;


  std::shared_ptr<std::thread> device_thread_;


  std::atomic<bool> running_;


};



} // namespace velodyne_driver



RCLCPP_COMPONENTS_REGISTER_NODE(
    velodyne_driver::DriverComponent)
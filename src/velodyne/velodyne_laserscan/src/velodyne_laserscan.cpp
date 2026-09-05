// Copyright (C) 2018, 2019 Kevin Hallenbeck, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Copyright (C) 2018, 2019 Kevin Hallenbeck, Joshua Whitley
// ROS2 Foxy migration

#include "velodyne_laserscan/velodyne_laserscan.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <limits>
#include <functional>
#include <cstdint>


namespace velodyne_laserscan
{

VelodyneLaserScan::VelodyneLaserScan()
: Node("velodyne_laserscan"),
  ring_count_(0)
{
  this->declare_parameter<int>("ring", -1);
  this->declare_parameter<double>("resolution", 0.007);


  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan",
      10);


  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points",
      10,
      std::bind(
          &VelodyneLaserScan::recvCallback,
          this,
          std::placeholders::_1));

}



void VelodyneLaserScan::connectCb()
{
}



void VelodyneLaserScan::recvCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  if (!ring_count_)
  {

    bool found = false;


    for (size_t i = 0; i < msg->fields.size(); i++)
    {
      if (msg->fields[i].datatype ==
          sensor_msgs::msg::PointField::UINT16 &&
          msg->fields[i].name == "ring")
      {
        found = true;
        break;
      }
    }


    if (!found)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "PointCloud2 does not contain UINT16 ring field");
      return;
    }



    for (
      sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg,"ring");
      it != it.end();
      ++it)
    {

      uint16_t ring = *it;

      if (ring + 1 > ring_count_)
      {
        ring_count_ = ring + 1;
      }
    }


    RCLCPP_INFO(
        this->get_logger(),
        "Detected %u lidar rings",
        ring_count_);

  }



  int ring_parameter;

  this->get_parameter(
      "ring",
      ring_parameter);



  uint16_t ring;



  if (ring_parameter < 0 ||
      ring_parameter >= static_cast<int>(ring_count_))
  {

    if (ring_count_ > 32)
      ring = 57;

    else if (ring_count_ > 16)
      ring = 23;

    else
      ring = 8;

  }
  else
  {
    ring = static_cast<uint16_t>(ring_parameter);
  }



  double resolution;

  this->get_parameter(
      "resolution",
      resolution);



  const float RESOLUTION =
      std::abs(resolution);



  const size_t SIZE =
      static_cast<size_t>(
          (2.0 * M_PI) / RESOLUTION);



  auto scan =
      std::make_shared<sensor_msgs::msg::LaserScan>();


  scan->header = msg->header;

  scan->angle_min = -M_PI;
  scan->angle_max = M_PI;

  scan->angle_increment =
      RESOLUTION;

  scan->range_min = 0.0;
  scan->range_max = 200.0;


  scan->ranges.resize(
      SIZE,
      std::numeric_limits<float>::infinity());



  bool has_intensity = false;


  for (auto &field : msg->fields)
  {
    if (field.name == "intensity")
    {
      has_intensity = true;
      break;
    }
  }



  if (has_intensity)
  {
    scan->intensities.resize(SIZE);
  }



  sensor_msgs::PointCloud2ConstIterator<float> iter_x(
      *msg,
      "x");


  sensor_msgs::PointCloud2ConstIterator<float> iter_y(
      *msg,
      "y");


  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(
      *msg,
      "ring");



  if (has_intensity)
  {

    sensor_msgs::PointCloud2ConstIterator<float> iter_i(
        *msg,
        "intensity");


    for(;
        iter_r != iter_r.end();
        ++iter_x,
        ++iter_y,
        ++iter_r,
        ++iter_i)
    {

      if (*iter_r != ring)
        continue;


      float x = *iter_x;
      float y = *iter_y;


      int bin =
        static_cast<int>(
          (atan2f(y,x)+M_PI) /
          RESOLUTION);



      if(bin >=0 &&
         bin < static_cast<int>(SIZE))
      {

        scan->ranges[bin] =
            sqrtf(
                x*x +
                y*y);


        scan->intensities[bin] =
            *iter_i;

      }

    }

  }
  else
  {

    for(;
        iter_r != iter_r.end();
        ++iter_x,
        ++iter_y,
        ++iter_r)
    {

      if(*iter_r != ring)
        continue;


      float x = *iter_x;
      float y = *iter_y;


      int bin =
        static_cast<int>(
          (atan2f(y,x)+M_PI)
          /
          RESOLUTION);



      if(bin >=0 &&
         bin < static_cast<int>(SIZE))
      {

        scan->ranges[bin] =
            sqrtf(
                x*x +
                y*y);

      }

    }

  }



  pub_->publish(*scan);

}


} // namespace velodyne_laserscan
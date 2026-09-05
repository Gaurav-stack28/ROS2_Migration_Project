// Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack O'Quin
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

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include "velodyne_driver/driver.h"



namespace velodyne_driver
{


VelodyneDriver::VelodyneDriver(
    rclcpp::Node::SharedPtr node)
:
node_(node),
diagnostics_(node_)
{

  node_->declare_parameter(
      "frame_id",
      "velodyne");


  node_->get_parameter(
      "frame_id",
      config_.frame_id);



  node_->declare_parameter(
      "model",
      "64E");


  node_->get_parameter(
      "model",
      config_.model);



  double packet_rate = 2600.0;

  std::string model_full_name;



  if(config_.model == "VLS128")
  {

    packet_rate = 6253.9;

    model_full_name =
        "VLS128";

  }

  else if(
      config_.model == "64E_S2" ||
      config_.model == "64E_S2.1")
  {

    packet_rate = 3472.17;

    model_full_name =
        "HDL-" + config_.model;

  }

  else if(config_.model == "64E")
  {

    packet_rate = 2600.0;

    model_full_name =
        "HDL-64E";

  }

  else if(config_.model == "64E_S3")
  {

    packet_rate = 5787.03;

    model_full_name =
        "HDL-" + config_.model;

  }

  else if(config_.model == "32E")
  {

    packet_rate = 1808.0;

    model_full_name =
        "HDL-32E";

  }

  else if(config_.model == "32C")
  {

    packet_rate = 1507.0;

    model_full_name =
        "VLP-32C";

  }

  else if(config_.model == "VLP16")
  {

    packet_rate = 754.0;

    model_full_name =
        "VLP-16";

  }

  else
  {

    RCLCPP_ERROR(
        node_->get_logger(),
        "Unknown Velodyne model");


    model_full_name =
        "Unknown";

  }



  RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "Starting "
      << model_full_name);



  node_->declare_parameter(
      "rpm",
      600.0);



  node_->get_parameter(
      "rpm",
      config_.rpm);



  double frequency =
      config_.rpm / 60.0;



  config_.npackets =
      static_cast<int>(
          ceil(packet_rate / frequency));



  RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "Packets per scan: "
      << config_.npackets);



  node_->declare_parameter(
      "timestamp_first_packet",
      false);



  node_->get_parameter(
      "timestamp_first_packet",
      config_.timestamp_first_packet);



  std::string dump_file;


  node_->declare_parameter(
      "pcap",
      "");


  node_->get_parameter(
      "pcap",
      dump_file);



  node_->declare_parameter(
      "cut_angle",
      -0.01);



  double cut_angle;


  node_->get_parameter(
      "cut_angle",
      cut_angle);



  if(cut_angle < 0.0)
  {

    config_.cut_angle = -1;

  }

  else
  {

    config_.cut_angle =
        static_cast<int>(
          cut_angle *
          360.0 /
          (2.0 * M_PI) *
          100.0);

  }



  int udp_port;



  node_->declare_parameter(
      "port",
      static_cast<int>(
          DATA_PORT_NUMBER));



  node_->get_parameter(
      "port",
      udp_port);

    node_->declare_parameter(
      "time_offset",
      0.0);


  node_->get_parameter(
      "time_offset",
      config_.time_offset);



  node_->declare_parameter(
      "enabled",
      true);


  node_->get_parameter(
      "enabled",
      config_.enabled);



  /*
   * Diagnostics setup
   */

  diagnostics_.setHardwareID(
      model_full_name);



  double diag_frequency =
      packet_rate /
      config_.npackets;



  diag_min_freq_ =
      diag_frequency;


  diag_max_freq_ =
      diag_frequency;



  double diagnostic_frequency_tolerance;



  node_->declare_parameter(
      "diagnostic_frequency_tolerance",
      0.1);



  node_->get_parameter(
      "diagnostic_frequency_tolerance",
      diagnostic_frequency_tolerance);



  diag_topic_ =
      std::make_shared<
          diagnostic_updater::TopicDiagnostic>(
              "velodyne_packets",
              diagnostics_,
              diagnostic_updater::FrequencyStatusParam(
                  &diag_min_freq_,
                  &diag_max_freq_,
                  diagnostic_frequency_tolerance,
                  10),
              diagnostic_updater::TimeStampStatusParam());



  diag_timer_ =
      node_->create_wall_timer(
          std::chrono::milliseconds(200),
          std::bind(
              &VelodyneDriver::diagTimerCallback,
              this));



  /*
   * Create Velodyne input
   */


  if(!dump_file.empty())
  {

    RCLCPP_INFO(
        node_->get_logger(),
        "Using PCAP input");


    input_ =
        std::make_shared<InputPCAP>(
            node_,
            udp_port,
            packet_rate,
            dump_file,
            false,
            false,
            0.0);

  }

  else
  {

    RCLCPP_INFO(
        node_->get_logger(),
        "Using UDP input");


    input_ =
        std::make_shared<InputSocket>(
            node_,
            udp_port);

  }



  /*
   * Publisher
   */


  output_ =
      node_->create_publisher<
          velodyne_msgs::msg::VelodyneScan>(
              "velodyne_packets",
              10);



  last_azimuth_ = -1;



  RCLCPP_INFO(
      node_->get_logger(),
      "Velodyne driver initialized");

}    

bool VelodyneDriver::poll()
{

  if(!config_.enabled)
  {

    rclcpp::sleep_for(
        std::chrono::seconds(1));

    return true;

  }



  auto scan =
      std::make_shared<
          velodyne_msgs::msg::VelodyneScan>();



  /*
   * Cut angle mode
   */

  if(config_.cut_angle >= 0)
  {

    scan->packets.reserve(
        config_.npackets);



    velodyne_msgs::msg::VelodynePacket tmp_packet;



    while(true)
    {

      while(true)
      {

        int rc =
            input_->getPacket(
                &tmp_packet,
                config_.time_offset);



        if(rc == 1)
        {
          break;
        }


        if(rc < 0)
        {
          return false;
        }


        if(rc == 0)
        {
          continue;
        }

      }



      scan->packets.push_back(
          tmp_packet);



      /*
       * Extract azimuth
       */

      std::size_t azimuth_data_pos = 2;


      int azimuth =
          *(reinterpret_cast<uint16_t *>(
              &tmp_packet.data[
                  azimuth_data_pos]));



      if(last_azimuth_ == -1)
      {

        last_azimuth_ = azimuth;

        continue;

      }



      if(
          (last_azimuth_ < config_.cut_angle &&
           config_.cut_angle <= azimuth)

          ||

          (config_.cut_angle <= azimuth &&
           azimuth < last_azimuth_)

          ||

          (azimuth < last_azimuth_ &&
           last_azimuth_ < config_.cut_angle)
        )
      {

        last_azimuth_ = azimuth;

        break;

      }



      last_azimuth_ = azimuth;


    }


  }


  /*
   * Normal scan mode
   */

  else
  {

    scan->packets.resize(
        config_.npackets);



    for(int i = 0;
        i < config_.npackets;
        i++)
    {


      while(true)
      {

        int rc =
            input_->getPacket(
                &scan->packets[i],
                config_.time_offset);



        if(rc == 1)
        {
          break;
        }


        if(rc < 0)
        {
          return false;
        }


        if(rc == 0)
        {
          continue;
        }

      }

    }

  }



  /*
   * Timestamp
   */

  if(config_.timestamp_first_packet)
  {

    scan->header.stamp =
        scan->packets.front().stamp;

  }

  else
  {

    scan->header.stamp =
        scan->packets.back().stamp;

  }



  scan->header.frame_id =
      config_.frame_id;



  /*
   * Publish scan
   */

  output_->publish(
      *scan);



  /*
   * Update diagnostics
   */

  if(diag_topic_)
  {

    diag_topic_->tick(
        scan->header.stamp);

  }


  diagnostics_.force_update();



  RCLCPP_DEBUG(
      node_->get_logger(),
      "Published Velodyne scan (%zu packets)",
      scan->packets.size());



  return true;

}




void VelodyneDriver::diagTimerCallback()
{

  diagnostics_.force_update();

}



} // namespace velodyne_driver
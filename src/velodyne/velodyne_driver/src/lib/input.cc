// Copyright (C) 2007, 2009, 2010, 2015 Austin Robot Technology, Patrick Beeson, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <pcap.h>
#include <poll.h>
#include <string>
#include <sstream>
#include <cstring>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <velodyne_driver/input.h>
#include <velodyne_driver/time_conversion.hpp>

namespace velodyne_driver
{

static const size_t packet_size =
  sizeof(velodyne_msgs::msg::VelodynePacket().data);



//////////////////////////////////////////////////////////////////////
// Input base class implementation
//////////////////////////////////////////////////////////////////////

Input::Input(rclcpp::Node::SharedPtr node, uint16_t port):
  node_(node),
  port_(port)
{
  node_->declare_parameter<std::string>("device_ip", "");
  node_->declare_parameter<bool>("gps_time", false);

  node_->get_parameter("device_ip", devip_str_);
  node_->get_parameter("gps_time", gps_time_);

  if (!devip_str_.empty())
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Only accepting packets from IP address: %s",
      devip_str_.c_str());
  }
}



//////////////////////////////////////////////////////////////////////
// InputSocket class implementation
//////////////////////////////////////////////////////////////////////

InputSocket::InputSocket(
    rclcpp::Node::SharedPtr node,
    uint16_t port):
  Input(node, port)
{
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }


  RCLCPP_INFO(
      node_->get_logger(),
      "Opening UDP socket: port %d",
      port);


  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);

  if (sockfd_ == -1)
  {
    perror("socket");
    return;
  }


  sockaddr_in my_addr;

  memset(
      &my_addr,
      0,
      sizeof(my_addr));


  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(port);
  my_addr.sin_addr.s_addr = INADDR_ANY;


  int val = 1;

  if (setsockopt(
        sockfd_,
        SOL_SOCKET,
        SO_REUSEADDR,
        &val,
        sizeof(val)) == -1)
  {
    perror("socketopt");
    return;
  }


  if (bind(
        sockfd_,
        (sockaddr *)&my_addr,
        sizeof(sockaddr)) == -1)
  {
    perror("bind");
    return;
  }


  if (fcntl(
        sockfd_,
        F_SETFL,
        O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }


  RCLCPP_DEBUG(
      node_->get_logger(),
      "Velodyne socket fd is %d",
      sockfd_);
}



InputSocket::~InputSocket()
{
  (void)close(sockfd_);
}

//////////////////////////////////////////////////////////////////////
// InputSocket getPacket implementation
//////////////////////////////////////////////////////////////////////

int InputSocket::getPacket(
    velodyne_msgs::msg::VelodynePacket *pkt,
    const double time_offset)
{
  double time1 =
      node_->get_clock()->now().seconds();


  struct pollfd fds[1];

  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;


  static const int POLL_TIMEOUT = 1000;


  sockaddr_in sender_address;

  socklen_t sender_address_len =
      sizeof(sender_address);



  while (true)
  {

    do
    {

      int retval =
          poll(
              fds,
              1,
              POLL_TIMEOUT);


      if (retval < 0)
      {

        if (errno != EINTR)
        {
          RCLCPP_ERROR(
              node_->get_logger(),
              "poll() error: %s",
              strerror(errno));
        }

        return -1;
      }


      if (retval == 0)
      {

        RCLCPP_WARN(
            node_->get_logger(),
            "Velodyne poll() timeout");

        return 0;
      }


      if ((fds[0].revents & POLLERR) ||
          (fds[0].revents & POLLHUP) ||
          (fds[0].revents & POLLNVAL))
      {

        RCLCPP_ERROR(
            node_->get_logger(),
            "poll() reports Velodyne error");

        return -1;
      }


    } while ((fds[0].revents & POLLIN) == 0);



    ssize_t nbytes =
        recvfrom(
            sockfd_,
            &pkt->data[0],
            packet_size,
            0,
            (sockaddr *)&sender_address,
            &sender_address_len);



    if (nbytes < 0)
    {

      if (errno != EWOULDBLOCK)
      {

        perror("recvfail");

        RCLCPP_INFO(
            node_->get_logger(),
            "recvfail");

        return -1;
      }

    }

    else if ((size_t)nbytes == packet_size)
    {

      if (devip_str_ != "" &&
          sender_address.sin_addr.s_addr != devip_.s_addr)
      {
        continue;
      }

      else
      {
        break;
      }
    }


    RCLCPP_DEBUG(
        node_->get_logger(),
        "incomplete Velodyne packet read: %ld bytes",
        nbytes);

  }



  if (!gps_time_)
  {

    double time2 =
        node_->get_clock()->now().seconds();


    double stamp =
        (time2 + time1) / 2.0 + time_offset;


    pkt->stamp =
        rclcpp::Time(
            static_cast<int64_t>(stamp * 1e9));

  }

  else
  {

    pkt->stamp =
        rosTimeFromGpsTimestamp(
            &(pkt->data[1200]));

  }


  return 1;
}


//////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
//////////////////////////////////////////////////////////////////////

InputPCAP::InputPCAP(
    rclcpp::Node::SharedPtr node,
    uint16_t port,
    double packet_rate,
    std::string filename,
    bool read_once,
    bool read_fast,
    double repeat_delay):

  Input(node, port),
  packet_rate_(packet_rate),
  filename_(filename)
{

  pcap_ = NULL;
  empty_ = true;


  node_->declare_parameter<bool>("read_once", false);
  node_->declare_parameter<bool>("read_fast", false);
  node_->declare_parameter<double>("repeat_delay", 0.0);
  node_->declare_parameter<bool>("pcap_time", false);


  node_->get_parameter(
      "read_once",
      read_once_);

  node_->get_parameter(
      "read_fast",
      read_fast_);

  node_->get_parameter(
      "repeat_delay",
      repeat_delay_);

  node_->get_parameter(
      "pcap_time",
      pcap_time_);



  if (read_once_)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "Read input file only once.");
  }


  if (read_fast_)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "Read input file as quickly as possible.");
  }


  if (repeat_delay_ > 0.0)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "Delay %.3f seconds before repeating input file.",
        repeat_delay_);
  }



  RCLCPP_INFO(
      node_->get_logger(),
      "Opening PCAP file \"%s\"",
      filename_.c_str());



  if ((pcap_ =
       pcap_open_offline(
           filename_.c_str(),
           errbuf_)) == NULL)
  {

    RCLCPP_FATAL(
        node_->get_logger(),
        "Error opening Velodyne socket dump file.");

    return;
  }



  std::stringstream filter;


  if (devip_str_ != "")
  {
    filter
        << "src host "
        << devip_str_
        << " && ";
  }


  filter
      << "udp dst port "
      << port;



  pcap_compile(
      pcap_,
      &pcap_packet_filter_,
      filter.str().c_str(),
      1,
      PCAP_NETMASK_UNKNOWN);

}



InputPCAP::~InputPCAP()
{
  pcap_close(pcap_);
}





int InputPCAP::getPacket(
    velodyne_msgs::msg::VelodynePacket *pkt,
    const double time_offset)
{

  struct pcap_pkthdr *header;

  const u_char *pkt_data;



  while(true)
  {

    int res;


    if ((res =
        pcap_next_ex(
            pcap_,
            &header,
            &pkt_data)) >= 0)
    {


      if (0 ==
          pcap_offline_filter(
              &pcap_packet_filter_,
              header,
              pkt_data))
      {
        continue;
      }



      if (!read_fast_)
      {
        packet_rate_.sleep();
      }



      memcpy(
          &pkt->data[0],
          pkt_data + 42,
          packet_size);



      if (!gps_time_)
      {


        if (!pcap_time_)
        {

          pkt->stamp =
              node_->get_clock()->now();

        }

        else
        {

          pkt->stamp =
              rclcpp::Time(
                  header->ts.tv_sec * 1000000000LL +
                  header->ts.tv_usec * 1000);

        }

      }

      else
      {

        pkt->stamp =
            rosTimeFromGpsTimestamp(
                &(pkt->data[1200]),
                header);

      }



      empty_ = false;

      return 1;

    }




    if (empty_)
    {

      RCLCPP_WARN(
          node_->get_logger(),
          "Error %d reading Velodyne packet: %s",
          res,
          pcap_geterr(pcap_));

      return -1;

    }




    if (read_once_)
    {

      RCLCPP_INFO(
          node_->get_logger(),
          "end of file reached -- done reading.");

      return -1;

    }




    if (repeat_delay_ > 0.0)
    {

      RCLCPP_INFO(
          node_->get_logger(),
          "end of file reached -- delaying %.3f seconds.",
          repeat_delay_);


      usleep(
          rint(
              repeat_delay_ *
              1000000.0));

    }




    RCLCPP_DEBUG(
        node_->get_logger(),
        "replaying Velodyne dump file");



    pcap_close(pcap_);


    pcap_ =
        pcap_open_offline(
            filename_.c_str(),
            errbuf_);


    empty_ = true;

  }

}


} // namespace velodyne_driver
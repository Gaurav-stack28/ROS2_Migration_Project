// Copyright (C) 2019 Matthew Pitropov, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)

#ifndef VELODYNE_DRIVER_TIME_CONVERSION_HPP
#define VELODYNE_DRIVER_TIME_CONVERSION_HPP


#include <pcap.h>

#include <rclcpp/rclcpp.hpp>



/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct.
 */
rclcpp::Time resolveHourAmbiguity(
    const rclcpp::Time &stamp,
    const rclcpp::Time &nominal_stamp)
{

    const int HALFHOUR_TO_SEC = 1800;


    rclcpp::Time retval = stamp;


    int64_t stamp_sec =
        stamp.seconds();


    int64_t nominal_sec =
        nominal_stamp.seconds();



    if (nominal_sec > stamp_sec)
    {

        if (nominal_sec - stamp_sec > HALFHOUR_TO_SEC)
        {
            retval =
                rclcpp::Time(
                    stamp.nanoseconds()
                    +
                    (2 * HALFHOUR_TO_SEC *
                     1000000000LL));
        }

    }

    else if (stamp_sec - nominal_sec > HALFHOUR_TO_SEC)
    {

        retval =
            rclcpp::Time(
                stamp.nanoseconds()
                -
                (2 * HALFHOUR_TO_SEC *
                 1000000000LL));

    }


    return retval;
}





rclcpp::Time rosTimeFromGpsTimestamp(
    const uint8_t * const data,
    const struct pcap_pkthdr *header = NULL)
{

    const int HOUR_TO_SEC = 3600;


    // time for each packet is a 4 byte uint
    // It is the number of microseconds from the top of the hour

    uint32_t usecs =
        (uint32_t)
        (
          ((uint32_t)data[3]) << 24 |
          ((uint32_t)data[2]) << 16 |
          ((uint32_t)data[1]) << 8  |
          ((uint32_t)data[0])
        );



    rclcpp::Time time_nom;



    // if header is NULL, assume real time operation

    if (!header)
    {

        time_nom =
            rclcpp::Clock(
                RCL_SYSTEM_TIME).now();

    }

    else
    {

        time_nom =
            rclcpp::Time(
                header->ts.tv_sec *
                1000000000LL
                +
                header->ts.tv_usec *
                1000);

    }



    uint64_t cur_hour =
        static_cast<uint64_t>(
            time_nom.seconds()
        )
        /
        HOUR_TO_SEC;



    rclcpp::Time stamp(
        (cur_hour * HOUR_TO_SEC +
        (usecs / 1000000))
        *
        1000000000LL
        +
        ((usecs % 1000000)
        * 1000));



    stamp =
        resolveHourAmbiguity(
            stamp,
            time_nom);



    return stamp;

}



#endif // VELODYNE_DRIVER_TIME_CONVERSION_HPP
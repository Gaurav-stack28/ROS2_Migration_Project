/*
 * Authors: Nick Hillier and Fred Pauling (CSIRO, 2011)
 * 
 * Based on the sicklms.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 * 
 * Released under BSD license.
 */ 

#include <memory>
#include <iostream>
#include <cmath>
#include <sicktoolbox/SickLD.hh>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <deque>

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace std;
using namespace SickToolbox;


// TODO: refactor these functions into a common util lib (similar to code in sicklms.cpp)

void publish_scan(
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub,
    double *range_values,
    uint32_t n_range_values,
    unsigned int *intensity_values,
    uint32_t n_intensity_values,
    rclcpp::Time start,
    double scan_time,
    bool inverted,
    float angle_min,
    float angle_max,
    std::string frame_id)
{
    sensor_msgs::msg::LaserScan scan_msg;

    scan_msg.header.frame_id = frame_id;

    if (inverted) { 
        // assumes scan window at the bottom
        scan_msg.angle_min = angle_max;
        scan_msg.angle_max = angle_min;
    } 
    else {
        scan_msg.angle_min = angle_min;
        scan_msg.angle_max = angle_max;
    }

    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) /
        (double)(n_range_values - 1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / n_range_values;

    scan_msg.range_min = 0.5;
    scan_msg.range_max = 250.0;

    scan_msg.ranges.resize(n_range_values);

    scan_msg.header.stamp = start;

    for (size_t i = 0; i < n_range_values; i++) {
        scan_msg.ranges[i] = (float)range_values[i];
    }


    scan_msg.intensities.resize(n_intensity_values);

    for (size_t i = 0; i < n_intensity_values; i++) {
        scan_msg.intensities[i] = (float)intensity_values[i];
    }

    pub->publish(scan_msg);
}



// A complimentary filter to get a (much) better time estimate

class smoothtime 
{ 
protected:

    rclcpp::Time smoothtime_prev;
    rclcpp::Time smoothed_timestamp;

    double time_smoothing_factor;
    double error_threshold;


public:

    smoothtime()
    {
        time_smoothing_factor = 0.95;
        error_threshold = 0.50;
    }


    //! Between 0 and 1, bigger is smoother
    void set_smoothing_factor(double smoothing_factor)
    { 
        time_smoothing_factor = smoothing_factor;
    }


    //! Between 0 and 1
    void set_error_threshold(double err_threshold)
    { 
        error_threshold = err_threshold;
    }


    rclcpp::Time smooth_timestamp(
        rclcpp::Time recv_timestamp,
        rclcpp::Duration expctd_dur)
    {

        if (smoothtime_prev.nanoseconds() == 0) {

            smoothed_timestamp = recv_timestamp;

        } 
        else {

            smoothed_timestamp =
                smoothtime_prev + expctd_dur;


            double err =
                (recv_timestamp - smoothed_timestamp).seconds();


            double time_error_threshold =
                expctd_dur.seconds() * error_threshold;



            if ((time_smoothing_factor > 0) &&
                (fabs(err) < time_error_threshold))
            {

                rclcpp::Duration correction =
                    rclcpp::Duration::from_seconds(
                        err * (1 - time_smoothing_factor));


                smoothed_timestamp += correction;

            } 
            else {

                smoothed_timestamp = recv_timestamp;

            }
        }


        smoothtime_prev = smoothed_timestamp;

        return smoothed_timestamp;
    }
};


class averager 
{
protected:

    std::deque<double> deq;

    unsigned int max_len;

    double sum = 0.0;


public:

    averager(int max_len = 50)
    {
        this->max_len = max_len;
    }


    void add_new(double data)
    {

        deq.push_back(data);

        sum += data;


        if (deq.size() > max_len)
        {

            sum -= deq.front();

            deq.pop_front();

        }
    }


    double get_mean()
    {

        if (deq.empty())
        {
            return 0.0;
        }

        return sum / deq.size();

    }

};




int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);


    int port;

    std::string ipaddress;

    std::string frame_id;

    bool inverted;


    int sick_motor_speed = 5;

    double sick_step_angle = 1.5;


    double active_sector_start_angle = 0;

    double active_sector_stop_angle = 300;


    double smoothing_factor;

    double error_threshold;



    auto node =
        std::make_shared<rclcpp::Node>("sickld");



    auto scan_pub =
        node->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS());




    node->declare_parameter(
        "port",
        DEFAULT_SICK_TCP_PORT);

    port =
        node->get_parameter("port").as_int();




    node->declare_parameter(
        "ipaddress",
        (std::string)DEFAULT_SICK_IP_ADDRESS);

    ipaddress =
        node->get_parameter("ipaddress").as_string();




    node->declare_parameter(
        "inverted",
        false);

    inverted =
        node->get_parameter("inverted").as_bool();




    node->declare_parameter(
        "frame_id",
        "laser");

    frame_id =
        node->get_parameter("frame_id").as_string();




    node->declare_parameter(
        "timer_smoothing_factor",
        0.97);

    smoothing_factor =
        node->get_parameter(
            "timer_smoothing_factor").as_double();




    node->declare_parameter(
        "timer_error_threshold",
        0.5);

    error_threshold =
        node->get_parameter(
            "timer_error_threshold").as_double();




    node->declare_parameter(
        "resolution",
        1.0);

    sick_step_angle =
        node->get_parameter(
            "resolution").as_double();




    node->declare_parameter(
        "start_angle",
        0.0);

    active_sector_start_angle =
        node->get_parameter(
            "start_angle").as_double();




    node->declare_parameter(
        "stop_angle",
        300.0);

    active_sector_stop_angle =
        node->get_parameter(
            "stop_angle").as_double();




    node->declare_parameter(
        "scan_rate",
        10);


    sick_motor_speed =
        node->get_parameter(
            "scan_rate").as_int();




    /*
     * Define buffers for return values
     */


    double range_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};


    unsigned int intensity_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};




    /*
     * Define buffers to hold sector specific data
     */


    unsigned int num_measurements = 0;


    unsigned int sector_start_timestamp = 0;


    unsigned int sector_stop_timestamp = 0;



    double sector_step_angle = 0;


    double sector_start_angle = 0;


    double sector_stop_angle = 0;



    /*
     * Instantiate the object
     */


    SickLD sick_ld(
        ipaddress.c_str(),
        port);




    try
    {

        /*
         * Initialize the device
         */


        sick_ld.Initialize();



        try
        {

            sick_ld.SetSickGlobalParamsAndScanAreas(
                (unsigned int)sick_motor_speed,
                sick_step_angle,
                &active_sector_start_angle,
                &active_sector_stop_angle,
                (unsigned int)1);

        }
        catch (...)
        {

            RCLCPP_ERROR(
                node->get_logger(),
                "Configuration error");

            return -1;

        }



        smoothtime smoothtimer;


        averager avg_fulldur;

        averager avg_scandur;



        smoothtimer.set_smoothing_factor(
            smoothing_factor);


        smoothtimer.set_error_threshold(
            error_threshold);


        rclcpp::Time last_start_scan_time;


        unsigned int last_sector_stop_timestamp = 0;


        double full_duration;

        
        while (rclcpp::ok())
        {

            /*
             * Grab the measurements (from all sectors)
             */

            sick_ld.GetSickMeasurements(
                range_values,
                intensity_values,
                &num_measurements,
                NULL,
                NULL,
                &sector_step_angle,
                &sector_start_angle,
                &sector_stop_angle,
                &sector_start_timestamp,
                &sector_stop_timestamp
            );



            rclcpp::Time end_scan_time =
                node->get_clock()->now();




            double scan_duration =
                (sector_stop_timestamp -
                 sector_start_timestamp) * 1e-3;



            avg_scandur.add_new(scan_duration);


            scan_duration =
                avg_scandur.get_mean();




            if (last_sector_stop_timestamp == 0)
            {

                full_duration =
                    1.0 /
                    ((double)sick_motor_speed);

            }
            else
            {

                full_duration =
                    (sector_stop_timestamp -
                     last_sector_stop_timestamp) *
                    1e-3;

            }




            avg_fulldur.add_new(full_duration);


            full_duration =
                avg_fulldur.get_mean();




            rclcpp::Time smoothed_end_scan_time =
                smoothtimer.smooth_timestamp(
                    end_scan_time,
                    rclcpp::Duration::from_seconds(
                        full_duration));




            rclcpp::Time start_scan_time =
                smoothed_end_scan_time -
                rclcpp::Duration::from_seconds(
                    scan_duration);




            publish_scan(
                scan_pub,
                range_values,
                num_measurements,
                intensity_values,
                num_measurements,
                start_scan_time,
                scan_duration,
                inverted,
                DEG2RAD((float)sector_start_angle),
                DEG2RAD((float)sector_stop_angle),
                frame_id
            );




            RCLCPP_DEBUG_STREAM(
                node->get_logger(),

                "Num meas: "
                << num_measurements

                << " smoothed start T: "
                << start_scan_time.nanoseconds()

                << " smoothed rate: "
                << 1.0 /
                   (start_scan_time -
                    last_start_scan_time).seconds()

                << " raw start T: "
                << sector_start_timestamp

                << " raw stop T: "
                << sector_stop_timestamp

                << " dur: "
                << full_duration

                << " step A: "
                << sector_step_angle

                << " start A: "
                << sector_start_angle

                << " stop A: "
                << sector_stop_angle
            );




            last_start_scan_time =
                start_scan_time;


            last_sector_stop_timestamp =
                sector_stop_timestamp;




            rclcpp::spin_some(node);

        }



        /*
         * Uninitialize the device
         */

        sick_ld.Uninitialize();

    }


    catch (...)
    {

        RCLCPP_ERROR(
            node->get_logger(),
            "Error");

        rclcpp::shutdown();

        return -1;

    }



    rclcpp::shutdown();


    return 0;

}
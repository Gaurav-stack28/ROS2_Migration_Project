// Author: Jonathan Sprinkle
// This (very simple) node reads a laser scan, and 
// publishes the distance to the nearest point
//
// TODO: ensure nearest few points are somewhat close
// TODO: what to return if no closest points?
// TODO: enable angle range we care about
// TODO: enable distance range we care about

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cstdio>
#include <cstdlib>


// this global var holds the distance
std_msgs::msg::Float64 angle;
std_msgs::msg::Float64 dist;
std_msgs::msg::Float64 dist_1;
std_msgs::msg::Float64 vel;
std_msgs::msg::Float64 vel_1;
std_msgs::msg::Float64 accel;
rclcpp::Time lastUpdate;
bool newMessage;
double angle_min;
double angle_max;

// This very simple callback looks through the data array, and then
// returns the value (not the index) of that distance
void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{

    // update what was 1 time ago
    dist_1 = dist;
    vel_1 = vel;
    lastUpdate = scan->header.stamp;   
//    ROS_INFO_STREAM(lastUpdate); 
    newMessage = true;
    dist.data = scan->range_max;
    angle.data = scan->angle_min;
    float angle_tmp=scan->angle_min;
    float inc = scan->angle_increment;
    for( std::vector<float>::const_iterator it=scan->ranges.begin();
        it!=scan->ranges.end(); it++, angle_tmp=angle_tmp+inc )
    {
        if( dist.data > *it && *it > scan->range_min 
            && angle_tmp > angle_min && angle_tmp < angle_max
            )
        {
            dist.data = *it;
            angle.data = angle_tmp;
        }
    }

    // HACK update from 0.02 to reflect actual time
    // if the distance is greater now than before, then
    // vel is greater (i.e., they are moving away from us)
    vel.data = (dist.data - dist_1.data)/0.02;
    // if vel is greater now than it was before, then
    // accel is greater (i.e., they are accel away from us)
    accel.data = (vel.data - vel_1.data)/0.02;
}

int main( int argc, char **argv )
{
    // initialize global vars
    dist.data = dist_1.data = 0;
    vel.data = vel_1.data = 0;
    accel.data = 0;

    std::string scan_topic;
    std::string dist_topic;
    std::string angle_topic;

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	rclcpp::init(argc, argv);

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
//	ros::NodeHandle n;
	// set up the handle for this node, in order to publish information
	// the node handle is retrieved from the parameters in the .launch file,
	// but we have to initialize the handle to have the name in that file.
	auto node = rclcpp::Node::make_shared("DistanceEstimator");

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
	node->declare_parameter<std::string>("dist_topic", "dist");
    dist_topic = node->get_parameter("dist_topic").as_string();
	node->declare_parameter<std::string>("angle_topic", "angle");
    angle_topic = node->get_parameter("angle_topic").as_string();
	node->declare_parameter<std::string>("scan_topic", "/scan");
    scan_topic = node->get_parameter("scan_topic").as_string();
//	n.param("angle_min", angle_min, -M_PI/32.0f);
//	n.param("angle_max", angle_max, M_PI/32.0f);
	node->declare_parameter<double>("angle_min", -0.1);
    angle_min = node->get_parameter("angle_min").as_double();
	node->declare_parameter<double>("angle_max", 0.1);
    angle_max = node->get_parameter("angle_max").as_double();

    RCLCPP_INFO_STREAM(node->get_logger(),"Node namespace is " << node->get_namespace());
    RCLCPP_INFO_STREAM(node->get_logger(),"Node name is " << node->get_name());


// TODO: make this not just a float value
  	auto dist_pub = node->create_publisher<std_msgs::msg::Float64>(dist_topic, 1);
  	auto angle_pub = node->create_publisher<std_msgs::msg::Float64>(angle_topic, 1);

  	// we also want to subscribe to the signaller
  	auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic,1,scanCallback);

    RCLCPP_INFO_STREAM(node->get_logger(),"Looking for scan in topic " << scan_topic);
    RCLCPP_INFO_STREAM(node->get_logger(),"Publishing estimated distance to target in topic "<< node->get_name() << "/" << dist_topic);
    RCLCPP_INFO_STREAM(node->get_logger(),"Publishing estimated angle to target in topic "<< node->get_name() << "/" << angle_topic);

  	// run at 50Hz?
  	rclcpp::Rate loop_rate(1000);
    lastUpdate = node->now();
    newMessage = false;

	while(rclcpp::ok())
	{
//		ROS_INFO( "Nearest object=%lf", dist );

        // TODO: publish only if the time stamp is newer from the sensor
        double eps = (double)1/(double)100; // the epsilon time that is 'equal' in diff
        if( newMessage )
        {
    		dist_pub->publish(dist);
            angle_pub->publish(angle);
            newMessage = false;
        }
//        vel_pub.publish(vel);
//        accel_pub.publish(accel);
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}


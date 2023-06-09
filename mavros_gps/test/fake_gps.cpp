#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

using namespace ros;

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char** argv)
{
    // init ros node
    init(argc, argv, "fake_gps");
    NodeHandle nh;

    // read parameters
    double lat,lon,alt;
    nh.param(this_node::getName() + "/lat", lat, 46.612918);
    nh.param(this_node::getName() + "/lon", lon, 14.265227);
    nh.param(this_node::getName() + "/alt", alt, 500.0);

    // publish rate in hertz
    Rate rate(20);

    // init publisher
    Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("mavros/global_position/global", 1);

    // initial position
    sensor_msgs::NavSatFix fix;
    fix.header.frame_id = "base_link";
    fix.status.status = 0;
    fix.status.service = 1;
    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude = alt;
    fix.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    fix.position_covariance_type = 2;

    // publish gps fix
    while (ok()) {
        fix.header.stamp = Time::now();
        gps_pub.publish(fix);
        rate.sleep();
    }

    return 0;
}

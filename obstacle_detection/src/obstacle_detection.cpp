#include <ros/ros.h>
#include "lib/obstacle_detection.h"

using namespace std;
using namespace ros;

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv)
{
    // init ros node
    init(argc, argv, "obstacle_detection");
    NodeHandle nh;

    // obstacle detection library
    obstacle_detection lib;

    // advertise services
    ServiceServer clear_of_obstacles_service = nh.advertiseService("obstacle_detection/clear_of_obstacles", &obstacle_detection::clear_of_obstacles, &lib);
    ServiceServer danger_service = nh.advertiseService("obstacle_detection/danger", &obstacle_detection::danger, &lib);
    ServiceServer get_occupied_sector_service = nh.advertiseService("obstacle_detection/get_occupied_sector", &obstacle_detection::get_occupied_sector, &lib);

    ROS_INFO("OBST_DETECT - Services are ready");
    spin();

    return 0;
}

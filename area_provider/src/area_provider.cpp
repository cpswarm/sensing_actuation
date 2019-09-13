#include "lib/area.h"

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
    init(argc, argv, "area_provider");
    NodeHandle nh;

    // area library
    area lib;

    // advertise services
    ServiceServer closest_bound_service = nh.advertiseService("area/closest_bound", &area::closest_bound, &lib);
    ServiceServer fix_to_pose_service = nh.advertiseService("area/get_area", &area::get_area, &lib);
    ServiceServer get_origin_service = nh.advertiseService("area/get_origin", &area::get_origin, &lib);
    ServiceServer out_of_bounds_service = nh.advertiseService("area/out_of_bounds", &area::out_of_bounds, &lib);

    // init publishers
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    Publisher gridmap_publisher = nh.advertise<nav_msgs::OccupancyGrid>("map", queue_size, true);

    // publish grid map
    gridmap_publisher.publish(lib.get_gridmap());

    ROS_DEBUG("AREA_PROV - Services are ready");
    spin();

    return 0;
}

#include "lib/ma.h"

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
    init(argc, argv, "mission_area");
    NodeHandle nh;

    // area library
    ma lib;

    // advertise services
    ServiceServer closest_bound_service = nh.advertiseService("area/closest_bound", &ma::closest_bound, &lib);
    ServiceServer get_area_service      = nh.advertiseService("area/get_area",      &ma::get_area, &lib);
    ServiceServer get_center_service    = nh.advertiseService("area/get_center",    &ma::get_center, &lib);
    ServiceServer get_map_service       = nh.advertiseService("area/get_map",       &ma::get_map, &lib);
    ServiceServer get_origin_service    = nh.advertiseService("area/get_origin",    &ma::get_origin, &lib);
    ServiceServer get_rotation_service  = nh.advertiseService("area/get_rotation",  &ma::get_rotation, &lib);
    ServiceServer out_of_bounds_service = nh.advertiseService("area/out_of_bounds", &ma::out_of_bounds, &lib);

    ROS_DEBUG("AREA_PROV - Mission area services are ready");
    spin();

    return 0;
}

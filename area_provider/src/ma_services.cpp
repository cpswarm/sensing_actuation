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

    // mission area library
    ma ma_lib;
    area &lib = ma_lib;

    // advertise services
    ServiceServer get_area_service      = nh.advertiseService("area/get_area",      &area::get_area, &lib);
    ServiceServer get_center_service    = nh.advertiseService("area/get_center",    &area::get_center, &lib);
    ServiceServer get_distance_service  = nh.advertiseService("area/get_distance",  &area::get_distance, &lib);
    ServiceServer get_map_service       = nh.advertiseService("area/get_map",       &area::get_map, &lib);
    ServiceServer get_origin_service    = nh.advertiseService("area/get_origin",    &area::get_origin, &lib);
    ServiceServer out_of_bounds_service = nh.advertiseService("area/out_of_bounds", &area::out_of_bounds, &lib);

    ROS_DEBUG("AREA_PROV - Mission area services are ready");
    spin();

    return 0;
}

#include <ros/ros.h>
#include "lib/asctec_gps_lib.h"

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
    init(argc, argv, "gps");
    NodeHandle nh;

    // gps library
    asctec_gps_lib lib;

    // advertise services
    ServiceServer fix_to_pose_service = nh.advertiseService("gps/fix_to_pose", &asctec_gps_lib::fix_to_pose, &lib);
    ServiceServer fix_to_gpsdata_service = nh.advertiseService("gps/fix_to_gpsdata", &asctec_gps_lib::fix_to_gpsdata, &lib);
    ServiceServer get_gps_origin_service = nh.advertiseService("gps/get_gps_origin", &asctec_gps_lib::get_gps_origin, &lib);
    ServiceServer gpsdata_to_fix_service = nh.advertiseService("gps/gpsdata_to_fix", &asctec_gps_lib::gpsdata_to_fix, &lib);
    ServiceServer gpsdata_to_pose_service = nh.advertiseService("gps/gpsdata_to_pose", &asctec_gps_lib::gpsdata_to_pose, &lib);
    ServiceServer ned_to_enu_service = nh.advertiseService("gps/ned_to_enu", &asctec_gps_lib::fix_to_gpsdata, &lib);
    ServiceServer pose_to_fix_service = nh.advertiseService("gps/pose_to_fix", &asctec_gps_lib::pose_to_fix, &lib);
    ServiceServer pose_to_gpsdata_service = nh.advertiseService("gps/pose_to_gpsdata", &asctec_gps_lib::pose_to_gpsdata, &lib);

    ROS_INFO("GPS - Services are ready");
    spin();

    return 0;
}

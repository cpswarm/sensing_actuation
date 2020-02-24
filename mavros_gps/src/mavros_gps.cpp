#include <ros/ros.h>
#include "lib/mavros_gps_lib.h"

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
    mavros_gps_lib lib;

    // advertise services
    ServiceServer fix_to_pose_service = nh.advertiseService("gps/fix_to_pose", &mavros_gps_lib::fix_to_pose, &lib);
    ServiceServer fix_to_target_service = nh.advertiseService("gps/fix_to_target", &mavros_gps_lib::fix_to_target, &lib);
    ServiceServer get_gps_origin_service = nh.advertiseService("gps/get_gps_origin", &mavros_gps_lib::get_gps_origin, &lib);
    ServiceServer ned_to_enu_service = nh.advertiseService("gps/ned_to_enu", &mavros_gps_lib::fix_to_target, &lib);
    ServiceServer pose_to_fix_service = nh.advertiseService("gps/pose_to_fix", &mavros_gps_lib::pose_to_fix, &lib);
    ServiceServer pose_to_target_service = nh.advertiseService("gps/pose_to_target", &mavros_gps_lib::pose_to_target, &lib);
    ServiceServer target_to_fix_service = nh.advertiseService("gps/target_to_fix", &mavros_gps_lib::target_to_fix, &lib);

    ROS_INFO("GPS - Services are ready");
    spin();

    return 0;
}

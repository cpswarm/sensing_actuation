#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
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
    ServiceServer get_area_service      = nh.advertiseService("area/get_area",      &area::get_area, &lib);
    ServiceServer get_center_service    = nh.advertiseService("area/get_center",    &area::get_center, &lib);
    ServiceServer get_map_service       = nh.advertiseService("area/get_map",       &area::get_map, &lib);
    ServiceServer get_origin_service    = nh.advertiseService("area/get_origin",    &area::get_origin, &lib);
    ServiceServer out_of_bounds_service = nh.advertiseService("area/out_of_bounds", &area::out_of_bounds, &lib);

    // broadcast static tf
    static tf2_ros::StaticTransformBroadcaster tf_bc;
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "local_origin_ned";
    tf.child_frame_id = "map";
    tf.transform.rotation.w = 1;
    tf_bc.sendTransform(tf);

    ROS_DEBUG("AREA_PROV - Services are ready");
    spin();

    return 0;
}

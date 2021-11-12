#include "lib/rois.h"

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
    init(argc, argv, "rois");
    NodeHandle nh;

    // roi library
    rois rois_lib;

    // advertise services general for all rois
    ServiceServer get_all  = nh.advertiseService("rois/get_all",  &rois::get_all, &rois_lib);
    ServiceServer get_closest_service  = nh.advertiseService("rois/get_closest",  &rois::get_closest, &rois_lib);
    ServiceServer reload_service  = nh.advertiseService("rois/reload",  &rois::reload, &rois_lib);

    // advertise services per roi
    for (auto roi : rois_lib.get_rois()) {
        area &lib = roi.second;
        ServiceServer get_distance_service  = nh.advertiseService("rois/roi_" + to_string(roi.first) + "/get_distance",  &area::get_distance, &lib);
    }

    ROS_DEBUG("ROI_PROV - ROI services are ready");
    spin();

    return 0;
}

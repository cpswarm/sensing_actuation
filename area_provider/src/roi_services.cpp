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

    // advertise services
    // TODO: what services are actually required?

    ROS_DEBUG("ROI_PROV - ROI services are ready");
    spin();

    return 0;
}

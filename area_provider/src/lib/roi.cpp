#include "lib/roi.h"

roi::roi (vector<geometry_msgs::Point> raw_coords)
{
    coords = raw_coords;

    if (global)
        global_to_local();

    set_origin();
}

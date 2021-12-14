#include "lib/roi.h"

roi::roi (vector<double> x, vector<double> y)
{
    // equal number of x and y coordinates required
    if (x.size() != y.size()) {
        ROS_ERROR("Cannot create ROI, number of x and y coordinates do not match (%lu != %lu)", x.size(), y.size());
    }

    // not enough coordinates
    else if (x.size() < 3) {
        ROS_ERROR("Cannot create ROI, not enough coordinates: %lu", x.size());
    }

    else {
        // set coordinates
        for (int i=0; i<x.size(); ++i) {
            geometry_msgs::Point c;
            c.x = x[i];
            c.y = y[i];
            coords.push_back(c);
        }

        // convert global coordinates
        if (global)
            global_to_local();

        // set origin
        set_origin();
    }

    // gridmap still needs to be created
    map_exists = false;
}

bool roi::operator== (const roi other)
{
    // rois are not equal, they have different number of coordinates
    if (coords.size() != other.coords.size())
        return false;

    // the vector of equal coordinates
    unsigned int equal = 0;

    // count coordinates that are equal between rois
    for (auto c : coords) {
        for (auto c_other : other.coords) {
            if (c == c_other) {
                ++equal;
                break;
            }
        }
    }

    // rois are equal if all coordinates are equal
    return coords.size() == equal;
}

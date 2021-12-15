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
            coords.emplace(x[i], y[i]);
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
    return coords == other.coords;
}

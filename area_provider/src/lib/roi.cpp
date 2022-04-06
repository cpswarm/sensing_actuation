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
            coords[0].emplace(x[i], y[i]);
        }

        // convert global coordinates
        if (global)
            global_to_local();

        // set origin
        set_origin();
    }

    sort_coords();
}

bool roi::operator== (const roi other)
{
    return coords[0] == other.coords.at(0);
}

bool roi::operator< (const roi other) const
{
    return coords.at(0) < other.coords.at(0);
}

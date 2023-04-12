#include "lib/roi.h"

roi::roi (vector<double> x, vector<double> y, vector<double> z)
{
    // equal number of x and y coordinates required
    if (x.size() != y.size() || x.size() != z.size() || y.size() != z.size()) {
        ROS_ERROR("Cannot create ROI, number of x and y coordinates do not match (%lu != %lu != %lu)", x.size(), y.size(), z.size());
    }

    // not enough coordinates
    else if (x.size() < 3) {
        ROS_ERROR("Cannot create ROI, not enough coordinates: %lu", x.size());
    }

    else {
        // initialize state
        state = ROI_TODO;

        // set coordinates
        for (int i=0; i<x.size(); ++i) {
            coords[0].emplace(x[i], y[i], z[i]);
        }

        // convert global coordinates
        if (global)
            global_to_local();

        // set origin
        set_origin();
    }

    sort_coords();
}

tuple<vector<double>, vector<double>, vector<double>> roi::get_global ()
{
    vector<double> lon, lat, alt;

    // get global coordinates
    if (global) {
        for (auto it=coords_global.begin(); it!=coords_global.end(); ++it) {
            lon.push_back(get<0>(*it));
            lat.push_back(get<1>(*it));
            alt.push_back(get<2>(*it));
        }
    }

    // use local coordinates (not rotated)
    else {
        for (auto it=coords[0].begin(); it!=coords[0].end(); ++it) {
            lon.push_back(get<0>(*it));
            lat.push_back(get<1>(*it));
            alt.push_back(get<2>(*it));
        }
    }

    return make_tuple(lon, lat, alt);
}

bool roi::operator== (const roi other)
{
    return coords[0] == other.coords.at(0);
}

bool roi::operator< (const roi other) const
{
    return coords.at(0) < other.coords.at(0);
}

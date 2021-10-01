#ifndef ROIS_H
#define ROIS_H

#include <nlohmann/json.hpp>
#include "lib/roi.h"

using json = nlohmann::json;

/**
 * @brief A class that manages several regions of interest (ROIs).
 */
class rois
{
public:
    /**
     * @brief Constructor that initializes the ROIs.
     */
    rois ();

private:
    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief All ROIs with IDs.
     */
    map<int,roi> regions;
};

#endif // ROIS_H

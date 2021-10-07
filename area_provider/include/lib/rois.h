#ifndef ROIS_H
#define ROIS_H

#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <ros/package.h>
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

    /**
     * @brief Get all ROIs managed by this class.
     * @return Each ROI together with its unique ID.
     */
    map<int,roi> get_rois ();

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

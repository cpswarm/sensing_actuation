#ifndef ROIS_H
#define ROIS_H

#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <ros/package.h>
#include <cpswarm_msgs/PointArrayEvent.h>
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
     * @brief Return the closest ROI.
     * @param req The coordinates of the point to check.
     * @param res For the closest ROI, the coordinates of the closest area boundary line segment, the coordinates of the closest point on the boundary, and the distance.
     * @return Whether request succeeded.
     */
    bool get_closest (cpswarm_msgs::GetDist::Request &req, cpswarm_msgs::GetDist::Response &res);

    /**
     * @brief Get all ROIs managed by this class.
     * @return Each ROI together with its unique ID.
     */
    map<int,roi> get_rois ();

private:
    /**
     * @brief Add an ROI.
     * @param coords A vector of the ROI coordinates.
     */
    void add_roi (vector<geometry_msgs::Point> coords);

    /**
     * @brief Read ROI coordinates from files at the folder specified with the `roi_dir` param.
     */
    void from_file ();

    /**
     * @brief Receive ROI coordinates from an event message.
     * @param event A pointer to the event holding a vector of coordinates (points).
     */
    void roi_callback (const cpswarm_msgs::PointArrayEvent::ConstPtr& event);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber object to receive ROI coordinates from event messages.
     */
    Subscriber roi_subscriber;

    /**
     * @brief All ROIs with IDs.
     */
    map<int,roi> regions;
};

#endif // ROIS_H

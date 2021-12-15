#ifndef ROIS_H
#define ROIS_H

#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <ros/package.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_srvs/SetBool.h>
#include <cpswarm_msgs/GetMultiPoints.h>
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
     * @brief Return all ROIs
     * @param req Empty request.
     * @param res The coordinates of the ROI boundaries.
     * @return Whether the request succeeded.
     */
    bool get_all (cpswarm_msgs::GetMultiPoints::Request &req, cpswarm_msgs::GetMultiPoints::Response &res);

    /**
     * @brief Return the closest ROI.
     * @param req The coordinates of the point to check.
     * @param res For the closest ROI, the coordinates of the closest area boundary line segment, the coordinates of the closest point on the boundary, and the distance.
     * @return Whether request succeeded.
     */
    bool get_closest (cpswarm_msgs::GetDist::Request& req, cpswarm_msgs::GetDist::Response& res);

    /**
     * @brief Return the map that correspond to a given ROI.
     * @param req The coordinates of the ROI together with the desired resolution of the map and whether its bottom edge should be aligned horizontally and the origin should be an integer.
     * @param res The grid map of the environment together with the angle it has been rotated by and the offset it has been translated by.
     * @return Whether the request succeeded.
     */
    bool get_map (cpswarm_msgs::GetMap::Request &req, cpswarm_msgs::GetMap::Response &res);

    /**
     * @brief Get all ROIs managed by this class.
     * @return Each ROI together with its unique ID.
     */
    map<int,roi> get_rois ();

    /**
     * @brief Reload ROIs from files.
     * @param req The trigger to reload. If true, current ROIs are removed before reloading.
     * @param res
     * @return Whether request succeeded.
     */
    bool reload (std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

private:
    /**
     * @brief Add an ROI.
     * @param x A vector of the ROI x-coordinates.
     * @param y A vector of the ROI y-coordinates.
     */
    void add_roi (vector<double> x, vector<double> y);

    /**
     * @brief Check whether a given ROI is already known.
     *
     * @param roi The ROI to check.
     * @return True, if the coordinates of the given ROI are identical with an existing ROI. False otherwise.
     */
    bool exists (roi roi);

    /**
     * @brief Read ROI coordinates from files at the folder specified with the `roi_dir` param.
     */
    void from_file ();

    /**
     * @brief Receive ROI coordinates from an event message.
     * @param event A pointer to the event holding a vector of coordinates.
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
     * @brief Event publisher for ROI coordinates.
     */
    Publisher roi_publisher;

    /**
     * @brief All ROIs with IDs.
     */
    map<int,roi> regions;

    /**
     * @brief Whether to allow duplicate ROIs, i.e., ROIs with identical coordinates.
     */
    bool duplicates;

    /**
     * @brief Whether to publish any newly imported ROI as event.
     */
    bool publish;
};

#endif // ROIS_H

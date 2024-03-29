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
#include <cpswarm_msgs/PointArrayStateEvent.h>
#include <cpswarm_msgs/SetRoiState.h>
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
     * @brief Return all ROIs.
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
     * @brief Return the distance to a given ROI.
     * @param req The coordinates of the point to check.
     * @param res The coordinates of the closest area boundary line segment, the coordinates of the closest point on the boundary, and the distance.
     * @return Whether request succeeded.
     */
    bool get_distance (cpswarm_msgs::GetDist::Request& req, cpswarm_msgs::GetDist::Response& res);

    /**
     * @brief Return the map that correspond to a given ROI.
     * @param req The coordinates of the ROI together with the desired resolution of the map and whether its bottom edge should be aligned horizontally and the origin should be an integer.
     * @param res The grid map of the environment together with the angle it has been rotated by and the offset it has been translated by.
     * @return Whether the request succeeded.
     */
    bool get_map (cpswarm_msgs::GetMap::Request &req, cpswarm_msgs::GetMap::Response &res);

    /**
     * @brief Return all ROIs in the state TODO.
     * @param req Empty request.
     * @param res The coordinates of the ROI boundaries.
     * @return Whether the request succeeded.
     */
    bool get_todo (cpswarm_msgs::GetMultiPoints::Request &req, cpswarm_msgs::GetMultiPoints::Response &res);

    /**
     * @brief Reload ROIs from files.
     * @param req The trigger to reload. If true, current ROIs are removed before reloading.
     * @param res
     * @return Whether request succeeded.
     */
    bool reload (std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    /**
     * @brief Set the state of an ROI.
     * @param req The ROI and the state.
     * @param res Whether the request succeeded.
     * @return Whether the request succeeded.
     */
    bool set_state (cpswarm_msgs::SetRoiState::Request &req, cpswarm_msgs::SetRoiState::Response &res);

private:
    /**
     * @brief Add an ROI.
     * @param x A vector of the ROI x-coordinates.
     * @param y A vector of the ROI y-coordinates.
     * @param z A vector of the ROI z-coordinates.
     */
    void add_roi (vector<double> x, vector<double> y, vector<double> z);

    /**
     * @brief Check whether a given ROI is already known.
     *
     * @param roi The ROI to check.
     * @return True, if the coordinates of the given ROI are identical with an existing ROI. False otherwise.
     */
    bool exists (roi roi);

    /**
     * @brief Create a flat 1D vector of coordinates.
     *
     * @param vector_2d The original 2D vector of coordinates.
     * @param vector_flat The flattened 1D vector of coordinates.
     * @param layout Information about how the coordinates are stored in the flat vector.
     */
    void flatten_vector (vector<vector<geometry_msgs::Point>> vector_2d, vector<geometry_msgs::Point>& vector_flat, std_msgs::MultiArrayLayout& layout);

    /**
     * @brief Read ROI coordinates from files at the folder specified with the `roi_dir` param.
     */
    void from_file ();

    /**
     * @brief Set the state of a ROI.
     * @param roi The coordinates of the ROI.
     * @param state The new state for the ROI.
     * @return True, if the coordinates correspond to a known ROI and the state is valid. False otherwise.
     */
    bool set_state (set<tuple<double,double,double>> roi, roi_state_t state);

    /**
     * @brief Receive ROI coordinates from an event message.
     * @param event A pointer to the event holding a vector of coordinates.
     */
    void roi_callback (const cpswarm_msgs::PointArrayEvent::ConstPtr& event);

    /**
     * @brief Receive a state update for a ROI.
     * @param event A pointer to the event holding the ROI coordinates and the desired state.
     */
    void state_callback (const cpswarm_msgs::PointArrayStateEvent::ConstPtr& event);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Service client to convert global (GPS) coordinates to the local coordinate system.
     */
    ServiceClient fix_to_pose_client;

    /**
     * @brief Subscriber object to receive ROI coordinates from event messages.
     */
    Subscriber roi_subscriber;

    /**
     * @brief Subscriber object to receive ROI coordinates from assignment event messages.
     */
    Subscriber assignment_subscriber;

    /**
     * @brief Subscriber object to receive ROI state updates from event messages.
     */
    Subscriber state_subscriber;

    /**
     * @brief Event publisher for ROI coordinates.
     */
    Publisher roi_publisher;

    /**
     * @brief Publisher of ROI maps for introspection.
     */
    vector<Publisher> map_publisher;

    /**
     * @brief All ROIs with IDs.
     */
    set<roi> regions;

    /**
     * @brief Whether to allow duplicate ROIs, i.e., ROIs with identical coordinates.
     */
    bool duplicates;

    /**
     * @brief Whether to publish any newly imported ROI as event.
     */
    bool publish;

    /**
     * @brief Whether to publish the map of any newly imported ROI.
     */
    bool visualize;

    /**
     * @brief Whether global (GPS) or local coordinates are used as source.
     */
    bool global;
};

#endif // ROIS_H

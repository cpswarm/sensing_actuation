#ifndef AREA_H
#define AREA_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "angle.h"
#include "cpswarm_msgs/fix_to_pose.h"
#include "cpswarm_msgs/get_gps_origin.h"
#include "cpswarm_msgs/closest_bound.h"
#include "cpswarm_msgs/get_area.h"
#include "cpswarm_msgs/get_origin.h"
#include "cpswarm_msgs/out_of_bounds.h"

using namespace std;
using namespace ros;

/**
 * @brief A class that holds the mission area and offers related services.
 */
class area
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    area ();

    /**
     * @brief Find the closest area bound to a given point.
     * @param req The point for which to find the closest bound.
     * @param res The coordinates of the bound and the perpendicular distance to the given point.
     * @return Whether request succeeded.
     */
    bool closest_bound (cpswarm_msgs::closest_bound::Request &req, cpswarm_msgs::closest_bound::Response &res);

    /**
     * @brief Return the bounding area in which the CPS is allowed to move.
     * @param req Empty request.
     * @param res A vector of points that defines the bounding polygon coordinates.
     * @return Whether request succeeded.
     */
    bool get_area (cpswarm_msgs::get_area::Request &req, cpswarm_msgs::get_area::Response &res);

    /**
     * @brief Return the origin of the coordinate system.
     * @param req Empty request.
     * @param res A point that defines the origin coordinates.
     * @return Whether request succeeded.
     */
    bool get_origin (cpswarm_msgs::get_origin::Request &req, cpswarm_msgs::get_origin::Response &res);

    /**
     * @brief Determine whether a position is within the area. Uses the approach described in solution 2 at http://paulbourke.net/geometry/polygonmesh/#insidepoly.
     * @param req The position to check.
     * @param res True, if the position is outside of the area, false otherwise.
     * @return Whether the request succeeded.
     */
    bool out_of_bounds (cpswarm_msgs::out_of_bounds::Request &req, cpswarm_msgs::out_of_bounds::Response &res);

private:
    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The coordinates of the area in which the CPS can move.
     */
    vector<geometry_msgs::Point> coords;

    /**
     * @brief The origin of the coordinate system.
     */
    geometry_msgs::Point origin;
};

#endif // AREA_H

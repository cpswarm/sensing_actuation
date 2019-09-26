#ifndef AREA_H
#define AREA_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include "cpswarm_msgs/FixToPose.h"
#include "cpswarm_msgs/GetGpsOrigin.h"
#include "cpswarm_msgs/ClosestBound.h"
#include "cpswarm_msgs/GetArea.h"
#include "cpswarm_msgs/GetOrigin.h"
#include "cpswarm_msgs/OutOfBounds.h"

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
    bool closest_bound (cpswarm_msgs::ClosestBound::Request &req, cpswarm_msgs::ClosestBound::Response &res);

    /**
     * @brief Return the bounding area in which the CPS is allowed to move.
     * @param req Empty request.
     * @param res A vector of points that defines the bounding polygon coordinates.
     * @return Whether request succeeded.
     */
    bool get_area (cpswarm_msgs::GetArea::Request &req, cpswarm_msgs::GetArea::Response &res);

    /**
     * @brief Generate a grid map the the given area coordinates.
     * @return An empty grid map that represents the area.
     */
    nav_msgs::OccupancyGrid get_gridmap ();

    /**
     * @brief Return the origin of the coordinate system.
     * @param req Empty request.
     * @param res A point that defines the origin coordinates.
     * @return Whether request succeeded.
     */
    bool get_origin (cpswarm_msgs::GetOrigin::Request &req, cpswarm_msgs::GetOrigin::Response &res);

    /**
     * @brief Determine whether a position is within the area. Uses the approach described in solution 2 at http://paulbourke.net/geometry/polygonmesh/#insidepoly.
     * @param req The position to check.
     * @param res True, if the position is outside of the area, false otherwise.
     * @return Whether the request succeeded.
     */
    bool out_of_bounds (cpswarm_msgs::OutOfBounds::Request &req, cpswarm_msgs::OutOfBounds::Response &res);

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

    /**
     * @brief Resolution of the grid map representing the area in meter / cell.
     */
    double resolution;
};

#endif // AREA_H

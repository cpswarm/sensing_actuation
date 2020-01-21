#ifndef AREA_H
#define AREA_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include "cpswarm_msgs/FixToPose.h"
#include "cpswarm_msgs/GetGpsFix.h"
#include "cpswarm_msgs/ClosestBound.h"
#include "cpswarm_msgs/GetPoints.h"
#include "cpswarm_msgs/GetPoint.h"
#include "cpswarm_msgs/GetDouble.h"
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
    bool get_area (cpswarm_msgs::GetPoints::Request &req, cpswarm_msgs::GetPoints::Response &res);

    /**
     * @brief Return the center of the area in which the CPS is allowed to move.
     * @param req Empty request.
     * @param res The center of the area.
     * @return Whether request succeeded.
     */
    bool get_center (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res);

    /**
     * @brief Generate a grid map the the given area coordinates.
     * @return An empty grid map that represents the area.
     */
    nav_msgs::OccupancyGrid get_gridmap ();

    /**
     * @brief Return the map that represents the environment in which the CPS is allowed to move.
     * @param req Empty request.
     * @param res The grid map of the environment.
     * @return Whether the request succeeded.
     */
    bool get_map (nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

    /**
     * @brief Return the origin of the coordinate system.
     * @param req Empty request.
     * @param res A point that defines the origin coordinates.
     * @return Whether request succeeded.
     */
    bool get_origin (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res);

    /**
     * @brief Determine whether a position is within the area using the winding number algorithm.
     * @param req The position to check.
     * @param res True, if the position is outside of the area, false otherwise.
     * @return Whether the request succeeded.
     */
    bool out_of_bounds (cpswarm_msgs::OutOfBounds::Request &req, cpswarm_msgs::OutOfBounds::Response &res);

    /**
     * @brief Get the rotation of the area given by coordinates. Assuming a quadrilateral.
     * @param req Empty request.
     * @param res The angle that the bottom edge of the area is rotated with respect to the x-axis.
     * @return Whether the request succeeded.
     */
    bool get_rotation (cpswarm_msgs::GetDouble::Request &req, cpswarm_msgs::GetDouble::Response &res);

private:
    /**
     * @brief Initialize area coordinates, either from existing map or from given coordinates.
     */
    void init_area ();

    /**
     * @brief Test whether a point is left of an infinite line.
     * @param p0 First point of the line.
     * @param p1 Second point of the line.
     * @param p2 Point to test.
     * @return True, if the point is left of the line, false otherwise.
     */
    bool is_left (geometry_msgs::Point p0, geometry_msgs::Point p1, geometry_msgs::Point p2);

    /**
     * @brief Test whether a point is right of an infinite line.
     * @param p0 First point of the line.
     * @param p1 Second point of the line.
     * @param p2 Point to test.
     * @return True, if the point is right of the line, false otherwise.
     */
    bool is_right (geometry_msgs::Point p0, geometry_msgs::Point p1, geometry_msgs::Point p2);

    /**
     * @brief Callback function to receive map updates.
     */
    void map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber to receive an existing map.
     */
    Subscriber map_subscriber;

    /**
     * @brief Publisher to publish own map.
     */
    Publisher map_publisher;

    /**
    /**@brief The grid map representing the area.
     */
    nav_msgs::OccupancyGrid map;

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

    /**
     * @brief Set coordinates from existing map provided by another node.
     */
    bool map_exists;
};

#endif // AREA_H

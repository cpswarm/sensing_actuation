#ifndef AREA_H
#define AREA_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include "cpswarm_msgs/FixToPose.h"
#include "cpswarm_msgs/GetGpsFix.h"
#include "cpswarm_msgs/GetPoints.h"
#include "cpswarm_msgs/GetPoint.h"
#include "cpswarm_msgs/GetDouble.h"
#include "cpswarm_msgs/OutOfBounds.h"
#include "cpswarm_msgs/GetDist.h"

using namespace std;
using namespace ros;

/**
 * @brief A class that holds the an area and offers related services.
 */
class area
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    area ();

    /**
     * @brief Return the area coordinates.
     * @param req Empty request.
     * @param res A vector of points that defines the bounding polygon coordinates.
     * @return Whether request succeeded.
     */
    bool get_area (cpswarm_msgs::GetPoints::Request &req, cpswarm_msgs::GetPoints::Response &res);

    /**
     * @brief Return the center of the area.
     * @param req Empty request.
     * @param res The coordinates of the center of the area.
     * @return Whether request succeeded.
     */
    bool get_center (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res);

    /**
     * @brief Return the distance of a given point to the area boundary (outside or inside the area). Based on https://stackoverflow.com/questions/10983872/distance-from-a-point-to-a-polygon.
     * @param req The coordinates of the point to check. If it is empty (0,0), the area origin is used.
     * @param res The coordinates of the closest area boundary line segment, the coordinates of the closest point on the boundary, and the distance.
     * @return Whether request succeeded.
     */
    bool get_distance (cpswarm_msgs::GetDist::Request &req, cpswarm_msgs::GetDist::Response &res);

    /**
     * @brief Generate a grid map from the given area coordinates.
     * @return A grid map that represents the area.
     */
    nav_msgs::OccupancyGrid get_gridmap ();

    /**
     * @brief Return the map that represents the area.
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
     * @brief Get the rotation of the area given by coordinates. Assuming a quadrilateral.
     * @param req Empty request.
     * @param res The angle that the bottom edge of the area is rotated with respect to the x-axis.
     * @return Whether the request succeeded.
     */
    bool get_rotation (cpswarm_msgs::GetDouble::Request &req, cpswarm_msgs::GetDouble::Response &res);

    /**
     * @brief Determine whether a position is within the area using the winding number algorithm.
     * @param req The position to check.
     * @param res True, if the position is outside of the area, false otherwise.
     * @return Whether the request succeeded.
     */
    bool out_of_bounds (cpswarm_msgs::OutOfBounds::Request &req, cpswarm_msgs::OutOfBounds::Response &res);

protected:
    /**
     * @brief Convert the coordinates of the area from global (GPS) coordinates to the local coordinate system.
     */
    void global_to_local ();

    /**
     * @brief Set the origin of the local coordinate system from the GPS starting position or given parameter.
     */
    void set_origin ();

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Publisher to publish grid map.
     */
    Publisher map_publisher;

    /**
     * @brief Map has been created.
     */
    bool map_exists;

    /**
     * @brief Whether a grid map will be generated by this node.
     */
    bool create_map;

    /**
    /**@brief The grid map representing the area.
     */
    nav_msgs::OccupancyGrid map;

    /**
     * @brief The coordinates of the area.
     */
    vector<geometry_msgs::Point> coords;

    /**
     * @brief Whether global (GPS) or local coordinates are used as source.
     */
    bool global;

private:
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
     * @brief Service client to convert global (GPS) coordinates to the local coordinate system.
     */
    ServiceClient fix_to_pose_client;

    /**
     * @brief The origin of the coordinate system.
     */
    geometry_msgs::Point origin;

    /**
     * @brief Number of grid cells in the map above which a performance warning is issued.
     */
    int cell_warn;

    /**
     * @brief Resolution of the grid map representing the area in meter / cell.
     */
    double resolution;
};

#endif // AREA_H

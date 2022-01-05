#ifndef AREA_H
#define AREA_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cpswarm_msgs/GetMap.h>
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
 * @brief An enumeration for grid cell values.
 */
typedef enum {
    CELL_FREE = 0,      // cell value for free cells
    CELL_OCCUPIED = 100 // cell value for occupied cells
} cell_value_t;

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
     * @brief Return the centroid of the area (https://en.wikipedia.org/wiki/Centroid#Of_a_polygon).
     * @param req Empty request.
     * @param res The coordinates of the centroid of the area.
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
     * @brief Return the map that represents the area.
     * @param req The desired resolution of the map and whether its bottom edge should be aligned horizontally and the origin should be an integer.
     * @param res The grid map of the environment together with the angle it has been rotated by and the offset it has been translated by.
     * @return Whether the request succeeded.
     */
    bool get_map (cpswarm_msgs::GetMap::Request &req, cpswarm_msgs::GetMap::Response &res);

    /**
     * @brief Return the origin of the coordinate system.
     * @param req Empty request.
     * @param res A point that defines the origin coordinates.
     * @return Whether request succeeded.
     */
    bool get_origin (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res);

    /**
     * @brief Determine whether a position is within the area.
     * @param req The position to check.
     * @param res True, if the position is outside of the area, false otherwise.
     * @return Whether the request succeeded.
     */
    bool out_of_bounds (cpswarm_msgs::OutOfBounds::Request &req, cpswarm_msgs::OutOfBounds::Response &res);

    /**
     * @brief Return a string that uniquely represents the area.
     * @return A concatenation of the area's coordinates.
     */
    string to_string ();

protected:
    /**
     * @brief Generate a grid map from the given area coordinates.
     * @param rotation Whether the map bottom edge should be aligned horizontally, default false.
     * @param resolution Resolution of the map in meter / cell, default 0, i.e., using value provided by parameter server or subscribed map.
     * @return A grid map that represents the area.
     */
    nav_msgs::OccupancyGrid get_gridmap (bool rotated=false, double resolution=0);

    /**
     * @brief Convert the coordinates of the area from global (GPS) coordinates to the local coordinate system.
     */
    void global_to_local ();

    /**
     * @brief Set the origin of the local coordinate system from the GPS starting position or given parameter.
     */
    void set_origin ();

    /**
     * @brief Convert a set of double pairs to a vector of geometry_msgs/Point.
     * @param set The set to convert.
     * @return The converted vector.
     */
    vector<geometry_msgs::Point> set2vector (set<pair<double,double>> set);

    /**
     * @brief Sort the coordinates of the area counter-clockwise about the center by their angle [-π,π].
     */
    void sort_coords ();

    /**
     * @brief Convert a vector of geometry_msgs/Point to a set of double pairs.
     * @param vector The vector to convert.
     * @return The converted set.
     */
    set<pair<double,double>> vector2set (vector<geometry_msgs::Point> vector);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The grid map representing the area at different rotations and resolutions.
     *
     * Mapping from rotation angle to mapping from resolution to grid map.
     */
    map<double,map<double,nav_msgs::OccupancyGrid>> gridmaps;

    /**
     * @brief The coordinates of the area at different rotations.
     */
    map<double,set<pair<double,double>>> coords;

    /**
     * @brief Whether global (GPS) or local coordinates are used as source.
     */
    bool global;

    /**
     * @brief Resolution of the grid map representing the area in meter / cell.
     */
    double resolution;

private:
    /**
     * @brief Test whether a point is left of an infinite line.
     * @param p0 First point of the line.
     * @param p1 Second point of the line.
     * @param p2 Point to test.
     * @return True, if the point is left of the line, false otherwise.
     */
    bool is_left (pair<double,double> p0, pair<double,double> p1, pair<double,double> p2);

    /**
     * @brief Test whether a point is right of an infinite line.
     * @param p0 First point of the line.
     * @param p1 Second point of the line.
     * @param p2 Point to test.
     * @return True, if the point is right of the line, false otherwise.
     */
    bool is_right (pair<double,double> p0, pair<double,double> p1, pair<double,double> p2);

    /**
     * @brief Test whether a point is on the area boundary.
     * @param pos Point to test
     * @param angle The angle to rotate the area by, radian, counter-clockwise, default 0.
     * @return True, if the distance from p0 to p2 and from p2 to p1 is almost the same as from p0 to p1, false otherwise.
     */
    bool on_bound (pair<double,double> pos, double angle);

    /**
     * @brief Determine whether a position is within the area using the winding number algorithm.
     * @param pos The position to check.
     * @param angle The angle to rotate the area by, radian, counter-clockwise, default 0.
     * @return True, if the position is outside of the area boundary, false otherwise.
     */
    bool out_of_bounds (pair<double,double> pos, double angle=0);

    /**
     * @brief Convert a pair of doubles to a geometry_msgs/Point.
     * @param pair The pair to convert.
     * @return A geometry_msgs/Point where x is the first element of the pair and y the second.
     */
    geometry_msgs::Point pair2point (pair<double,double> pair);

    /**
     * @brief Convert a geometry_msgs/Point to a pair of doubles.
     * @param point The point to convert.
     * @return A pair of doubles where the first element is x of the point and the second element is y.
     */
    pair<double,double> point2pair (geometry_msgs::Point point);

    /**
     * @brief Rotate the area coordinates to make the bottom edge horizontal. Stores the resulting coordinates in class variable.
     * @return The angle that the area has to be rotated, radian, counter-clockwise.
     */
    double rotate ();

    /**
     * @brief Shift a map to be aligned with the grid, i.e., the origin should be an even number.
     * @param map A reference to the occupancy grid map to shift.
     * @return A vector that specifies the amount that the map has been shifted in x and y direction.
     */
    geometry_msgs::Vector3 translate (nav_msgs::OccupancyGrid& map);

    /**
     * @brief Service client to convert global (GPS) coordinates to the local coordinate system.
     */
    ServiceClient fix_to_pose_client;

    /**
     * @brief The origin of the coordinate system.
     */
    geometry_msgs::Point origin;

    /**
     * @brief The coordinates of the area at different rotations, sorted counter-clockwise about the center, starting from negative x-axis [-π,π]. They are mapped by the respective angle.
     *
     * Mapping from rotation angle to mapping from angle around origin to coordinate.
     */
    map<double,map<double, pair<double,double>>> coords_sorted;

    /**
     * @brief Number of grid cells in the map above which a performance warning is issued.
     */
    int cell_warn;

    /**
     * @brief The angle that the map has to rotated by to make the bottom edge horizontal, radian, counter-clockwise.
     */
    double rotation;
};

#endif // AREA_H

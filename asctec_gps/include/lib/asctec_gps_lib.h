#ifndef ASCTEC_GPS_LIB_H
#define ASCTEC_GPS_LIB_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <asctec_msgs/GPSData.h>
#include "cpswarm_msgs/FixToPose.h"
#include "cpswarm_msgs/GetGpsFix.h"
#include "cpswarm_msgs/NedToEnu.h"
#include "cpswarm_msgs/PoseToFix.h"
#include "asctec_gps/FixToGpsdata.h"
#include "asctec_gps/GpsdataToFix.h"
#include "asctec_gps/GpsdataToPose.h"
#include "asctec_gps/PoseToGpsdata.h"

using namespace std;
using namespace ros;

/**
 * @brief A class that offers common operations required when working with GPS coordinates.
 */
class asctec_gps_lib
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    asctec_gps_lib ();

    /**
     * @brief Convert a message from sensor_msgs::NavSatFix to geometry_msgs::PoseStamped.
     * @param req The global message to convert.
     * @param res The converted local message without orientation
     * @return Whether the conversion succeeded.
     */
    bool fix_to_pose (cpswarm_msgs::FixToPose::Request &req, cpswarm_msgs::FixToPose::Response &res);

    /**
     * @brief Convert a message from sensor_msgs::NavSatFix to asctec_msgs::GPSData.
     * @param req The message to convert together with the bearing of the GPS data.
     * @param res The converted message.
     * @return Whether the conversion succeeded.
     */
    bool fix_to_gpsdata (asctec_gps::FixToGpsdata::Request &req, asctec_gps::FixToGpsdata::Response &res);

    /**
     * @brief Get the GPS fix defining the local origin of this CPS.
     * @param req Empty request.
     * @param res The GPS coordinates first received by this CPS.
     * @return Whether the request succeeded.
     */
    bool get_gps_origin (cpswarm_msgs::GetGpsFix::Request &req, cpswarm_msgs::GetGpsFix::Response &res);

    /**
     * @brief Convert a message from asctec_msgs::GPSData to sensor_msgs::NavSatFix.
     * @param req The message to convert.
     * @param res The converted message.
     * @return Whether the conversion succeeded.
     */
    bool gpsdata_to_fix (asctec_gps::GpsdataToFix::Request &req, asctec_gps::GpsdataToFix::Response &res);

    /**
     * @brief Convert a message from asctec_msgs::GPSData to geometry_msgs::PoseStamped.
     * @param req The message to convert.
     * @param res The converted message.
     * @return Whether the conversion succeeded.
     */
    bool gpsdata_to_pose (asctec_gps::GpsdataToPose::Request &req, asctec_gps::GpsdataToPose::Response &res);

    /**
     * @brief Convert a yaw angle from north east down (NED) to east north up (ENU) coordinate system and vice versa.
     * @param req The yaw angle in radian to convert.
     * @param res The converted yaw angle in radian.
     * @return Whether the conversion succeeded.
     */
    bool ned_to_enu(cpswarm_msgs::NedToEnu::Request &req, cpswarm_msgs::NedToEnu::Response &res);

    /**
     * @brief Convert a message from geometry_msgs::PoseStamped to sensor_msgs::NavSatFix.
     * @param req The local message to convert.
     * @param res The converted global message.
     * @return Whether the conversion succeeded.
     */
    bool pose_to_fix (cpswarm_msgs::PoseToFix::Request &req, cpswarm_msgs::PoseToFix::Response &res);

    /**
     * @brief Convert a message from geometry_msgs::PoseStamped to asctec_msgs::GPSData.
     * @param req The local message to convert.
     * @param res The converted global message including yaw.
     * @return Whether the conversion succeeded.
     */
    bool pose_to_gpsdata (asctec_gps::PoseToGpsdata::Request &req, asctec_gps::PoseToGpsdata::Response &res);

private:
    /**
     * @brief Compute the great circle distance between two points.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The distance in meters.
     */
    double dist (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const;

    /**
     * @brief Compute the great circle distance between two points.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The distance in meters.
     */
    double dist (sensor_msgs::NavSatFix start, asctec_msgs::GPSData goal) const;

    /**
     * @brief Compute the great circle distance between two points.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The distance in meters.
     */
    double dist (asctec_msgs::GPSData start, sensor_msgs::NavSatFix goal) const;

    /**
     * @brief Compute the great circle distance between two points.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The distance in meters.
     */
    double dist (asctec_msgs::GPSData start, asctec_msgs::GPSData goal) const;

    /**
     * @brief Convert a message from sensor_msgs::NavSatFix to asctec_msgs::GPSData.
     * @param fix The message to convert.
     * @param yaw The bearing of the GPS data.
     * @return The converted message.
     */
    asctec_msgs::GPSData fix_to_gpsdata (sensor_msgs::NavSatFix fix, double yaw) const;

    /**
     * @brief Compute GPS coordinates of a goal point given starting point, distance, and yaw.
     * @param start The starting point GPS coordinates.
     * @param distance The distance in meters between start and destination.
     * @param yaw The initial yaw in radian, counterclockwise starting from east.
     * @return The goal GPS coordinates keeping the altitude of start.
     */
    sensor_msgs::NavSatFix goal (sensor_msgs::NavSatFix start, double distance, double yaw) const;

    /**
     * @brief Compute GPS coordinates of a goal point given starting point, distance, and yaw.
     * @param start The starting point GPS coordinates.
     * @param distance The distance in meters between start and destination.
     * @param yaw The initial yaw in radian, counterclockwise starting from east.
     * @return The goal GPS coordinates keeping the altitude of start.
     */
    sensor_msgs::NavSatFix goal (asctec_msgs::GPSData start, double distance, double yaw) const;

    /**
     * @brief Convert a message from asctec_msgs::GPSData to sensor_msgs::NavSatFix.
     * @param gpsdata The message to convert.
     * @return The converted message.
     */
    sensor_msgs::NavSatFix gpsdata_to_fix (asctec_msgs::GPSData gpsdata) const;

    /**
     * @brief Convert a message from asctec_msgs::GPSData to geometry_msgs::PoseStamped.
     * @param gpsdata The message to convert.
     * @return The converted message.
     */
    geometry_msgs::PoseStamped gpsdata_to_pose (asctec_msgs::GPSData gpsdata) const;

    /**
     * @brief Convert a yaw angle from north east down (NED) to east north up (ENU) coordinate system and vice versa.
     * @param yaw The yaw angle in radian to convert.
     * @return The converted yaw angle in radian.
     */
    double ned_to_enu(double yaw) const;

    /**
     * @brief Compute yaw angle in order to reach the goal.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The initial yaw in radian [0,2π].
     */
    double yaw (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const;

    /**
     * @brief Compute yaw angle in order to reach the goal.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The initial yaw in radian [0,2π].
     */
    double yaw (sensor_msgs::NavSatFix start, asctec_msgs::GPSData goal) const;

    /**
     * @brief Compute yaw angle in order to reach the goal.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The initial yaw in radian [0,2π].
     */
    double yaw (asctec_msgs::GPSData start, sensor_msgs::NavSatFix goal) const;

    /**
     * @brief Compute yaw angle in order to reach the goal.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The initial yaw in radian [0,2π].
     */
    double yaw (asctec_msgs::GPSData start, asctec_msgs::GPSData goal) const;

    /**
     * @brief Callback function to retrieve initial position.
     * @param msg Position received from the CPS.
     */
    void pose_callback (const asctec_msgs::GPSData::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The subscriber to receive the initial position of the CPS as localization origin.
     */
    Subscriber pose_sub;

    /**
     * @brief The GPS coordinates that where first received when starting this node.
     */
    sensor_msgs::NavSatFix origin;

    /**
     * @brief The earth radius in meters.
     */
    const long R = 6378137;
};

#endif // ASCTEC_GPS_LIB_H

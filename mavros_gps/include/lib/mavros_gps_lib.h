#ifndef MAVROS_GPS_LIB_H
#define MAVROS_GPS_LIB_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include "cpswarm_msgs/FixToPose.h"
#include "cpswarm_msgs/GetGpsFix.h"
#include "cpswarm_msgs/NedToEnu.h"
#include "cpswarm_msgs/PoseToFix.h"
#include "mavros_gps/FixToTarget.h"
#include "mavros_gps/PoseToTarget.h"
#include "mavros_gps/TargetToFix.h"

using namespace std;
using namespace ros;

/**
 * @brief A class that offers common operations required when working with GPS coordinates.
 */
class mavros_gps_lib
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    mavros_gps_lib ();

    /**
     * @brief Convert a message from sensor_msgs::NavSatFix to geometry_msgs::PoseStamped.
     * @param req The global message to convert.
     * @param res The converted local message without orientation
     * @return Whether the conversion succeeded.
     */
    bool fix_to_pose (cpswarm_msgs::FixToPose::Request &req, cpswarm_msgs::FixToPose::Response &res);

    /**
     * @brief Convert a message from sensor_msgs::NavSatFix to mavros_msgs::GlobalPositionTarget.
     * @param req The message to convert together with the bearing of the global position target.
     * @param res The converted message.
     * @return Whether the conversion succeeded.
     */
    bool fix_to_target (mavros_gps::FixToTarget::Request &req, mavros_gps::FixToTarget::Response &res);

    /**
     * @brief Get the GPS fix defining the local origin of this CPS.
     * @param req Empty request.
     * @param res The GPS coordinates first received by this CPS.
     * @return Whether the request succeeded.
     */
    bool get_gps_origin (cpswarm_msgs::GetGpsFix::Request &req, cpswarm_msgs::GetGpsFix::Response &res);

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
     * @brief Convert a message from geometry_msgs::PoseStamped to mavros_msgs::GlobalPositionTarget.
     * @param req The local message to convert.
     * @param res The converted global message including yaw.
     * @return Whether the conversion succeeded.
     */
    bool pose_to_target (mavros_gps::PoseToTarget::Request &req, mavros_gps::PoseToTarget::Response &res);

    /**
     * @brief Convert a message from mavros_msgs::GlobalPositionTarget to sensor_msgs::NavSatFix.
     * @param req The message to convert.
     * @param res The converted message.
     * @return Whether the conversion succeeded.
     */
    bool target_to_fix (mavros_gps::TargetToFix::Request &req, mavros_gps::TargetToFix::Response &res);

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
    double dist (sensor_msgs::NavSatFix start, mavros_msgs::GlobalPositionTarget goal) const;

    /**
     * @brief Compute the great circle distance between two points.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The distance in meters.
     */
    double dist (mavros_msgs::GlobalPositionTarget start, sensor_msgs::NavSatFix goal) const;

    /**
     * @brief Compute the great circle distance between two points.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The distance in meters.
     */
    double dist (mavros_msgs::GlobalPositionTarget start, mavros_msgs::GlobalPositionTarget goal) const;

    /**
     * @brief Convert a message from sensor_msgs::NavSatFix to mavros_msgs::GlobalPositionTarget.
     * @param fix The message to convert.
     * @param yaw The bearing of the global position target.
     * @return The converted message.
     */
    mavros_msgs::GlobalPositionTarget fix_to_target (sensor_msgs::NavSatFix fix, double yaw) const;

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
    sensor_msgs::NavSatFix goal (mavros_msgs::GlobalPositionTarget start, double distance, double yaw) const;

    /**
     * @brief Convert a yaw angle from north east down (NED) to east north up (ENU) coordinate system and vice versa.
     * @param yaw The yaw angle in radian to convert.
     * @return The converted yaw angle in radian.
     */
    double ned_to_enu(double yaw) const;

    /**
     * @brief Convert a message from mavros_msgs::GlobalPositionTarget to sensor_msgs::NavSatFix.
     * @param target The message to convert.
     * @return The converted message.
     */
    sensor_msgs::NavSatFix target_to_fix (mavros_msgs::GlobalPositionTarget target) const;

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
    double yaw (sensor_msgs::NavSatFix start, mavros_msgs::GlobalPositionTarget goal) const;

    /**
     * @brief Compute yaw angle in order to reach the goal.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The initial yaw in radian [0,2π].
     */
    double yaw (mavros_msgs::GlobalPositionTarget start, sensor_msgs::NavSatFix goal) const;

    /**
     * @brief Compute yaw angle in order to reach the goal.
     * @param start The starting point GPS coordinates.
     * @param goal The goal point GPS coordinates.
     * @return The initial yaw in radian [0,2π].
     */
    double yaw (mavros_msgs::GlobalPositionTarget start, mavros_msgs::GlobalPositionTarget goal) const;

    /**
     * @brief Callback function to retrieve initial position.
     * @param msg Position received from the CPS.
     */
    void pose_callback (const sensor_msgs::NavSatFix::ConstPtr& msg);

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

#endif // MAVROS_GPS_LIB_H

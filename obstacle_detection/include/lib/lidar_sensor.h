#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "sector.h"

using namespace std;
using namespace ros;

/**
 * @brief A helper class to process lidar sensor readings.
 */
class lidar_sensor
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param topic Topic for receiving sensor readings.
     */
    lidar_sensor (string topic);

    /**
     * @brief Check if the CPS is far enough from obstacles.
     * @return True if no obstacles are closer than avoidance distance, false otherwise.
     */
    bool clear () const;

    /**
     * @brief Check if the CPS is far enough from obstacles ahead.
     * @return True if no obstacles closer than avoidance distance or their bearing is outside of the heading angle of the CPS. False otherwise.
     */
    bool clear_ahead () const;

    /**
     * @brief Check if the CPS is dangerously close to obstacles.
     * @return The distance that the current CPS has to back off from the obstacle in order to reach a safe distance again. Returns 0.0 in case no obstacle is closer than the critical distance.
     */
    double danger () const;

    /**
     * @brief Get the absolute sector occupied by obstacles with added saftey bearing.
     * @return The inflated occupied sector.
     */
    sector inflated_region () const;

    /**
     * @brief Get the sector occupied by obstacles with added saftey bearing relative to the current heading.
     * @return The inflated occupied sector.
     */
    sector inflated_region (double heading) const;

    /**
     * @brief Get the absolute sector occupied by obstacles.
     * @return The occupied sector.
     */
    sector occupied_region () const;

    /**
     * @brief Get the sector occupied by obstacles relative to the current heading.
     * @return The occupied sector.
     */
    sector occupied_region (double heading) const;

private:
    /**
     * @brief Callback function to receive the sensor readings.
     * @param msg Laser scan data.
     */
    void callback (const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber for the sensor readings.
     */
    Subscriber subscriber;

    /**
     * @brief Latest sensor reading.
     */
    sensor_msgs::LaserScan data;

    /**
     * @brief Whether a valid laser scan has been received.
     */
    bool data_valid;

    /**
     * @brief The distance in meters to an obstacle below which the CPS starts its collision avoidance procedure.
     */
    double avoidance_dist;

    /**
     * @brief The distance in meters to an obstacle below which the CPS has to back off first before the standard avoidance procedure.
     */
    double critical_dist;

    /**
     * @brief The angle in radian that must be clear on each side around the CPS's movement direction.
     */
    double clear_ang;
};

#endif // LIDAR_SENSOR_H

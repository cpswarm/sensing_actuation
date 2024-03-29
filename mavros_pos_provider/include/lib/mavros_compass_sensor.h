#ifndef MAVROS_COMPASS_SENSOR_H
#define MAVROS_COMPASS_SENSOR_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace ros;

/**
 * @brief An implementation of the sensor class that describes a compass sensor. The yaw angle of the compass is measured in radian, counterclockwise starting from east.
 */
class mavros_compass_sensor
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    mavros_compass_sensor ();

    /**
     * @brief Get the computed reliable CPS yaw.
     * @return The yaw angle of the CPS.
     */
    double get_yaw ();

private:
    /**
     * @brief Process the msgs to compute a reliable yaw.
     */
    void process ();

    /**
     * @brief Callback function for sensor updates.
     * @param msg Compass heading received from the CPS's IMU.
     */
    void callback (const std_msgs::Float64::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber for the sensor readings.
     */
    Subscriber subscriber;

    /**
     * @brief The reliable yaw of the CPS computed by this class.
     */
    double yaw;

    /**
     * @brief Multiple sensor messages which are used to compute a reliable yaw.
     */
    vector<std_msgs::Float64> msgs;

    /**
     * @brief The position of the latest sensor message in the msgs vector.
     */
    unsigned int cur_msg;

    /**
     * @brief True when the msgs have been updated but no reliable yaw has been computed, false otherwise.
     */
    bool dirty;
};

#endif // MAVROS_COMPASS_SENSOR_H

#ifndef ASCTEC_COMPASS_SENSOR_H
#define ASCTEC_COMPASS_SENSOR_H

#include <ros/ros.h>
#include <asctec_msgs/IMUCalcData.h>

using namespace std;
using namespace ros;

/**
 * @brief An implementation of the sensor class that describes a compass sensor. The yaw angle of the compass is measured in radian, counterclockwise starting from east.
 */
class asctec_compass_sensor
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    asctec_compass_sensor ();

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
     * @param msg Magnetic field received from the CPS's IMU.
     */
    void callback (const asctec_msgs::IMUCalcData::ConstPtr& msg);

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
    vector<asctec_msgs::IMUCalcData> msgs;

    /**
     * @brief The position of the latest sensor message in the msgs vector.
     */
    unsigned int cur_msg;

    /**
     * @brief True when the msgs have been updated but no reliable yaw has been computed, false otherwise.
     */
    bool dirty;
};

#endif // ASCTEC_COMPASS_SENSOR_H

#ifndef RANGE_SENSOR_H
#define RANGE_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

using namespace std;
using namespace ros;

/**
 * @brief A helper class to process range sensor readings.
 */
class range_sensor
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param topic Topic for receiving sensor readings.
     * @param angle The angle at which the sensor is mounted in radians from the forward direction of the CPS.
     */
    range_sensor (string topic, double angle);

    /**
     * @brief Check if the CPS is dangerously close to an obstacle.
     * @return The distance that the CPS has to back off in order to reach a safe distance again. Returns 0.0 in case no obstacle is closer than the critical distance.
     */
    double danger ();

    /**
     * @brief Get the angle at which the sensor is looking.
     * @return The angle of the sensor.
     */
    double get_angle () const;

    /**
     * @brief Get the field of view of the sensor as read from the sensor messages.
     * @return The FOV of the sensor.
     */
    double get_fov () const;

    /**
     * @brief Get the computed reliable sensor range.
     * @return The sensor range.
     */
    double get_range ();

    /**
     * @brief Check if obstacles are detected by the range sensor.
     * @return True if there is an obstacle between min_dist and max_dist, false otherwise.
     */
    bool obstacle ();

private:
    /**
     * @brief Process the msgs to compute a reliable range.
     */
    void process ();

    /**
     * @brief Callback function for sensor readings.
     * @param msg Sensor message received from the UAV.
     */
    void callback (const sensor_msgs::Range::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber for sensor readings.
     */
    Subscriber subscriber;

    /**
     * @brief The horizontal angle at which the sensor is looking.
     */
    double angle;

    /**
     * @brief The field of view of the sensor as read from the sensor messages.
     */
    double fov;

    /**
     * @brief The reliable range of the sensor computed by this class.
     */
    double range;

    /**
     * @brief Multiple sensor messages which are used to compute a reliable range.
     */
    vector<sensor_msgs::Range> msgs;

    /**
     * @brief The position of the latest sensor message in the msgs vector.
     */
    unsigned int cur_msg;

    /**
     * @brief The minimum range at which obstacles are considered.
     */
    double min_dist;

    /**
     * @brief The maximum range at which obstacles are considered.
     */
    double max_dist;

    /**
     * @brief The distance in meters to an obstacle below which the CPS has to back off first before the standard avoidance procedure.
     */
    double critical_dist;

    /**
     * @brief Percentage of sensor messages that must report an obstacle in order for the behavior to assume an obstacle.
     */
    double threshold;

    /**
     * @brief True when the msgs have been updated but no reliable range has been computed, false otherwise.
     */
    bool dirty;
};

#endif // RANGE_SENSOR_H

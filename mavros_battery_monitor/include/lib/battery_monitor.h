#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ParamPull.h>
#include <cpswarm_msgs/BatteryState.h>

using namespace std;
using namespace ros;

/**
 * @brief A class that monitors the battery state of a CPS and calculates when it has to return for recharging.
 */
class battery_monitor
{
public:
    /**
     * @brief Constructor.
     */
    battery_monitor ();

    /**
     * @brief Calculate the remaining time the CPS can still operate.
     */
    void calculate ();

    /**
     * @brief Publish the calculated times.
     */
    void publish ();

private:
    /**
     * @brief Initialize the list of available charging points.
     */
    void init_cps ();

    /**
     * @brief Initialize the look-up-table for the battery state-of-charge from the voltage.
     */
    void init_lut ();

    /**
     * @brief Callback function to receive the current position of the CPS.
     * @param msg Position and orientation of the CPS.
     */
    void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief Callback function to receive the current battery state of the CPS.
     * @param msg The current battery state of the CPS.
     */
    void battery_callback (const sensor_msgs::BatteryState::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber to receive the current position of the CPS.
     */
    Subscriber pose_sub;

    /**
     * @brief Subscriber to receive the current battery state of the CPS.
     */
    Subscriber battery_sub;

    /**
     * @brief Publisher to transmit the estimated remaining run time of the CPS.
     */
    Publisher battery_pub;

    /**
     * @brief Current position of the CPS.
     */
    geometry_msgs::Pose pose;

    /**
     * @brief Whether a valid position has been received.
     */
    bool pose_valid;

    /**
     * @brief Current battery state.
     */
    sensor_msgs::BatteryState battery;

    /**
     * @brief Whether a valid battery state has been received.
     */
    bool battery_valid;

    /**
     * @brief A map with charging point IDs and positions.
     */
    map<int, geometry_msgs::Point> cps;

    /**
     * @brief Run time of the CPS in seconds.
     */
    int run_time;

    /**
     * @brief Spare time in seconds to keep as reserve when calculating the remaining run time.
     */
    int spare_time;

    /**
     * @brief A look-up-table for the battery state-of-charge from the battery voltage.
     */
    map<double, double> soc;

    /**
     * @brief The velocity for horizontal flight.
     */
    double vel_xy;

    /**
     * @brief The velocity for descend.
     */
    double vel_dn;

    /**
     * @brief The remaining time in seconds that the CPS can still operate.
     */
    int time_remaining_total;

    /**
     * @brief The remaining time in seconds that the CPS can still work before it has to return for recharing.
     */
    int time_remaining_work;

    /**
     * @brief The time in seconds the CPS needs for returning to the closest charging point.
     */
    int time_return;
};

#endif // BATTERY_MONITOR_H

#ifndef BATTERY_SIMULATOR_H
#define BATTERY_SIMULATOR_H

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

using namespace std;
using namespace ros;

class battery_simulator
{
public:
    /**
     * @brief Constructor.
     */
    battery_simulator ();

    /**
     * @brief TODO
     */
    void calculate ();

    void publish ();

private:
    /**
     * @brief Current battery state.
     */
    sensor_msgs::BatteryState state;
};

#endif // BATTERY_SIMULATOR_H

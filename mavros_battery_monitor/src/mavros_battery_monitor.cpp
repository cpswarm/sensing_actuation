#include "mavros_battery_monitor.h"

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv)
{
    // init ros node
    init(argc, argv, "battery_monitor");
    NodeHandle nh;

    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    bool simulation;
    nh.param(this_node::getName() + "/simulation", simulation, false);

	// monitor battery
    while (ok()) {
        spinOnce();
        // simulate battery
        // if (simulation) // TODO: implement
        //     simulator.simulate();

        // calculate remaining run time
        monitor.calculate();

        // publish remaining  run time
        monitor.publish();

        rate.sleep();
    }

    return 0;
}

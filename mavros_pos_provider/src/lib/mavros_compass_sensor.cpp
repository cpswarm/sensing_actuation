#include "lib/mavros_compass_sensor.h"

mavros_compass_sensor::mavros_compass_sensor ()
{
    // read parameters
    int num_msgs;
    nh.param(this_node::getName() + "/num_msgs", num_msgs, 1);

    // initialize parameters
    yaw = 0;
    cur_msg = 0;
    dirty = false;

    // subscribe sensor readings
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    subscriber = nh.subscribe("mavros/global_position/compass_hdg", queue_size, &mavros_compass_sensor::callback, this);

    // initialize range sensor messages
    for (int i = 0; i < num_msgs; ++i) {
        std_msgs::Float64 mag;
        msgs.push_back(mag);
    }
}

double mavros_compass_sensor::get_yaw ()
{
    // process new sensor messages
    if (dirty)
        process();

    return yaw;
}

void mavros_compass_sensor::process ()
{
    // compute the average yaw
    double total_x = 0, total_y = 0;
    for (int i = 0; i < msgs.size(); ++i) {
        total_x += cos(msgs[i].data / 180.0 * M_PI);
        total_y += sin(msgs[i].data / 180.0 * M_PI);
    }
    yaw = atan2(total_y, total_x);

    // all messages have been processed
    dirty = false;
}

void mavros_compass_sensor::callback (const std_msgs::Float64::ConstPtr& msg)
{
    // increase current position of message in vector
    cur_msg++;
    cur_msg %= msgs.size();

    // save range message
    msgs[cur_msg] = *msg;

    // sensor needs processing
    dirty = true;
}

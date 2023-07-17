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
    subscriber = nh.subscribe("mavros/imu/mag", queue_size, &mavros_compass_sensor::callback, this);

    // initialize range sensor messages
    for (int i = 0; i < num_msgs; ++i) {
        sensor_msgs::MagneticField mag;
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
        total_x += msgs[i].magnetic_field.x;
        total_y += msgs[i].magnetic_field.y;
    }
    yaw = atan2(total_y, total_x);

    // all messages have been processed
    dirty = false;
}

void mavros_compass_sensor::callback (const sensor_msgs::MagneticField::ConstPtr& msg)
{
    // increase current position of message in vector
    cur_msg++;
    cur_msg %= msgs.size();

    // save range message
    msgs[cur_msg] = *msg;

    // sensor needs processing
    dirty = true;
}

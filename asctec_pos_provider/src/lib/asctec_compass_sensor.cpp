#include "lib/asctec_compass_sensor.h"

asctec_compass_sensor::asctec_compass_sensor ()
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
    subscriber = nh.subscribe("IMU_CALC_DATA", queue_size, &asctec_compass_sensor::callback, this);

    // initialize range sensor messages
    for (int i = 0; i < num_msgs; ++i) {
        asctec_msgs::IMUCalcData mag;
        msgs.push_back(mag);
    }
}

double asctec_compass_sensor::get_yaw ()
{
    // process new sensor messages
    if (dirty)
        process();

    return yaw;
}

void asctec_compass_sensor::process ()
{
    // compute the average yaw
    double total_yaw = 0;
    for (int i = 0; i < msgs.size(); ++i) {
        total_yaw += double(msgs[i].mag_heading) / 1000 / 180 * M_PI;
    }
    yaw = total_yaw / double(msgs.size());

    // all messages have been processed
    dirty = false;
}

void asctec_compass_sensor::callback (const asctec_msgs::IMUCalcData::ConstPtr& msg)
{
    // increase current position of message in vector
    cur_msg++;
    cur_msg %= msgs.size();

    // save range message
    msgs[cur_msg] = *msg;

    // sensor needs processing
    dirty = true;
}
